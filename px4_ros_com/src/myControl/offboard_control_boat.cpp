/**
 * @brief 离线控制船只
 * @file offboard_control_boat.cpp
 * @addtogroup myControl
 * @author 徐立丹
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <unistd.h>
#include <poll.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControlBoat : public rclcpp::Node
{
public:
	OffboardControlBoat() : Node("offboard_control_boat"), 
		current_nav_state_(0),
		position_xy_des{0.0f, 0.0f},
		heading_ref_(0.0f),
		input_active_(false),
		position_initialized_(false),
		shutdown_keyboard_(false)
	{
		// 参数可通过launch或ROS参数服务器覆盖
		mass_ = this->declare_parameter("mass", 11.8f);
		inertia_z_ = this->declare_parameter("inertia_z", 0.6f);
		k1_position_gain_ = this->declare_parameter("k1_position_gain", 0.2f);
		k2_velocity_gain_ = this->declare_parameter("k2_velocity_gain", 0.5f);
		k3_attitude_gain_ = this->declare_parameter("k3_attitude_gain", 2.0f);
		k4_rate_gain_ = this->declare_parameter("k4_rate_gain", 2.0f);
		max_speed_ = this->declare_parameter("max_speed", 0.8f);
		max_thrust_ = this->declare_parameter("max_thrust", 6.0f);
		control_threshold_ = this->declare_parameter("control_threshold", 2.0f);
		yaw_rate_limit_ = this->declare_parameter("yaw_rate_limit", 0.3f);
		heading_tolerance_rad_ = static_cast<float>(this->declare_parameter("heading_tolerance_deg", 10.0) * 3.14159265358979323846 / 180.0);
		T_test_ = this->declare_parameter("T_test", 2.0f);
		
		// 打印所有加载的参数值以供调试
		RCLCPP_INFO(this->get_logger(), "参数加载完成:");
		RCLCPP_INFO(this->get_logger(), "  mass: %.2f", mass_);
		RCLCPP_INFO(this->get_logger(), "  inertia_z: %.2f", inertia_z_);
		RCLCPP_INFO(this->get_logger(), "  k1_position_gain: %.2f", k1_position_gain_);
		RCLCPP_INFO(this->get_logger(), "  k2_velocity_gain: %.2f", k2_velocity_gain_);
		RCLCPP_INFO(this->get_logger(), "  k3_attitude_gain: %.2f", k3_attitude_gain_);
		RCLCPP_INFO(this->get_logger(), "  k4_rate_gain: %.2f", k4_rate_gain_);
		RCLCPP_INFO(this->get_logger(), "  max_speed: %.2f", max_speed_);
		RCLCPP_INFO(this->get_logger(), "  max_thrust: %.2f", max_thrust_);
		RCLCPP_INFO(this->get_logger(), "  control_threshold: %.2f", control_threshold_);
		RCLCPP_INFO(this->get_logger(), "  yaw_rate_limit: %.2f", yaw_rate_limit_);
		RCLCPP_INFO(this->get_logger(), "  heading_tolerance_deg: %.2f", heading_tolerance_rad_ * 180.0f / 3.14159265358979323846f);
		RCLCPP_INFO(this->get_logger(), "  T_test: %.2f", T_test_);

		auto parameter_callback = [this](const std::vector<rclcpp::Parameter> &parameters) {
			rcl_interfaces::msg::SetParametersResult result;
			result.successful = true;
			for (const auto &param : parameters) {
				const auto &name = param.get_name();
				double value = param.as_double();
				if (name == "mass") {
					if (value <= 0.0) { result.successful = false; result.reason = "mass must be positive"; break; }
					mass_ = static_cast<float>(value);
				} else if (name == "inertia_z") {
					if (value <= 0.0) { result.successful = false; result.reason = "inertia_z must be positive"; break; }
					inertia_z_ = static_cast<float>(value);
				} else if (name == "k1_position_gain") {
					k1_position_gain_ = static_cast<float>(value);
				} else if (name == "k2_velocity_gain") {
					k2_velocity_gain_ = static_cast<float>(value);
				} else if (name == "k3_attitude_gain") {
					k3_attitude_gain_ = static_cast<float>(value);
				} else if (name == "k4_rate_gain") {
					k4_rate_gain_ = static_cast<float>(value);
				} else if (name == "max_speed") {
					if (value <= 0.0) { result.successful = false; result.reason = "max_speed must be positive"; break; }
					max_speed_ = static_cast<float>(value);
				} else if (name == "max_thrust") {
					if (value <= 0.0) { result.successful = false; result.reason = "max_thrust must be positive"; break; }
					max_thrust_ = static_cast<float>(value);
				} else if (name == "control_threshold") {
					if (value <= 0.0) { result.successful = false; result.reason = "control_threshold must be positive"; break; }
					control_threshold_ = static_cast<float>(value);
				} else if (name == "yaw_rate_limit") {
					if (value <= 0.0) { result.successful = false; result.reason = "yaw_rate_limit must be positive"; break; }
					yaw_rate_limit_ = static_cast<float>(value);
				} else if (name == "heading_tolerance_deg") {
					if (value <= 0.0) { result.successful = false; result.reason = "heading_tolerance_deg must be positive"; break; }
					heading_tolerance_rad_ = static_cast<float>(value * 3.14159265358979323846 / 180.0);
				} else if (name == "T_test") {
					if (0) { result.successful = false; result.reason = "T_test must be positive"; break; }
					T_test_ = static_cast<float>(value);
				}
			}
			return result;
		};
		parameter_callback_handle_ = this->add_on_set_parameters_callback(parameter_callback);

		// 创建发布器
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		vehicle_thrust_setpoint_publisher_ = this->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);
		vehicle_torque_setpoint_publisher_ = this->create_publisher<VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// xld 2025-12-29: 订阅VehicleStatus以检查nav_state，存入current_nav_state_
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		vehicle_status_subscription_ = this->create_subscription<VehicleStatus>(
			"/fmu/out/vehicle_status_v1",
			qos,
			[this](const VehicleStatus::SharedPtr msg) {
				uint8_t old_state = current_nav_state_;
				current_nav_state_ = msg->nav_state;
				if (old_state != current_nav_state_) {
					RCLCPP_INFO(this->get_logger(), "Nav state changed: %d -> %d (Offboard=14)", old_state, current_nav_state_);
				}
			});
		
		// 订阅VehicleOdometry以获取船的位姿及速度
		vehicle_odometry_subscription_ = this->create_subscription<VehicleOdometry>(
			"/fmu/out/vehicle_odometry",
			qos,
			[this](const VehicleOdometry::SharedPtr msg) {
				// 如有需要，可在此处使用本地位置信息
				position_[0] = msg->position[0];
				position_[1] = msg->position[1];
				position_[2] = msg->position[2];
				velocity_[0] = msg->velocity[0];
				velocity_[1] = msg->velocity[1];
				velocity_[2] = msg->velocity[2];
				attitude_quat_[0] = msg->q[0];
				attitude_quat_[1] = msg->q[1];
				attitude_quat_[2] = msg->q[2];
				attitude_quat_[3] = msg->q[3];
				angular_velocity_[0] = msg->angular_velocity[0];
				angular_velocity_[1] = msg->angular_velocity[1];
				angular_velocity_[2] = msg->angular_velocity[2];
				// 首次收到位置信息时，如果还没有键盘输入，则将目标位置初始化为当前位置
				if (!position_initialized_ && !input_active_) {
					std::lock_guard<std::mutex> lock(position_mutex_);
					position_xy_des = Eigen::Vector2f(msg->position[0], msg->position[1]);
				// 初始化期望航向为当前航向
				float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
				heading_ref_ = std::atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
				position_initialized_ = true;
				RCLCPP_INFO(this->get_logger(), "目标位置已初始化为当前位置: x=%.2f, y=%.2f, 航向=%.1f°", 
					msg->position[0], msg->position[1], heading_ref_ * 180.0f / 3.14159f);
				}
			});

		// 启动键盘监听线程
		keyboard_thread_ = std::thread(&OffboardControlBoat::keyboard_input_thread, this);
		RCLCPP_INFO(this->get_logger(), "键盘控制已启动。输入格式: x y (例如: 5.0 3.0)\n按空格键暂停/继续控制");

		// 单次1秒定时器：切换到Offboard并解锁
		offboard_switch_timer_ = this->create_wall_timer(1s, [this]() {
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			this->arm();
			RCLCPP_INFO(this->get_logger(),  "开始1秒后已经发送offboard切换和解锁指令");
			offboard_switch_timer_->cancel();
		});

		// 100Hz控制循环定时器
		auto timer_callback = [this]() -> void {
			if (current_nav_state_ == 14) {
					publish_thrust_torque_setpoint();
			} else {
				static int warn_counter = 0;
				if (++warn_counter % 300 == 0) {  // 先加一，再整除，每3秒输出一次
					RCLCPP_WARN(this->get_logger(), "未处于Offboard模式 (nav_state=%d)，跳过推力/力矩发布", current_nav_state_);
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					RCLCPP_INFO(this->get_logger(), "已发送切换到Offboard模式的指令");
				}
			}
		};
		timer_ = this->create_wall_timer(10ms, timer_callback);

		// 10Hz心跳：只发布offboard_control_mode
		offboard_heartbeat_timer_ = this->create_wall_timer(100ms, [this]() {
			publish_offboard_control_mode();
		});
	}

	~OffboardControlBoat() {
		// 停止键盘线程
		shutdown_keyboard_ = true;
		if (keyboard_thread_.joinable()) {
			keyboard_thread_.join();
		}
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr offboard_heartbeat_timer_;
	rclcpp::TimerBase::SharedPtr offboard_switch_timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr vehicle_thrust_setpoint_publisher_;
	rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr vehicle_torque_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
	uint8_t current_nav_state_;   //!< 当前无人艇导航状态
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscription_;
	float_t position_[3];   //!< 当前本地位置
	float_t velocity_[3];   //!< 当前本地速度
	float_t attitude_quat_[4];   //!< 当前姿态四元数
	float_t angular_velocity_[3];   //!< 当前角速度
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
	float mass_{};
	float inertia_z_{};
	float k1_position_gain_{};
	float k2_velocity_gain_{};
	float k3_attitude_gain_{};
	float k4_rate_gain_{};
	float max_speed_{};
	float max_thrust_{};
	float control_threshold_{};
	float yaw_rate_limit_{};
	float heading_tolerance_rad_{};
	float T_test_{};

	// 键盘输入相关
	Eigen::Vector2f position_xy_des;   //!< 目标位置 (x, y)
	float heading_ref_;                //!< 平滑的期望航向
	std::atomic<bool> input_active_;    //!< 是否有新的键盘输入
	std::atomic<bool> position_initialized_;  //!< 位置是否已初始化
	std::atomic<bool> shutdown_keyboard_;  //!< 键盘线程关闭标志
	std::thread keyboard_thread_;       //!< 键盘输入线程
	std::mutex position_mutex_;         //!< 保护目标位置的互斥锁

	void keyboard_input_thread();       //!< 键盘输入线程函数
	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_thrust_torque_setpoint();
};

// 解锁无人艇
void OffboardControlBoat::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

// 上锁无人艇
void OffboardControlBoat::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

// 发布offboard控制模式指令
void OffboardControlBoat::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = true;
    msg.direct_actuator = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

// 发布推力和力矩设定值，仅在offboard模式下
void OffboardControlBoat::publish_thrust_torque_setpoint()
{
	float q0 = attitude_quat_[0];  // w
	float q1 = attitude_quat_[1];  // x
	float q2 = attitude_quat_[2];  // y
	float q3 = attitude_quat_[3];  // z
	
	// Yaw (z-axis rotation)
	float psi = std::atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
	Eigen::Matrix2f R;
	R << std::cos(psi), -std::sin(psi),
		 std::sin(psi),  std::cos(psi);
	
	Eigen::Vector2f position_xy(position_[0], position_[1]);
	float distance_to_target = (position_xy - position_xy_des).norm();
	
		// 计算指向目标的期望旋转矩阵
	Eigen::Matrix2f R_des;
	// Eigen::Vector2f R1_des = (position_xy_des - position_xy).normalized();
	Eigen::Vector2f R1_des = {0,1};
	double psi_des = std::atan2(R1_des.y(), R1_des.x());
	Eigen::Matrix2f R_90;
	R_90 << 0.0f, -1.0f,
	        1.0f,  0.0f;
	Eigen::Vector2f R2_des = R_90 * R1_des;
	R_des.col(0) = R1_des;
	R_des.col(1) = R2_des;

	float Tauz = 0.0f;
	
	// 如果姿态误差小于3度，则不进行转向控制
	Eigen::Vector2f R1 = R.col(0);
	Eigen::Matrix2f R_5_p;
	R_5_p << std::cos(5.0f * M_PI / 180.0), -std::sin(5.0f * M_PI / 180.0),
	        std::sin(5.0f * M_PI / 180.0),  std::cos(5.0f * M_PI / 180.0);
	Eigen::Matrix2f R_5_n;
	R_5_n << std::cos(-5.0f * M_PI / 180.0), -std::sin(-5.0f * M_PI / 180.0),
	        std::sin(-5.0f * M_PI / 180.0),  std::cos(-5.0f * M_PI / 180.0);
	Eigen::Vector2f R1_des_r = R_5_p * R1_des;
	Eigen::Vector2f R2_des_r;
	Eigen::Vector2f R1_des_l = R_5_n * R1_des;
	Eigen::Vector2f R2_des_l;
	Eigen::Matrix2f error_R;
	float r_ref;
	float r_ref_lim;
	float r = angular_velocity_[2]; // 当前偏航角速度
	float cos_angle = R1.dot(R1_des);
	float angle;
	if (cos_angle > std::cos(5.0 * M_PI / 180.0)) {
		Tauz = 0.0f;
	}else if (R1.dot(R1_des_r) > R1.dot(R1_des_l)) {
		// 右转更近
		R2_des_r = R_90 * R1_des_r;
		R_des.col(0) = R1_des_r;
		R_des.col(1) = R2_des_r;
		error_R = R.transpose() * R_des - R_des.transpose() * R;
		angle = abs(acos(R1_des_r.dot(R1)));
		r_ref = k3_attitude_gain_ * std::copysign(1.0f, error_R(1,0)) * angle; // 期望偏航角速度
		r_ref_lim = yaw_rate_limit_; // rad/s
		if (r_ref > r_ref_lim) r_ref = r_ref_lim;
		if (r_ref < -r_ref_lim) r_ref = -r_ref_lim;
		Tauz = inertia_z_ * (-k4_rate_gain_ * (r - r_ref));
	} else {
		// 左转更近
		R2_des_l = R_90 * R1_des_l;
		R_des.col(0) = R1_des_l;
		R_des.col(1) = R2_des_l;
		error_R = R.transpose() * R_des - R_des.transpose() * R;
		angle = abs(acos(R1_des_l.dot(R1)));
		r_ref = k3_attitude_gain_ * std::copysign(1.0f, error_R(1,0)) * angle; // 期望偏航角速度
		r_ref_lim = yaw_rate_limit_; // rad/s
		if (r_ref > r_ref_lim) r_ref = r_ref_lim;
		if (r_ref < -r_ref_lim) r_ref = -r_ref_lim;
		r = angular_velocity_[2]; // 当前偏航角速度
		Tauz = inertia_z_ * (-k4_rate_gain_ * (r - r_ref));
	}

	// 如果在2米范围内，不触发控制，直接发布零推力和零力矩
	if (distance_to_target < control_threshold_) {
		
		VehicleThrustSetpoint msg_thrust{};
		msg_thrust.xyz = {0.0, 0.0, 0.0};
		msg_thrust.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		VehicleTorqueSetpoint msg_torque{};
		msg_torque.xyz = {0.0, 0.0, 0.0};
		msg_torque.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_thrust_setpoint_publisher_->publish(msg_thrust);
		vehicle_torque_setpoint_publisher_->publish(msg_torque);
		
		// 调试输出（每2秒输出一次）
		static int debug_counter_idle = 0;
		if (++debug_counter_idle % 200 == 0) {
			RCLCPP_INFO(this->get_logger(), 
				"位置:(%.2f,%.2f) 目标:(%.2f,%.2f) 距离:%.2fm < %.2fm - 在控制死区内，保持静止\n 力矩:%.2f",
				position_[0], position_[1], 
				position_xy_des.x(), position_xy_des.y(),
				distance_to_target, control_threshold_, Tauz);
		}
		return;
	}
	
	// // 计算指向目标的期望旋转矩阵
	// Eigen::Matrix2f R_des;
	// // Eigen::Vector2f R1_des = (position_xy_des - position_xy).normalized();
	// Eigen::Vector2f R1_des = {1,0};
	// double psi_des = std::atan2(R1_des.y(), R1_des.x());
	// Eigen::Matrix2f R_90;
	// R_90 << 0.0f, -1.0f,
	//         1.0f,  0.0f;
	// Eigen::Vector2f R2_des = R_90 * R1_des;
	// R_des.col(0) = R1_des;
	// R_des.col(1) = R2_des;
	
	// // === 转向控制（优先级最高）===
	// // 限制期望偏航角速度，避免近距离过度摇摆
	// Eigen::Matrix2f error_R = R.transpose() * R_des - R_des.transpose() * R;
	// float r_ref = k3_attitude_gain_ * 0.5f * error_R(1,0); // 期望偏航角速度
	// float r_ref_lim = yaw_rate_limit_; // rad/s
	// if (r_ref > r_ref_lim) r_ref = r_ref_lim;
	// if (r_ref < -r_ref_lim) r_ref = -r_ref_lim;
	// float r = angular_velocity_[2]; // 当前偏航角速度
	// float Tauz = inertia_z_ * (-k4_rate_gain_ * (r - r_ref));
	
	// === 推力控制 ===
	float T = 0.0f;
	
	// 只有当航向误差在±10°以内才施加推力
	if (std::abs(error_R(1,0)) < heading_tolerance_rad_) {
		// 计算期望速度（沿目标方向）
		Eigen::Vector2f error_xy = position_xy_des - position_xy;
		Eigen::Vector2f velocity_xy_ref = k1_position_gain_ * error_xy;
		
		// 限制期望速度大小
		float max_speed = max_speed_;
		if (velocity_xy_ref.norm() > max_speed) {
			velocity_xy_ref = velocity_xy_ref.normalized() * max_speed;
		}
		
		// 计算推力（速度控制）
		Eigen::Vector2f velocity_xy(velocity_[0], velocity_[1]);
		Eigen::Vector2f force_desired = mass_ * k2_velocity_gain_ * (velocity_xy_ref - velocity_xy);
		
		// 推力投影到船头方向
		Eigen::Vector2f heading_vec(std::cos(psi), std::sin(psi));
		T = force_desired.dot(heading_vec);
		
		// 限制推力为正值
		T = std::max(0.0f, T);
		
		// 限制最大推力
		float max_thrust = max_thrust_;
		T = std::min(T, max_thrust);
	}

	VehicleThrustSetpoint msg_thrust{};
	if (T > 0.001f) {
		T = std::max(T, 0.12f);
	} else if (T < -0.001f)
	{
		T = std::min(T, -0.12f);
	}
	msg_thrust.xyz = {0.0, 0.0, 0.0};
	msg_thrust.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	VehicleTorqueSetpoint msg_torque{};
	if (Tauz > 0.001f) {
		Tauz = std::max(Tauz, 0.16f);
	} else if (Tauz < -0.001f)
	{
		Tauz = std::min(Tauz, -0.16f);
	}
	msg_torque.xyz = {0.0, 0.0, Tauz};
	msg_torque.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_thrust_setpoint_publisher_->publish(msg_thrust);
	vehicle_torque_setpoint_publisher_->publish(msg_torque);

	// 调试输出（每2秒输出一次）
	static int debug_counter = 0;
	if (++debug_counter % 200 == 0) {
		RCLCPP_INFO(this->get_logger(), 
			"位置:(%.2f,%.2f) 目标:(%.2f,%.2f) 距离:%.2f 推力:%.2f \n 偏航角:%.2f 期望偏航角:%.2f 角速度:%.2f 期望角速度:%.2f 力矩:%.2f",
			position_[0], position_[1], 
			position_xy_des.x(), position_xy_des.y(),
			distance_to_target,
			T, 
			psi*180.0f/3.14159f, psi_des*180.0f/3.14159f,
			angular_velocity_[2], r_ref, Tauz);
	}
}

// 发布无人艇指令
void OffboardControlBoat::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

//键盘输入线程函数：持续监听键盘输入并解析x、y坐标
void OffboardControlBoat::keyboard_input_thread()
{
	std::string line;
	std::cout << "\n请输入目标位置 (x y): ";
	std::cout.flush();
	
	while (!shutdown_keyboard_ && rclcpp::ok()) {
		// 使用poll检查标准输入是否有数据（非阻塞）
		struct pollfd fds;
		fds.fd = STDIN_FILENO;
		fds.events = POLLIN;
		
		// 等待100ms检查输入
		int ret = poll(&fds, 1, 100);
		
		if (ret > 0 && (fds.revents & POLLIN)) {
			// 有数据可用
			if (std::getline(std::cin, line)) {
				if (line.empty()) {
					std::cout << "\n请输入目标位置 (x y): ";
					std::cout.flush();
					continue;
				}
				
				std::istringstream iss(line);
				float x, y;
				
				if (iss >> x >> y) {
					{
						std::lock_guard<std::mutex> lock(position_mutex_);
						position_xy_des = Eigen::Vector2f(x, y);
						input_active_ = true;
					}
					RCLCPP_INFO(this->get_logger(), "\n目标位置已更新: x=%.2f, y=%.2f", x, y);
				} else {
					RCLCPP_WARN(this->get_logger(), "\n输入格式错误！请使用格式: x y (例如: 5.0 3.0)");
				}
				
				std::cout << "\n请输入目标位置 (x y): ";
				std::cout.flush();
			}
		}
	}
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	
	auto node = std::make_shared<OffboardControlBoat>();
	
	// 使用自定义信号处理实现优雅关闭
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	
	// 在独立线程中运行executor以响应Ctrl+C
	std::thread spin_thread([&executor]() {
		executor.spin();
	});
	
	// 等待关闭信号
	spin_thread.join();
	
	std::cout << "\nShutting down..." << std::endl;
	rclcpp::shutdown();
	return 0;
}



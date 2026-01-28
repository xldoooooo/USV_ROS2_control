/**
 * @brief 离线控制船只 - 主实现文件
 * @file offboard_control_boat.cpp
 * @addtogroup myControl
 * @author 徐立丹
 */

#include "offboard_control_boat.hpp"

// 无人艇控制节点构造函数
OffboardControlBoat::OffboardControlBoat() : Node("offboard_control_boat"), 
		current_nav_state_(0),
		position_xy_des_{0.0f, 0.0f},
		position_initialized_(false),
		waiting_for_input_(false),
		shutdown_keyboard_(false),
		current_mode_(BoatControlMode::IDLE)
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
	idle_threshold_ = this->declare_parameter("idle_threshold", 2.0f);
	area_threshold_ = this->declare_parameter("area_threshold", 5.0f);
	yaw_rate_limit_ = this->declare_parameter("yaw_rate_limit", 0.3f);
	heading_tolerance_rad_ = static_cast<float>(this->declare_parameter("heading_tolerance_deg", 5.0) * 3.14159265358979323846 / 180.0);
	T_test_ = this->declare_parameter("T_test", -0.16f);
	
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
	RCLCPP_INFO(this->get_logger(), "  idle_threshold: %.2f", idle_threshold_);
	RCLCPP_INFO(this->get_logger(), "  area_threshold: %.2f", area_threshold_);
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
			} else if (name == "idle_threshold") {
				if (value <= 0.0) { result.successful = false; result.reason = "idle_threshold must be positive"; break; }
				idle_threshold_ = static_cast<float>(value);
			} else if (name == "area_threshold") {
				if (value <= 0.0) { result.successful = false; result.reason = "area_threshold must be positive"; break; }
				area_threshold_ = static_cast<float>(value);
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
		}
	);
	
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
			if (!position_initialized_) {
				std::lock_guard<std::mutex> lock(position_des_mutex_);
				position_xy_des_ = Eigen::Vector2f(msg->position[0], msg->position[1]);
				// 初始化期望航向为当前航向
				float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
				float yaw0 = std::atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
				position_initialized_ = true;
				RCLCPP_INFO(this->get_logger(), "目标位置已初始化为当前位置: x=%.2f, y=%.2f, 航向=%.1f°", 
					msg->position[0], msg->position[1], yaw0 * 180.0f / 3.14159f);
			}
		}
	);

	// 单次1秒定时器：切换到Offboard并解锁
	offboard_switch_timer_ = this->create_wall_timer(1s, [this]() {
		if (current_nav_state_ != 14) {
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			this->arm();
		} else {
			RCLCPP_INFO(this->get_logger(), "已经处于Offboard模式");
			offboard_switch_timer_->cancel();
		}
	});
	
	// 10Hz FSM状态机定时器
	FSM_timer_ = this->create_wall_timer(100ms, [this]() {
		if (current_nav_state_ == 14) {
			this->fsm_update();
		}
	});

	// 10Hz心跳定时器：定期发布offboard_control_mode保持模式激活
	offboard_heartbeat_timer_ = this->create_wall_timer(100ms, [this]() {
		publish_offboard_control_mode();
	});

	// 100Hz控制循环定时器
	control_loop_timer_ = this->create_wall_timer(10ms, [this]() {
		if (current_nav_state_ == 14) {
			publish_vehicle_thrust_torque_setpoint();
		}
	});

	// 启动键盘输入线程
	keyboard_thread_ = std::thread(&OffboardControlBoat::keyboard_input_thread, this);
	RCLCPP_INFO(this->get_logger(), "键盘输入线程已启动，按空格键触发坐标输入\n");
}

// 无人艇控制节点析构函数
OffboardControlBoat::~OffboardControlBoat() {
	// 停止键盘线程
	shutdown_keyboard_ = true;
	if (keyboard_thread_.joinable()) {
		keyboard_thread_.join();
	}
}

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

// 发布无人艇指令(解锁/上锁等)
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

// 发布期望推力和力矩指令
void OffboardControlBoat::publish_vehicle_thrust_torque_setpoint()
{
	switch (current_mode_) {
		case BoatControlMode::IDLE:
			mode_idle();
			break;
		case BoatControlMode::HEADING_STABILIZATION:
			mode_heading_stabilization();
			break;
		case BoatControlMode::POINT_STABILIZATION:
			mode_point_stabilization();
			break;
		default:
			mode_idle();
			break;
	}
	VehicleThrustSetpoint msg_thrust{};
	VehicleTorqueSetpoint msg_torque{};

	msg_thrust.xyz = {Tx_sp, 0.0, 0.0};
	msg_torque.xyz = {0.0, 0.0, Tauz_sp};

	msg_thrust.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg_torque.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	vehicle_thrust_setpoint_publisher_->publish(msg_thrust);
	vehicle_torque_setpoint_publisher_->publish(msg_torque);	
}

// 主函数
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



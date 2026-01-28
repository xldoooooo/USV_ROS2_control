/**
 * @brief 离线控制船只 - 头文件
 * @file offboard_control_boat.hpp
 * @addtogroup myControl
 * @author 徐立丹
 */

#ifndef OFFBOARD_CONTROL_BOAT_HPP
#define OFFBOARD_CONTROL_BOAT_HPP

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
#include <termios.h>
#include <Eigen/Dense>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#pragma GCC diagnostic pop

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// 船的运动模式枚举
enum class BoatControlMode {
	IDLE,                  // 空闲模式（不控制）
    HEADING_STABILIZATION, // 航向稳定模式：只控制当前航向
	POINT_STABILIZATION,   // 点镇定模式：从当前位置驶向指定点
};

class OffboardControlBoat : public rclcpp::Node
{
public:
	OffboardControlBoat();
	~OffboardControlBoat();
	void arm();
	void disarm();

private:
    rclcpp::TimerBase::SharedPtr offboard_switch_timer_;    // 1Hz 离线控制模式切换
    rclcpp::TimerBase::SharedPtr offboard_heartbeat_timer_; // 10Hz 离线控制心跳
	rclcpp::TimerBase::SharedPtr FSM_timer_;                // 10Hz FSM状态机定时器
    rclcpp::TimerBase::SharedPtr control_loop_timer_;       // 100Hz 控制循环
    

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr vehicle_thrust_setpoint_publisher_;
	rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr vehicle_torque_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscription_;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    
    uint8_t current_nav_state_;     //!< 当前无人艇导航状态
    float_t position_[3];           //!< 当前本地位置
	float_t velocity_[3];           //!< 当前本地速度
	float_t attitude_quat_[4];      //!< 当前姿态四元数
	float_t angular_velocity_[3];   //!< 当前角速度
    float_t Tx_sp;                  //!< 期望推力
    float_t Tauz_sp;                //!< 期望力矩
    float_t distance_to_target;     //!< 当前到目标位置的距离
    float_t psi;                    //!< 当前偏航角（弧度）
    float_t psi_des;                //!< 期望偏航角（弧度）

    // 船只参数
	float mass_{};
	float inertia_z_{};
	float k1_position_gain_{};
	float k2_velocity_gain_{};
	float k3_attitude_gain_{};
	float k4_rate_gain_{};
	float max_speed_{};
	float max_thrust_{};
    float idle_threshold_{};
    float area_threshold_{};
	float yaw_rate_limit_{};
	float heading_tolerance_rad_{};
	float T_test_{};

	// 键盘输入相关
	Eigen::Vector2f position_xy_des_;        //!< 目标位置 (x, y)
	std::atomic<bool> position_initialized_; //!< 位置是否已初始化
	std::atomic<bool> waiting_for_input_;    //!< 是否正在等待用户输入坐标
	std::atomic<bool> shutdown_keyboard_;    //!< 键盘线程关闭标志
	std::thread keyboard_thread_;            //!< 键盘输入线程
	std::mutex position_des_mutex_;          //!< 保护目标位置的互斥锁

	// === 运动模式相关 ===
	BoatControlMode current_mode_;        //!< 当前运动模式
	std::mutex mode_mutex_;               //!< 保护模式切换的互斥锁
    void fsm_update();                    //!< FSM状态机更新函数
    void keyboard_input_thread();         //!< 键盘输入线程函数
    void mode_idle();                     //!< 空闲模式
    void mode_heading_stabilization();    //!< 航向控制模式
	void mode_point_stabilization();      //!< 定点控制模式

    void publish_vehicle_thrust_torque_setpoint();    // 定时控制函数，实现期望力和力矩的发布（100Hz）
	void publish_offboard_control_mode();             //!< 发布离线控制模式
	void publish_vehicle_command(uint16_t command, 
                                 float param1 = 0.0, 
                                 float param2 = 0.0); //!< 发布车辆命令
	
};

#endif // OFFBOARD_CONTROL_BOAT_HPP

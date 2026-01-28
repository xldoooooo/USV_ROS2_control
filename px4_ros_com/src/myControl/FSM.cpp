/**
 * @brief 离线控制船只 - 有限状态机
 * @file FSM.cpp
 * @addtogroup myControl
 * @author 徐立丹
 * 
 * 此文件包含有限状态机的主要逻辑
 * 负责根据当前状态和条件自动切换控制模式
 */

#include "offboard_control_boat.hpp"

/*
 * @brief 有限状态机更新函数，10Hz执行一次
 * 根据当前状态和距离等条件自动切换控制模式
 */
void OffboardControlBoat::fsm_update()
{
	// ========== FSM 状态转移逻辑 ==========
	float q0 = attitude_quat_[0];  // w
	float q1 = attitude_quat_[1];  // x
	float q2 = attitude_quat_[2];  // y
	float q3 = attitude_quat_[3];  // z

	// Yaw (z-axis rotation)
	psi = std::atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
	Eigen::Matrix2f R;
	R << std::cos(psi), -std::sin(psi),
		std::sin(psi),  std::cos(psi);
	Eigen::Vector2f position_xy_(position_[0], position_[1]);

	// 计算指向目标的单位向量
	Eigen::Vector2f R1_des = (position_xy_des_ - position_xy_).normalized();
	distance_to_target = (position_xy_des_ - position_xy_).norm();
	float cos_angle = R1_des.dot(R.col(0));
	psi_des = std::atan2(R1_des.y(), R1_des.x());

	// 根据当前模式执行相应的控制逻辑
	std::lock_guard<std::mutex> lock(mode_mutex_);
	switch(current_mode_) {
		case BoatControlMode::IDLE: // 空闲模式
			if (distance_to_target > area_threshold_) {
				RCLCPP_WARN(this->get_logger(), "距离目标位置较远 (%.2f m)，将切换为定点控制模式", distance_to_target);
				current_mode_ = BoatControlMode::HEADING_STABILIZATION;
				RCLCPP_INFO(this->get_logger(), "运动模式已切换为: HEADING_STABILIZATION（航向控制）");
			}
			break;
		case BoatControlMode::HEADING_STABILIZATION: // 航向控制模式
			if (cos_angle > std::cos(heading_tolerance_rad_)) {
				RCLCPP_INFO(this->get_logger(), "已对准目标方向，切换为定点控制模式");
				current_mode_ = BoatControlMode::POINT_STABILIZATION;
				RCLCPP_INFO(this->get_logger(), "运动模式已切换为: POINT_STABILIZATION（定点控制）");
			}
			break;
		case BoatControlMode::POINT_STABILIZATION: // 定点控制模式
			if (distance_to_target <= idle_threshold_) {
				RCLCPP_INFO(this->get_logger(), "已到达目标位置 (%.2f m)，切换为空闲模式", distance_to_target);
				current_mode_ = BoatControlMode::IDLE;
				RCLCPP_INFO(this->get_logger(), "运动模式已切换为: IDLE（空闲）");
			}
			break;
		default:
			RCLCPP_ERROR(this->get_logger(), "未知的运动模式！");
			break;
	}
}

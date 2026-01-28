/**
 * @brief 离线控制船只 - 推力/力矩设定值发布
 * @file offboard_control_boat_thrust_torque.cpp
 * @addtogroup myControl
 * @author 徐立丹
 * 
 * 此文件包含 mode_point_stabilization 函数的定义
 * 便于单独修改和维护推力/力矩控制逻辑
 */

#include "offboard_control_boat.hpp"

// 发布推力和力矩设定值，仅在offboard模式下
void OffboardControlBoat::mode_point_stabilization()
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
	Eigen::Vector2f R1 = R.col(0);
	
	Eigen::Vector2f position_xy(position_[0], position_[1]);
	float distance_to_target = (position_xy - position_xy_des_).norm();
	
	// 计算指向目标的期望旋转矩阵
	Eigen::Matrix2f R_des;
	Eigen::Vector2f R1_des = (position_xy_des_ - position_xy).normalized();
	double psi_des = std::atan2(R1_des.y(), R1_des.x());
	Eigen::Matrix2f R_90;
	R_90 << 0.0f, -1.0f,
	        1.0f,  0.0f;
	Eigen::Vector2f R2_des = R_90 * R1_des;
	R_des.col(0) = R1_des;
	R_des.col(1) = R2_des;
	
	Eigen::Matrix2f error_R;
	float r_ref; // 期望偏航角速度
	float r = angular_velocity_[2]; // 当前偏航角速度
	float angle;
	error_R = R.transpose() * R_des - R_des.transpose() * R;
	angle = abs(acos(R1_des.dot(R1)));
	r_ref = k3_attitude_gain_ * std::copysign(1.0f, error_R(1,0)) * angle; // 期望偏航角速度
	if (r_ref > yaw_rate_limit_) r_ref = yaw_rate_limit_;
	if (r_ref < -yaw_rate_limit_) r_ref = -yaw_rate_limit_;
	float Tauz;
	Tauz = inertia_z_ * (-k4_rate_gain_ * (r - r_ref));
	
	// === 推力控制 ===
	float T = 0.0f;
	
	// 只有当航向误差在±5°以内才施加推力
	if (angle < heading_tolerance_rad_) {
		// 计算期望速度（沿目标方向）
		Eigen::Vector2f error_xy = position_xy_des_ - position_xy;
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
		T = force_desired.dot(R1);
		
		// 限制推力为正值
		T = std::max(0.0f, T);
		
		// 限制最大推力
		float max_thrust = max_thrust_;
		T = std::min(T, max_thrust);
	}

	// 应用死区
	if (T > 0.001f) {
		Tx_sp = std::max(T, 0.12f);
	} else if (T < -0.001f)
	{
		Tx_sp = std::min(T, -0.12f);
	} else {
		Tx_sp = 0.0f;
	}
	
	// 应用死区
	if (Tauz > 0.001f) {
		Tauz_sp = std::max(Tauz, 0.16f);
	} else if (Tauz < -0.001f)
	{
		Tauz_sp = std::min(Tauz, -0.16f);
	} else {
		Tauz_sp = 0.0f;
	}
}

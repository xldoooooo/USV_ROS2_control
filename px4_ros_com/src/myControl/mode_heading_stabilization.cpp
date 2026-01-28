/**
 * 此文件包含 mode_heading_stabilization 函数的定义
 * 便于单独修改和维护推力/力矩控制逻辑
 */

#include "offboard_control_boat.hpp"

// 发布推力和力矩设定值，仅在offboard模式下
void OffboardControlBoat::mode_heading_stabilization()
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
	Eigen::Vector2f position_xy_(position_[0], position_[1]);
	
	// 计算指向目标的期望旋转矩阵
	Eigen::Matrix2f R_des;
	Eigen::Vector2f R1_des = (position_xy_des_ - position_xy_).normalized();
	Eigen::Matrix2f R_90;
	R_90 << 0.0f, -1.0f,
	        1.0f,  0.0f;
	
	// 如果姿态误差小于3度，则不进行转向控制
	Eigen::Vector2f R1 = R.col(0);
	Eigen::Matrix2f R_p;
	R_p << std::cos(heading_tolerance_rad_), -std::sin(heading_tolerance_rad_),
	        std::sin(heading_tolerance_rad_),  std::cos(heading_tolerance_rad_);
	Eigen::Matrix2f R_n;
	R_n << std::cos(-heading_tolerance_rad_), -std::sin(-heading_tolerance_rad_),
	        std::sin(-heading_tolerance_rad_),  std::cos(-heading_tolerance_rad_);
	Eigen::Vector2f R1_des_r = R_p * R1_des;
	Eigen::Vector2f R2_des_r = R_90 * R1_des_r;
	Eigen::Vector2f R1_des_l = R_n * R1_des;
	Eigen::Vector2f R2_des_l = R_90 * R1_des_l;
	Eigen::Matrix2f error_R;
	float r_ref;
	float r = angular_velocity_[2]; // 当前偏航角速度
	float cos_angle = R1.dot(R1_des);
	float angle;
	if (cos_angle > std::cos(heading_tolerance_rad_)) {
		Tauz_sp = 0.0f;
	}else if (R1.dot(R1_des_r) > R1.dot(R1_des_l)) {
		// 右转更近
		R_des.col(0) = R1_des_r;
		R_des.col(1) = R2_des_r;
		error_R = R.transpose() * R_des - R_des.transpose() * R;
		angle = abs(acos(R1_des_r.dot(R1)));
		r_ref = k3_attitude_gain_ * std::copysign(1.0f, error_R(1,0)) * angle; // 期望偏航角速度
		if (r_ref > yaw_rate_limit_) r_ref = yaw_rate_limit_;
		if (r_ref < -yaw_rate_limit_) r_ref = -yaw_rate_limit_;
		Tauz_sp = inertia_z_ * (-k4_rate_gain_ * (r - r_ref));
	} else {
		// 左转更近
		R_des.col(0) = R1_des_l;
		R_des.col(1) = R2_des_l;
		error_R = R.transpose() * R_des - R_des.transpose() * R;
		angle = abs(acos(R1_des_l.dot(R1)));
		r_ref = k3_attitude_gain_ * std::copysign(1.0f, error_R(1,0)) * angle; // 期望偏航角速度
		if (r_ref > yaw_rate_limit_) r_ref = yaw_rate_limit_;
		if (r_ref < -yaw_rate_limit_) r_ref = -yaw_rate_limit_;
		Tauz_sp = inertia_z_ * (-k4_rate_gain_ * (r - r_ref));
	}
	
	// === 推力控制 ===
	Tx_sp = 0.0f;
}

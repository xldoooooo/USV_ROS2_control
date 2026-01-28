/**
 * @brief 离线控制船只 - 推力/力矩设定值发布
 * @file offboard_control_boat_thrust_torque.cpp
 * @addtogroup myControl
 * @author 徐立丹
 * 
 * 此文件包含 mode_idle 函数的定义
 * 便于单独修改和维护推力/力矩控制逻辑
 */

#include "offboard_control_boat.hpp"

// 发布零推力和零力矩，仅在offboard模式下
void OffboardControlBoat::mode_idle()
{
	Tx_sp = 0.0f;
	Tauz_sp = 0.0f;
}
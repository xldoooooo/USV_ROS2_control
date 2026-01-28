/**
 * @brief 离线控制船只 - 键盘输入线程
 * @file keyboard_input.cpp
 * @addtogroup myControl
 * @author 徐立丹
 * 
 * 此文件包含独立的键盘输入线程
 * 监听键盘空格键，不需要按enter即可触发
 * 参考 keyboard_vel_controller.cpp 中的 sh_getch() 实现
 */

#include "offboard_control_boat.hpp"
#include <poll.h>
#include <errno.h>
#include <cstring>
#include <unistd.h>

/**
 * @brief 键盘输入线程函数
 * 
 * 独立运行在单独的线程中，不会阻塞主程序
 * 键盘交互流程：
 * 1. 用户按下空格键（无需enter）
 * 2. 系统立即提示输入目标位置 (x y)
 * 3. 用户输入 "x y" 后按enter确认
 * 4. 系统更新目标位置并自动切换到POINT_STABILIZATION模式
 */
void OffboardControlBoat::keyboard_input_thread()
{
	// 保存原始终端设置
	struct termios old_tio, new_tio;
	tcgetattr(STDIN_FILENO, &old_tio);
	
	// 设置为 raw 模式（立即响应按键，无需回车）
	new_tio = old_tio;
	new_tio.c_lflag &= ~(ICANON | ECHO);  // 禁用 canonical 模式和回显
	tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

	std::cout << "\n========================================" << std::endl;
	std::cout << "键盘输入提示:" << std::endl;
	std::cout << "  【重要】先按空格键，再输入坐标！" << std::endl;
	std::cout << "  操作步骤：" << std::endl;
	std::cout << "    1. 按下空格键（只按空格，不要按其他键）" << std::endl;
	std::cout << "    2. 看到提示后，输入坐标 x y（如: 1 3）" << std::endl;
	std::cout << "    3. 按回车确认" << std::endl;
	std::cout << "========================================\n" << std::endl;

	RCLCPP_INFO(this->get_logger(), "键盘输入线程正在等待输入（raw模式已启用）...");

	while (!shutdown_keyboard_ && rclcpp::ok()) {
		char ch;
		// 使用非阻塞 read，每次读取1个字符
		fd_set readfds;
		FD_ZERO(&readfds);
		FD_SET(STDIN_FILENO, &readfds);
		
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;  // 100ms超时
		
		int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
		
		if (shutdown_keyboard_) break;
		
		if (ret > 0) {
			// 有输入可用
			if (read(STDIN_FILENO, &ch, 1) == 1) {
				// 调试：显示收到的字符
				if (ch >= 32 && ch <= 126) {
					RCLCPP_INFO(this->get_logger(), "键盘输入: '%c' (ASCII=%d), 当前nav_state=%d", ch, (int)ch, current_nav_state_);
				} else {
					RCLCPP_INFO(this->get_logger(), "键盘输入: 控制字符 (ASCII=%d), 当前nav_state=%d", (int)ch, current_nav_state_);
				}

				if (ch == ' ' && current_nav_state_ == 14) {
					// 检测到空格键，需要临时恢复 canonical 模式以读取完整行
					tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
					
					waiting_for_input_ = true;
					std::cout << "\n>>> 请输入目标位置 (x y): ";
					std::cout.flush();
					
					RCLCPP_INFO(this->get_logger(), "空格键已触发，等待坐标输入...");
					
					// 读取完整的一行输入
					std::string line;
					if (std::getline(std::cin, line)) {
						RCLCPP_INFO(this->get_logger(), "读取到输入: '%s'", line.c_str());
						if (!line.empty()) {
							float x, y;
							std::istringstream iss(line);
							if (iss >> x >> y) {
								std::lock_guard<std::mutex> lock(position_des_mutex_);
								position_xy_des_ = Eigen::Vector2f(x, y);
								waiting_for_input_ = false;
								
								// 自动切换到POINT_STABILIZATION模式进行跟踪
								std::lock_guard<std::mutex> mode_lock(mode_mutex_);
								current_mode_ = BoatControlMode::POINT_STABILIZATION;
								RCLCPP_INFO(this->get_logger(), 
									"目标位置已更新: x=%.2f, y=%.2f，已切换到POINT_STABILIZATION模式", x, y);
								std::cout << "\n按空格键输入新的目标位置: \n";
								std::cout.flush();
							} else {
								RCLCPP_WARN(this->get_logger(), "坐标格式无效！请输入 'x y' 格式 (例如: 5.0 3.0)");
								waiting_for_input_ = false;
								std::cout << "\n按空格键输入新的目标位置: \n";
								std::cout.flush();
							}
						} else {
							RCLCPP_WARN(this->get_logger(), "输入为空");
							waiting_for_input_ = false;
						}
					}
					
					// 恢复 raw 模式
					tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
				} else if (ch == ' ' && current_nav_state_ != 14) {
					std::cout << "\n";  // 换行，因为没有回显
					RCLCPP_WARN(this->get_logger(), "请先解锁并切换到Offboard模式 (当前状态: %d, 需要: 14)", current_nav_state_);
				}
			}
		}
		// ret == 0 表示超时，继续循环
		// ret < 0 表示出错
	}

	// 恢复原始终端设置
	tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
	RCLCPP_INFO(this->get_logger(), "键盘输入线程已停止，终端已恢复");
}

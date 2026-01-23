#include "ros/ros.h"
#include "wheel_leg/wl_control.h"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>
#include <csignal>
#include <algorithm>  // 用于std::clamp

// 全局变量存储终端设置
struct termios oldt, newt;
bool terminal_configured = false;

// 限幅参数
const double MAX_LINEAR_VEL = 2.0;   // 最大线速度 m/s
const double MAX_ANGULAR_VEL = 3.14; // 最大角速度 rad/s (约180度/秒)
const double MAX_LEG_POS = 0.2;     // 最大腿位置 rad (约90度)
const double MIN_LEG_POS = -0.3;    // 最小腿位置 rad (约-90度)

// 控制增量
double linear_step = 0.1;    // 线速度增量
double angular_step = 0.1;   // 角速度增量  
double leg_step = 0.005;       // 腿位置增量



// 信号处理函数
void signalHandler(int sig) {
    std::cout << "\n接收到中断信号,退出程序..." << std::endl;
    if (terminal_configured) {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }
    exit(0);
}

// 设置非阻塞键盘输入
bool setNonBlockingInput(bool nonBlocking) {
    if (nonBlocking && !terminal_configured) {
        // 保存当前终端设置
        if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
            return false;
        }
        newt = oldt;
        
        // 禁用规范模式和回显
        newt.c_lflag &= ~(ICANON | ECHO);
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) != 0) {
            return false;
        }
        
        // 设置非阻塞
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) != 0) {
            return false;
        }
        
        terminal_configured = true;
        return true;
    } else if (!nonBlocking && terminal_configured) {
        // 恢复原始终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
        terminal_configured = false;
    }
    return true;
}

// 检查是否有键盘输入
int kbhit() {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

// 读取一个字符（非阻塞）
int getch() {
    int r;
    unsigned char c;
    if ((r = read(STDIN_FILENO, &c, sizeof(c))) < 0) {
        return -1;
    } else {
        return c;
    }
}

// 限幅函数
double clamp(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 应用限幅到控制消息
void applyLimits(wheel_leg::wl_control& msg) {
    msg.linear_vel = clamp(msg.linear_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    msg.angular_vel = clamp(msg.angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    msg.leg_pos = clamp(msg.leg_pos, MIN_LEG_POS, MAX_LEG_POS);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "key_control_node");
    ros::NodeHandle nh;

    // 注册信号处理
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 设置非阻塞输入
    if (!setNonBlockingInput(true)) {
        std::cerr << "错误：无法设置非阻塞输入" << std::endl;
        return -1;
    }

    // 创建发布者，发布到 wl_control 话题
    ros::Publisher control_pub = nh.advertise<wheel_leg::wl_control>("wl_control", 10);
    
    // 初始化控制消息
    wheel_leg::wl_control control_msg;
    control_msg.linear_vel = 0.0;
    control_msg.angular_vel = 0.0;
    control_msg.leg_pos = 0.0;


    std::cout << "=== 轮腿机器人键盘控制节点 ===" << std::endl;
    std::cout << "控制说明:" << std::endl;
    std::cout << "  运动控制:" << std::endl;
    std::cout << "    w/s: 增加/减少线速度 (范围: ±" << MAX_LINEAR_VEL << " m/s)" << std::endl;
    std::cout << "    a/d: 增加/减少角速度 (范围: ±" << MAX_ANGULAR_VEL << " rad/s)" << std::endl;
    std::cout << "  腿部位置控制:" << std::endl;
    std::cout << "    q/e: 微调增加/减少腿位置 (范围: [" << MIN_LEG_POS << ", " << MAX_LEG_POS << "] rad)" << std::endl;
    std::cout << "    c/C: 快速设置腿位置到最小值 (" << MIN_LEG_POS << " rad)" << std::endl;
    std::cout << "    z/Z: 快速设置腿位置到最大值 (" << MAX_LEG_POS << " rad)" << std::endl;
    std::cout << "    f/F: 快速设置腿位置到零位 (0 rad)" << std::endl;
    std::cout << "  系统控制:" << std::endl;
    std::cout << "    r/R: 重置线速度和角速度为0 (腿部位置保持不变)" << std::endl;
    std::cout << "    x/X: 退出程序" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "提示:" << std::endl;
    std::cout << "  - 所有控制量都自动限幅，确保在安全范围内" << std::endl;
    std::cout << "  - 控制消息以100Hz频率持续发布" << std::endl;
    std::cout << "  - 屏幕底部实时显示当前控制状态" << std::endl;
    std::cout << "=============================" << std::endl;

    ros::Rate rate(100);  

    while (ros::ok())
    {
        bool control_updated = false;
        
        // 检查是否有键盘输入（非阻塞）
        if (kbhit()) {
            int key = getch();
            
            // 忽略换行符和回车符
            if (key == '\n' || key == '\r') {
                continue;
            }
            
            switch (key)
            {
            // 线速度控制
            case 'w': case 'W':
                control_msg.linear_vel += linear_step;
                control_updated = true;
                std::cout << "\n设置线速度为: " << control_msg.linear_vel << std::endl;
                break;
            case 's': case 'S':
                control_msg.linear_vel -= linear_step;
                control_updated = true;
                std::cout << "\n设置线速度为: " << control_msg.linear_vel << std::endl;
                break;
                
            // 角速度控制
            case 'a': case 'A':
                control_msg.angular_vel += angular_step;
                control_updated = true;
                std::cout << "\n设置角速度为: " << control_msg.angular_vel << std::endl;
                break;
            case 'd': case 'D':
                control_msg.angular_vel -= angular_step;
                control_updated = true;
                std::cout << "\n设置角速度为: " << control_msg.angular_vel << std::endl;
                break;
                
            // 腿部位置微调控制
            case 'q': case 'Q':
                control_msg.leg_pos += leg_step;
                control_updated = true;
                std::cout << "\n设置腿位置为: " << control_msg.leg_pos << std::endl;
                break;
            case 'e': case 'E':
                control_msg.leg_pos -= leg_step;
                control_updated = true;
                std::cout << "\n设置腿位置为: " << control_msg.leg_pos << std::endl;
                break;
                
            // 腿部位置快速设置
            case 'c': case 'C':
                control_msg.leg_pos = MIN_LEG_POS;
                control_updated = true;
                std::cout << "\n快速设置腿位置到最小值: " << control_msg.leg_pos << std::endl;
                break;
            case 'z': case 'Z':
                control_msg.leg_pos = MAX_LEG_POS;
                control_updated = true;
                std::cout << "\n快速设置腿位置到最大值: " << control_msg.leg_pos << std::endl;
                break;
            case 'f': case 'F':
                control_msg.leg_pos = 0;
                control_updated = true;
                std::cout << "\n快速设置腿位置到零位: " << control_msg.leg_pos << std::endl;
                break;
                
            // 系统控制
            case 'r': case 'R':
                control_msg.linear_vel = 0.0;
                control_msg.angular_vel = 0.0;
                control_updated = true;
                std::cout << "\n重置线速度和角速度为0 (腿部位置保持不变)" << std::endl;
                break;
            case 'x': case 'X':
                std::cout << "\n退出程序" << std::endl;
                setNonBlockingInput(false);
                return 0;
            default:
                // 忽略其他按键
                break;
            }
        }

        // 应用限幅
        applyLimits(control_msg);

        // 持续发布控制消息
        control_pub.publish(control_msg);
        
        // 显示当前状态（使用回车符覆盖上一行）
        std::cout << "\r当前状态 - 线速度: " << control_msg.linear_vel 
                  << ", 角速度: " << control_msg.angular_vel
                  << ", 腿位置: " << control_msg.leg_pos 
                  << "     " << std::flush;

        ros::spinOnce();
        rate.sleep();
    }

    // 恢复终端设置
    setNonBlockingInput(false);

    return 0;
}
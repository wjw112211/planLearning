// 键盘遥控节点（grid_map_manual_mapping）。
//
// 用 WASD 控制机器人在 Gazebo 里走：
//   w 前进   s 后退   a 左转   d 右转   空格 / x 立即停车
// 速度、发布频率、"松开键多久后自动停"都能用参数配（见 config/manual_mapping.yaml）。
//
// 实现要点：
//   * 终端切成 raw + 非阻塞模式（关掉行缓冲和回显），所以不用回车、按一下就有反应；
//   * 一直在终端里回显当前速度，按了键马上能看到，方便确认"按键到底有没有被收到"；
//   * 退出（Ctrl+C）时把终端设置恢复原样；
//   * 不依赖 keyboard 的自动重复：松开键后 key_timeout 秒没有新按键就自动停车。
//     把 key_timeout 设成 <= 0 就变成"一直保持上次速度"；
//   * 只有"在动"和"刚停下"这两段时间才发 /cmd_vel，空闲时一句话都不发 ——
//     否则 20 Hz 的全 0 指令会把别人的指令（rostopic pub、规划器、看门狗）冲掉，
//     表现出来就是"机器人动不了"。
#include <cctype>
#include <cstdio>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "logManager/logManager.h"

class KeyboardTeleop
{
public:
  KeyboardTeleop()
      : nodeHandle_(),
        linearSpeed_(0.15),
        angularSpeed_(0.6),
        publishRate_(20.0),
        keyTimeout_(0.3),
        stopBurst_(0.5),
        hasTerminal_(false),
        moving_(false),
        stopping_(false),
        linear_(0.0),
        angular_(0.0)
  {
    ros::NodeHandle privateNodeHandle("~");
    std::string cmdVelTopic;
    privateNodeHandle.param<std::string>("cmd_vel_topic", cmdVelTopic, "/cmd_vel");
    privateNodeHandle.param("linear_speed", linearSpeed_, linearSpeed_);
    privateNodeHandle.param("angular_speed", angularSpeed_, angularSpeed_);
    privateNodeHandle.param("publish_rate", publishRate_, publishRate_);
    privateNodeHandle.param("key_timeout", keyTimeout_, keyTimeout_);
    privateNodeHandle.param("stop_burst", stopBurst_, stopBurst_);

    publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmdVelTopic, 1);
    lastKeyTime_ = ros::Time::now();

    hasTerminal_ = enableRawMode();
    std::printf("\n");
    std::printf("=========================================================\n");
    std::printf(" 键盘遥控（WASD），发布到 %s\n", cmdVelTopic.c_str());
    std::printf("   w : 前进        s : 后退\n");
    std::printf("   a : 左转        d : 右转\n");
    std::printf("   空格 / x : 立即停车\n");
    std::printf(" Ctrl+C : 退出（退出时自动恢复终端设置）\n");
    std::printf(" 速度：前后 %.3f m/s，转向 %.3f rad/s；松开 %.2f s 后自动停\n", linearSpeed_,
                angularSpeed_, keyTimeout_);
    std::printf(" 下面这行会实时显示当前速度，按了键没变化说明键盘焦点不在这\n");
    std::printf("=========================================================\n");
    printState();

    if (hasTerminal_)
    {
      LOG(INFO) << "keyboard teleop ready: wasd -> " << cmdVelTopic << ", linear " << linearSpeed_
                << " m/s, angular " << angularSpeed_ << " rad/s, key_timeout " << keyTimeout_ << " s";
    }
    else
    {
      LOG(WARNING) << "标准输入不是终端（roslaunch 没开 output=\"screen\"？），"
                      "键盘遥控不可用，只会一直发 0 速度。可以用 rostopic pub /cmd_vel ... 代替。";
    }
  }

  ~KeyboardTeleop()
  {
    stop();
    publishVelocity();
    restoreTerminal();
    std::printf("\nkeyboard_teleop 退出。\n");
  }

  void spin()
  {
    const double rate = std::max(1.0, publishRate_);
    ros::Rate loopRate(rate);
    while (ros::ok())
    {
      readKeys();
      applyTimeout();
      publishIfNeeded();
      ros::spinOnce();
      loopRate.sleep();
    }
  }

private:
  bool enableRawMode()
  {
    if (::isatty(STDIN_FILENO) == 0)
    {
      return false;
    }
    if (::tcgetattr(STDIN_FILENO, &originalSettings_) != 0)
    {
      return false;
    }
    struct termios raw = originalSettings_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));  // 关行缓冲、关回显
    raw.c_cc[VMIN] = 0;                                     // 读不到就返回，不阻塞
    raw.c_cc[VTIME] = 0;
    return ::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }

  void restoreTerminal()
  {
    if (hasTerminal_)
    {
      ::tcsetattr(STDIN_FILENO, TCSANOW, &originalSettings_);
    }
  }

  void readKeys()
  {
    if (!hasTerminal_)
    {
      LOG_EVERY_T(WARNING, 10.0) << "读不到按键：stdin 不是终端。"
                                    "用 keyboard_launch_prefix:=\"gnome-terminal --\" 启动，"
                                    "或在终端里 rosrun grid_map_manual_mapping keyboard_teleop";
      return;
    }
    char buffer[16];
    const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count <= 0)
    {
      return;
    }
    for (ssize_t i = 0; i < count; ++i)
    {
      handleKey(buffer[i]);
    }
  }

  void handleKey(char key)
  {
    switch (key)
    {
      case 'w':
      case 'W':
        linear_ = linearSpeed_;
        angular_ = 0.0;
        break;
      case 's':
      case 'S':
        linear_ = -linearSpeed_;
        angular_ = 0.0;
        break;
      case 'a':
      case 'A':
        linear_ = 0.0;
        angular_ = angularSpeed_;
        break;
      case 'd':
      case 'D':
        linear_ = 0.0;
        angular_ = -angularSpeed_;
        break;
      case ' ':
      case 'x':
      case 'X':
        linear_ = 0.0;
        angular_ = 0.0;
        break;
      default:
        return;  // 其它键（含 Ctrl+C 产生的 SIGINT 之外的杂字符）忽略
    }
    lastKeyTime_ = ros::Time::now();
    moving_ = (linear_ != 0.0 || angular_ != 0.0);
    stopping_ = !moving_;
    stopStart_ = ros::Time::now();
    if (moving_)
    {
      publishVelocity();  // 按下就往出打，立刻有反应
    }
    printState();
  }

  void applyTimeout()
  {
    if (keyTimeout_ <= 0.0 || !moving_)
    {
      return;
    }
    if ((ros::Time::now() - lastKeyTime_).toSec() > keyTimeout_)
    {
      stop();
    }
  }

  void stop()
  {
    if (linear_ != 0.0 || angular_ != 0.0)
    {
      printState();
    }
    linear_ = 0.0;
    angular_ = 0.0;
    moving_ = false;
    stopping_ = true;  // 再补发一小段停车指令，然后彻底安静
    stopStart_ = ros::Time::now();
  }

  // 只在"在动"和"刚停下补发停车指令"这两段时间发消息。
  // 空闲时一句话都不发，免得 20 Hz 的全 0 指令把别人的指令冲掉。
  void publishIfNeeded()
  {
    if (moving_)
    {
      publishVelocity();
      return;
    }
    if (stopping_ && (ros::Time::now() - stopStart_).toSec() < stopBurst_)
    {
      publishVelocity();  // 此时 linear_/angular_ 都是 0
      return;
    }
    stopping_ = false;
  }

  void publishVelocity()
  {
    geometry_msgs::Twist command;
    command.linear.x = linear_;
    command.angular.z = angular_;
    publisher_.publish(command);
  }

  // 在终端里实时回显当前速度（一行，\r 覆盖）
  void printState()
  {
    std::printf("\r linear=%+.3f m/s  angular=%+.3f rad/s   ", linear_, angular_);
    std::fflush(stdout);
  }

  ros::NodeHandle nodeHandle_;
  ros::Publisher publisher_;
  struct termios originalSettings_;
  double linearSpeed_;
  double angularSpeed_;
  double publishRate_;
  double keyTimeout_;
  double stopBurst_;
  bool hasTerminal_;
  bool moving_;
  bool stopping_;
  double linear_;
  double angular_;
  ros::Time lastKeyTime_;
  ros::Time stopStart_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  LOG_INIT("./log", 7, 6 * 3600, "mapping");

  KeyboardTeleop teleop;
  teleop.spin();
  return 0;
}

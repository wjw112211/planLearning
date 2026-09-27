#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "logManager/logManager.h"

/**
 * 速度指令看门狗（watchdog）。
 *
 * Gazebo 的差速驱动插件（libgazebo_ros_diff_drive.so）没有指令超时机制：
 * 只要收到过一次 /cmd_vel，它就会一直用这个速度驱动轮子，直到收到新的指令。
 * 所以用 rostopic pub 发完速度再 Ctrl+C 之后，机器人会一直走下去。
 *
 * 本节点订阅 /cmd_vel：如果在 timeout 秒内没有再收到新的速度指令，
 * 就按 publish_rate 持续发布全 0 的 Twist，把机器人停下来。
 */
class CmdVelWatchdog
{
public:
  CmdVelWatchdog()
      : privateNodeHandle_("~")
  {
    privateNodeHandle_.param<std::string>("cmd_vel_topic", cmdVelTopic_, "/cmd_vel");
    privateNodeHandle_.param("timeout", timeout_, 0.5);
    privateNodeHandle_.param("publish_rate", publishRate_, 20.0);

    publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmdVelTopic_, 1);
    subscriber_ = nodeHandle_.subscribe(cmdVelTopic_, 1,
                                        &CmdVelWatchdog::cmdVelCallback, this);
    timer_ = nodeHandle_.createTimer(ros::Duration(1.0 / publishRate_),
                                     &CmdVelWatchdog::timerCallback, this);

    LOG(INFO) << "cmd_vel watchdog on '" << cmdVelTopic_ << "', stop after "
              << timeout_ << " s without a new command.";
  }

private:
  // 是否为停车指令，即六个分量都为 0。
  static bool isStopCommand(const geometry_msgs::Twist& command)
  {
    return command.linear.x == 0.0 && command.linear.y == 0.0 && command.linear.z == 0.0 &&
           command.angular.x == 0.0 && command.angular.y == 0.0 && command.angular.z == 0.0;
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& message)
  {
    // 超时后本节点会往同一个话题发停车指令。这些指令必须忽略，
    // 否则会被当成“刚收到新指令”，超时判断永远不成立。
    if (stopped_ && isStopCommand(*message)) {
      return;
    }

    lastCommandTime_ = ros::Time::now();
    stopped_ = false;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    // 已经处于停车状态时继续补发停车指令，万一某一条消息丢了也不会再走起来。
    if (stopped_) {
      publishStopCommand();
      return;
    }

    const double elapsed = (ros::Time::now() - lastCommandTime_).toSec();
    if (elapsed > timeout_) {
      LOG(INFO) << "No velocity command for " << elapsed
                << " s, publishing a stop command.";
      publishStopCommand();
    }
  }

  void publishStopCommand()
  {
    publisher_.publish(geometry_msgs::Twist());
    stopped_ = true;
  }

  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  std::string cmdVelTopic_;
  double timeout_;
  double publishRate_;
  // 是否处于“本节点正在发停车指令”的状态。
  bool stopped_ = true;
  // 最近一次收到有效速度指令的时间。
  ros::Time lastCommandTime_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_watchdog");
  // 日志文件名固定为 mapping（log/mapping.INFO、log/mapping.WARNING ...），保留 7 天
  LOG_INIT("./log", 7, 6 * 3600, "mapping");

  CmdVelWatchdog watchdog;
  ros::spin();
  return 0;
}

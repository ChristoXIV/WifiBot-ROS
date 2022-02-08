#ifndef SERVER_WIFIBOT_H
#define SERVER_WIFIBOT_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

typedef struct _sposition
{
  double x;
  double y;
  double th;
} position;

class Wifibot {
public:
  Wifibot();
  ~Wifibot();
  void update();

private:
  void velocityCallback(const geometry_msgs::TwistConstPtr& vel);
  void computeOdometry(double dleft, double dright);
  double getSpeedLinear(double speedLeft, double speedRight);
  double getSpeedAngular(double speedLeft, double speedRight);

  ros::NodeHandle _nh, _nh_private;

  geometry_msgs::TransformStamped _odomTf;
  tf::TransformBroadcaster _odomBroadcast;

  wifibot::Driver *_pDriver;

  ros::Publisher _pubStatus;
  ros::Publisher _pubOdometry;
  ros::Publisher _pubRobotBatteryVoltage, _pubComputerBatteryVoltage;
  ros::Publisher _pubIsCharging;
  ros::Subscriber _subSpeeds;

  ros::Time _timeCurrent;
  ros::Time _timeLast;


  std::string _frameBase;
  std::string _frameOdom;
  position _position;
  double _odometryLeftLast;
  double _odometryRightLast;
  double _entrax;
  bool _updated;
  double _speedLeft;
  double _speedRight;
  double _batteryMinVoltage, _batteryMaxVoltage;
};

#endif

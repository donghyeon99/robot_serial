/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include "ros/init.h"
#include "ros/publisher.h"
#include <cstddef>
#include <serial/serial.h>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>


using namespace std;

class robotSerial{
    public:
  robotSerial() : nh_(""), nh_priv_("~"), spinner_(0), imuTimer_(), timer_(),
  pubRemote_(),pubebimu1_(),pubebimu2_(),pubLaser1_(),pubLaser2_(),pubMindata1_(),pubMindata2_(),pubUWB_(), pubAruco_()
  {
            // Log
    ROS_INFO_STREAM("Start to initialize robofrien serial node.");

    // Spinner
    spinner_.start();

    // Load parameters
    if (!nh_priv_.getParam("robot_port", port_)) {
      port_ = "/dev/ttyUSB0";
      ROS_WARN_STREAM("Parameter missing: port, set default: " << port_);
    }
    if (!nh_priv_.getParam("robot_baud_rate", baud_rate_)) {
      baud_rate_ = 57600;
      ROS_WARN_STREAM(
          "Parameter missing: baud_rate, set default: " << baud_rate_);
    }
    if (!nh_priv_.getParam("timer_rate", timerRate_)) {
      timerRate_ = 2;
      ROS_WARN_STREAM("Parameter missing: timer rate, set default: " << timerRate_);
    }
    

    // pub1_ = nh_.advertise<nav_msgs::Odometry>("imuData_robot1", 1000);
    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    
    pubUWB_     = nh_.advertise<std_msgs::String>("/final/distpozyx",100);
    pubRemote_  = nh_.advertise<geometry_msgs::TwistStamped>("/final/humanoidState" ,100);
    pubebimu1_  = nh_.advertise<nav_msgs::Odometry>("/final/imuData_robot1",100);
    pubebimu2_  = nh_.advertise<nav_msgs::Odometry>("/final/imuData_robot2",100);
    pubLaser1_  = nh_.advertise<sensor_msgs::LaserScan>("/final/hu1/scan"      ,100);
    pubLaser2_  = nh_.advertise<sensor_msgs::LaserScan>("/final/hu2/scan"      ,100);
    pubMindata1_= nh_.advertise<std_msgs::Float64MultiArray>("/final/hu1/minData"   ,100);
    pubMindata2_= nh_.advertise<std_msgs::Float64MultiArray>("/final/hu2/minData"   ,100);
    pubAruco_   = nh_.advertise<geometry_msgs::PoseStamped>("/final/Aruco"   ,100);

    // Subscriber
    sub_UWB_     = nh_.subscribe( "distpozyx" ,100, &robotSerial::pozyx_callback, this);
    sub_remote   = nh_.subscribe( "humanoidState" ,100, &robotSerial::remote_callback, this);
    sub_ebimu1   = nh_.subscribe( "imuData_robot1",100, &robotSerial::ebimu_callback1, this);
    sub_ebimu2   = nh_.subscribe( "imuData_robot2",100, &robotSerial::ebimu_callback2, this);
    sub_laser1   = nh_.subscribe( "hu1/scan"      ,100, &robotSerial::laserCallback1, this);
    sub_laser2   = nh_.subscribe( "hu2/scan"      ,100, &robotSerial::laserCallback2, this);
    sub_minData1 = nh_.subscribe( "hu1/minData"   ,100, &robotSerial::minDataCallback1, this);
    sub_minData2 = nh_.subscribe( "hu2/minData"   ,100, &robotSerial::minDataCallback2, this);
    sub_aruco    = nh_.subscribe( "aruco_single/pose"   ,100, &robotSerial::minArucoCallback, this);
    // Timer
    timer_ = nh_.createTimer(ros::Rate(timerRate_), &robotSerial::Timer_callback, this);
    timer_.stop();
    }

    virtual ~robotSerial(){
        timer_.stop();
        pubUWB_     .shutdown();
        pubRemote_  .shutdown();
        pubebimu1_  .shutdown();
        pubebimu2_  .shutdown();
        pubLaser1_  .shutdown();
        pubLaser2_  .shutdown();
        pubMindata1_.shutdown();
        pubMindata2_.shutdown();
        pubAruco_   .shutdown();
        spinner_.stop();
        nh_priv_.shutdown();
        nh_.shutdown();
    }
    virtual bool init_serial() {
    try {
      ser_.setPort(port_);
      ser_.setBaudrate(baud_rate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser_.setTimeout(to);
      ser_.open();
    } catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Unable to open port ");
      return false;
    }

    if (ser_.isOpen()) {
      ROS_INFO_STREAM("Serial Port initialized. (robot remote)");
      return true;
    } else {
      return false;
    }
  }
    virtual void startTimer() { timer_.start(); }

private:
  virtual void writeSerial()
  {
    int data = 0;
    if     (remoteData.twist.linear.x == 1)  { data = 1; }
    else if(remoteData.twist.linear.x == -1) { data = 2; }
    else if(remoteData.twist.linear.z == -1)  { data = 4; }
    else if(remoteData.twist.linear.z == 1) { data = 8; }
    else {data = 3;}

    if     (remoteData.twist.angular.x == 1)  { data += 1*10;}
    else if(remoteData.twist.angular.x == -1) { data += 2*10;}
    else if(remoteData.twist.angular.z == -1)  { data += 4*10;}
    else if(remoteData.twist.angular.z == 1) { data += 8*10;}
    else {data += 3*10;}

    std::string strData = std::to_string(data);
    cout<<"write data: "<<data<<endl;
    ser_.write(strData);

    if (ser_.available()) {
    std_msgs::String result;
    result.data = ser_.read(ser_.available());
    cout<<"data: "<<result.data<<endl;
    }
    
  }

  virtual void Timer_callback(const ros::TimerEvent& event)
  {
    if(cnt_++%5 == 0)
    {
      writeSerial();
      pubRemote_ .publish(remoteData);
    }
    else  { pubRemote_ .publish(remoteZero); }
    
    pubUWB_     .publish(uwbData  );
    pubebimu1_  .publish(ebimu1   );
    pubebimu2_  .publish(ebimu2   );
    pubLaser1_  .publish(laser1   );
    pubLaser2_  .publish(laser2   );
    pubMindata1_.publish(minArray1);
    pubMindata2_.publish(minArray2);
    pubAruco_   .publish(poseData );
  }

  virtual void pozyx_callback(const std_msgs::String& msg) {uwbData = msg;}
  virtual void remote_callback(const geometry_msgs::TwistStamped& msg){ remoteData = msg; }

  // imu data
  virtual void ebimu_callback1(const nav_msgs::Odometry& msg) { ebimu1 = msg; }
  virtual void ebimu_callback2(const nav_msgs::Odometry& msg) { ebimu2 = msg; }
  // lidar scan data
  virtual void laserCallback1(const sensor_msgs::LaserScan& msg) { laser1 = msg; }
  virtual void laserCallback2(const sensor_msgs::LaserScan& msg) { laser2 = msg; }
  // lidar minimum data
  virtual void minDataCallback1(const std_msgs::Float64MultiArray& msg) { minArray1 = msg; }
  virtual void minDataCallback2(const std_msgs::Float64MultiArray& msg) { minArray2 = msg; }
  virtual void minArucoCallback(const geometry_msgs::PoseStamped& msg) { poseData = msg; }
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::AsyncSpinner spinner_;
  
  ros::Publisher pubUWB_     ; 
  ros::Publisher pubRemote_  ;
  ros::Publisher pubebimu1_  ;
  ros::Publisher pubebimu2_  ;
  ros::Publisher pubLaser1_  ;
  ros::Publisher pubLaser2_  ;
  ros::Publisher pubMindata1_;
  ros::Publisher pubMindata2_;
  ros::Publisher pubAruco_   ; 
  ros::Subscriber sub_UWB_    ; 
  ros::Subscriber sub_remote  ;
  ros::Subscriber sub_ebimu1  ;
  ros::Subscriber sub_ebimu2  ;
  ros::Subscriber sub_laser1  ;
  ros::Subscriber sub_laser2  ;
  ros::Subscriber sub_minData1;
  ros::Subscriber sub_minData2;
  ros::Subscriber sub_aruco   ;
  ros::Timer imuTimer_; 
  ros::Timer timer_;

  serial::Serial ser_;
  string port_;
  int baud_rate_;
  int robot_num_;
  double timerRate_;  // Timer callback rate
  std_msgs::String uwbData;
  geometry_msgs::TwistStamped remoteData;
  nav_msgs::Odometry          ebimu1    ;
  nav_msgs::Odometry          ebimu2    ;
  sensor_msgs::LaserScan      laser1    ;
  sensor_msgs::LaserScan      laser2    ;
  std_msgs::Float64MultiArray minArray1 ;
  std_msgs::Float64MultiArray minArray2 ;
  geometry_msgs::PoseStamped  poseData  ;
  geometry_msgs::TwistStamped remoteZero;

  int cnt_ = 0;

};


void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    // ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "robot_serial_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("serial example node start");
    robotSerial serial;
    bool ret = serial.init_serial();
    if(ret == false) return -1;

    serial.startTimer();
    ros::waitForShutdown();
    
    return 0;

}


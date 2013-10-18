//Description : Differential drive emulator file for vehicle delection and factor settings
//Modifier : Devvrat Arya
//Change Descriptio : Modified wheelSpeedCallback funcion for linear and angular speed calculations
//Modification Date : October 18, 2013
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include <amr_msgs/WheelSpeeds.h>

ros::Subscriber wheel_speed_subscriber;
ros::Publisher velocity_publisher;
double wheel_diameter;
double distance_between_wheels;

void wheelSpeedCallback(const amr_msgs::WheelSpeeds::ConstPtr& msg)
{
  // Check that the message contains exactly two wheel speeds, otherwise it is
  // meaningless for this emulator.
  if (msg->speeds.size() != 2)
  {
    ROS_WARN("Ignoring WheelSpeeds message because it does not contain two wheel speeds.");
    return;
  }

  geometry_msgs::Twist twist;

  //==================== YOUR CODE HERE ====================
  // Instructions: compute linear and angular components and
  //               fill in the twist message.
  //
  //As you have already learned, our target robotic platform is omni-directional and is controlled by
  //sending geometry_msgs/Twist messages to the cmd_vel topic. These messages comprise both lin-
  //ear and angular components of the desired robot’s velocity. In the Braitenberg vehicles, however,
  //differential-drive control is assumed.
  //Therefore your first task is to implement the
  //differential_drive_emulator node. It will listen to the cmd_vel_diff topic, where messages of
  //the type mr_msgs/WheelSpeeds will be published. These messages convey the speeds of two wheels.
  //Upon reception of such a message the node should calculate and publish a velocity command that
  //will make the robot move as a differential- drive platform would have moved if its wheels were
  //spinning with the given speeds.

        twist.linear.x = 0.0;
        twist.angular.x = 0.0;
        float lin_vel;
        float angular_vel;

        //the value we get is angular velocity of the wheels. So we need to multiply it by radius as V = rW
        lin_vel = ((msg->speeds[0] + msg->speeds[1])/2)*(wheel_diameter/2);
        angular_vel = ((msg->speeds[1] - msg->speeds[0])/distance_between_wheels)*(wheel_diameter/2);
        
        twist.linear.x = lin_vel;
        twist.angular.z = angular_vel;
        
        velocity_publisher.publish(twist);

  //========================================================

  velocity_publisher.publish(twist);
  ROS_DEBUG("[%.2f %.2f] --> [%.2f %.2f]", msg->speeds[0], msg->speeds[1], twist.linear.x, twist.angular.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "differential_drive_emulator");
  // Read differential drive parameters from server.
  ros::NodeHandle pn("~");
  pn.param("wheel_diameter", wheel_diameter, 0.15);
  pn.param("distance_between_wheels", distance_between_wheels, 0.5);
  // Create subscriber and publisher.
  ros::NodeHandle nh;
  wheel_speed_subscriber = nh.subscribe("/cmd_vel_diff", 100, wheelSpeedCallback);
  velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  // Start infinite loop.
  ROS_INFO("Started differential drive emulator node.");
  ros::spin();
  return 0;
}


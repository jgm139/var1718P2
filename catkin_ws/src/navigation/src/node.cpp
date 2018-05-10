#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;


int lastChar = 0;
bool robot = false;
class RobotDriver{
  	private:
		//! The node handle we'll be using
		ros::NodeHandle nh_;
		//! We will be publishing to the "/base_controller/command" topic to issue commands
		ros::Publisher cmd_vel_pub_;

  	public:
		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			//set up the publisher for the cmd_vel topic
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
		}

		//! Loop forever while sending drive commands based on keyboard input
		bool driveKeyboard(){
			cout << "Type a command.  "
				"IMPORTANT: Use 'p' to take photos to the other robot, 'a' to turn left, "
				"'d' to turn right, '.' to exit.\n";

			//we will be sending commands of type "twist"
			geometry_msgs::Twist base_cmd;

			char cmd = 0;

			system ("/bin/stty raw");
			while(nh_.ok()){
				cmd = getchar(); 
				if(cmd!='w' && cmd!='a' && cmd!='d' && cmd!='.' && cmd!='r' && cmd!='p' && cmd!='n'){
					cout << "unknown command:" << cmd << "\n";
					continue;
				}

				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
				//move forward
				if(cmd=='w'){
					base_cmd.linear.x += 0.4;
					lastChar = 1;
				} 
				//turn left (yaw) and drive forward at the same time
				else if(cmd=='a'){
					base_cmd.angular.z += 0.3;
					lastChar = 2;
				} 
				//turn right (yaw) and drive forward at the same time
				else if(cmd=='d'){
					base_cmd.angular.z += -0.3;
					lastChar = 3;
				}else if (cmd == 'p'){
					robot = !robot;
				}
				//exit
				else if(cmd=='.'){
					break;
				}
				ros::spinOnce();
				//publish the assembled command
				cmd_vel_pub_.publish(base_cmd);
			}
			system ("/bin/stty cooked");
			return true;
		}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation");

	ros::NodeHandle nh;

	RobotDriver driver(nh);

	driver.driveKeyboard();
	ros::shutdown();
}
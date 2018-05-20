#include <iostream>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>
#include <math.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds;
bool capture = false;

void simpleVis (){
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	while(!viewer.wasStopped()){
		viewer.showCloud (visu_pc);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void saveCloudPoints(){
	ofstream f;
	f.open("listOfPointClouds.txt");
	for(unsigned int i = 0;i<clouds.size();i++){
		std::stringstream ss;
		ss << i;
		f << "pointsCloudsFiles/cloud"+ss.str()+".pcd" << '\n';
		pcl::io::savePCDFileBinary("pointsCloudsFiles/cloud"+ss.str()+".pcd", *clouds[i]);
		std::cerr << "Saved " << (*clouds[i]).size() << " data points to pointsCloudsFiles/cloud"+ss.str()+".pcd" << std::endl;
	}
	f.close();
}

void deleteLastOne(){
	clouds.pop_back();
}

class RobotDriver{
  	private:
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;

  	public:
		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
		}

		void driveKeyboard(){
			cout << "Type a command.  "
				"\n\tw: to go foward. \n\ta: to turn left."
				"\n\td: to turn right. \n\tb: to undo last capture."
				"\n\ts: to save.\n \n\t.: to exit.\n";

			geometry_msgs::Twist base_cmd;

			char cmd = 0;

			system ("/bin/stty raw");
			while(nh_.ok()){
				cmd = getchar();
				if(cmd!='w' && cmd!='a' && cmd!='d' && cmd!='s' && cmd!='b' && cmd!='.'){
					cout << "unknown command:" << cmd << "\n";
					continue;
				}
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
				switch(cmd){
					case 'w':
						base_cmd.linear.x += 0.4;
						capture = true;
						break;
					case 'a':
						base_cmd.angular.z += 0.3;
						capture = true;
						break;
					case 'd':
						base_cmd.angular.z += -0.3;
						capture = true;
						break;
					case 'b':
						deleteLastOne();
						break;
					case 's':
						saveCloudPoints();
						goto end;
					case '.':
						goto end;
				}
				ros::spinOnce();
				cmd_vel_pub_.publish(base_cmd);
			}
			end:;
			system ("/bin/stty cooked");
		}

};

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	if(capture){
		capture = false;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
		clouds.push_back(cloud);
		visu_pc = cloud;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "get_pointclouds");
	ros::NodeHandle nh;
	ros::NodeHandle imageNode;
	RobotDriver driver(nh);
	
	ros::Subscriber imageSub = imageNode.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);
	boost::thread t(simpleVis);
	driver.driveKeyboard();

	ros::shutdown();
}
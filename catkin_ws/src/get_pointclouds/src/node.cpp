#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <sstream>
#include <fstream>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds;
bool capture = false;
const char* menu = "Pulsa:\n\ts: Para guardar todo el conjunto de nubes.\n\tb: Para borrar la última nube capturada.\n\tCualquier otra tecla (menos enter): Para capturar una nube de puntos.\n"; 

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

void captureImage(){
	while(true){
		switch(getchar()){
			case 's':
				saveCloudPoints();
				break;
			case 'b':
				deleteLastOne();
				break;
			case 10: //Enter, no hacer nada
				std::cout << menu;
				break;
			default:
				capture = true;
				break;
		}
	}
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	if(capture){
		capture = false;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
		clouds.push_back(cloud);
		visu_pc = cloud;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

	boost::thread t(simpleVis);
	std::cout << menu;
	boost::thread t2(captureImage);

	while(ros::ok()){
		ros::spinOnce();
	}

}

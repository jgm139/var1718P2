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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>
#include <math.h>

using namespace std;

#define PRINT_INFO	cout << "\r" << std::flush; \
			  		cout << "\n------------------------------------------------" << endl; \
			  		cout << "\033[1;32mCorrespondences found: " << correspondences.size() << endl; \
			  		cout << "\033[1;31mCorrespondences rejected: " << correspondencesRejected.size() << "\033[0m" << endl; \
			  		cout << "------------------------------------------------" << endl << endl;

bool capture = false;
bool first = true;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc_save (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr last (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_last_save (new pcl::PointCloud<pcl::PointWithScale> ());
pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_last (new pcl::PointCloud<pcl::PointWithScale> ());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_left_save (new pcl::PointCloud<pcl::FPFHSignature33> ());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_left (new pcl::PointCloud<pcl::FPFHSignature33> ());




void estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals){
  	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
  	
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZRGB>());

 	ne.setInputCloud(cloud);
  	ne.setSearchMethod(tree_n);
  	ne.setRadiusSearch(0.05);
  	ne.compute(*cloud_normals);
}

void calculateKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
	pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals, pcl::PointCloud<pcl::PointWithScale>::Ptr& result){
  	
  	for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  	{
    	cloud_normals->points[i].x = cloud->points[i].x;
    	cloud_normals->points[i].y = cloud->points[i].y;
    	cloud_normals->points[i].z = cloud->points[i].z;
  	}
  	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  	
 	const static float min_scale = 0.01f; //0.01f
  	const static int n_octaves = 3; //3
  	const static int n_scales_per_octave = 4; //4
  	const static float min_contrast = 0.001f; //.001f

  	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  	sift.setSearchMethod(tree);
  	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  	sift.setMinimumContrast(min_contrast);
  	sift.setInputCloud(cloud_normals);
	sift.compute(*result);
}

void calculateFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints, 
	pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs){
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);
  	
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  	fpfh.setInputCloud (keypoints_xyzrgb);
  	fpfh.setInputNormals (cloud_normals);
	fpfh.setSearchSurface (cloud);
	fpfh.setSearchMethod(tree);
  	fpfh.setRadiusSearch (0.07);
  	fpfh.compute (*fpfhs);
}

void matchFeatures(pcl::PointCloud<pcl::FPFHSignature33>::Ptr& source_features, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& target_features,
	pcl::Correspondences& correspondences){

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
	est.setInputSource(source_features);
	est.setInputTarget(target_features);
	est.determineReciprocalCorrespondences (correspondences); // Probar distancias como segundo parámetro 0.07 
}

Eigen::Matrix4f correspondencesRejection(pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints_1, pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints_2,
	pcl::Correspondences& correspondences, pcl::Correspondences& correspondences_out){ 
	
	pcl::CorrespondencesConstPtr correspondences_p(new pcl::Correspondences(correspondences));

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> sac;
    sac.setInputSource(keypoints_1);
    sac.setInputTarget(keypoints_2);
    sac.setInlierThreshold(0.025);
    sac.setMaximumIterations(10000);
    sac.setRefineModel(true);
    sac.setInputCorrespondences(correspondences_p); 
    sac.getCorrespondences(correspondences_out);
    return sac.getBestTransformation();

}

void filterCloudPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered){
	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.025f, 0.025f, 0.025f);
	vGrid.filter (*filtered);
}

void filterSor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered){

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (cloud);
  	sor.setMeanK (100); // Probar valores
  	sor.setStddevMulThresh (0.25); //Probar valores
  	sor.filter (*cloud_filtered);
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
				"\n\td: to turn right."
				"\n\t.: to exit.\n";

			geometry_msgs::Twist base_cmd;

			char cmd = 0;

			system ("/bin/stty raw");
			while(nh_.ok()){
				cmd = getchar();
				if(cmd!='w' && cmd!='a' && cmd!='d' && cmd!='.'){
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
					case '.':
						goto end;
						break;
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
		if(!first){
			*visu_pc_save = *visu_pc;
			*keypoints_last_save = *keypoints_last;
			*fpfhs_left_save = *fpfhs_left;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr msgP(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_right (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sor_right (new pcl::PointCloud<pcl::PointXYZRGB>);
	  		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_right (new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale> ());
	  		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_right (new pcl::PointCloud<pcl::FPFHSignature33> ());
			pcl::Correspondences correspondences;
			pcl::Correspondences correspondencesRejected;
			
			// Filtramos los puntos
			filterCloudPoint(msgP, cloud_filtered_right);
			
			// Aplicamos Sor
			filterSor(cloud_filtered_right, cloud_filtered_sor_right);

			// Conseguimos las normales
	  		estimateNormals(cloud_filtered_sor_right, cloud_normals_right);

			// Sacamos puntos característicos
			calculateKeyPoints(cloud_filtered_sor_right, cloud_normals_right, keypoints_right);

			// Calculamos features de los keypoints		
	  		calculateFeatures(cloud_filtered_sor_right, keypoints_right, cloud_normals_right, fpfhs_right);

	  		// Buscamos las correspondencias
	  		matchFeatures(fpfhs_left, fpfhs_right, correspondences);

	  		// Filtramos las correspondencias
	  		// Obtenemos la matriz de transformación
  			Eigen::Matrix4f transformation = correspondencesRejection(keypoints_last, keypoints_right, correspondences, correspondencesRejected);

  			PRINT_INFO;

	  		if(correspondencesRejected.size() > correspondences.size()/1.25){
	  			//search globaly

		  		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_global (new pcl::PointCloud<pcl::PointNormal>);
				pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_global (new pcl::PointCloud<pcl::PointWithScale> ());
		  		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_global (new pcl::PointCloud<pcl::FPFHSignature33> ());
				pcl::Correspondences correspondences_global;
				pcl::Correspondences correspondencesRejected_global;
				
				// Conseguimos las normales
		  		estimateNormals(visu_pc, cloud_normals_global);

				// Sacamos puntos característicos
				calculateKeyPoints(visu_pc, cloud_normals_global, keypoints_global);

				// Calculamos features de los keypoints		
		  		calculateFeatures(visu_pc, keypoints_global, cloud_normals_global, fpfhs_global);

		  		// Buscamos las correspondencias
		  		matchFeatures(fpfhs_global, fpfhs_right, correspondences_global);

		  		// Filtramos las correspondencias
		  		// Obtenemos la matriz de transformación
	  			Eigen::Matrix4f transformation = correspondencesRejection(keypoints_global, keypoints_right, correspondences_global, correspondencesRejected_global);


	  			cout << "Checking Global Correspondences" << endl;
	  			cout << "\n------------------------------------------------" << endl; \
		  		cout << "\033[1;32mCorrespondences found: " << correspondences_global.size() << endl; \
	  			cout << "\033[1;31mCorrespondences rejected: " << correspondencesRejected_global.size() << "\033[0m" << endl; \
		  		cout << "------------------------------------------------" << endl << endl;			
	  			if(correspondencesRejected_global.size() > correspondences_global.size()/1.25){
	  				visu_pc = visu_pc_save;
					keypoints_last = keypoints_last_save;
					fpfhs_left = fpfhs_left_save;
					return;
	  			}	
			
	  			*keypoints_last = *keypoints_right;
		  		*fpfhs_left = *fpfhs_right;

		  		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_global (new pcl::PointCloud<pcl::PointXYZRGB>);
	    		pcl::transformPointCloud(*visu_pc, *transformed_cloud_global, transformation);

	  			*visu_pc = *transformed_cloud_global + *cloud_filtered_right;
				
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtrado_global (new pcl::PointCloud<pcl::PointXYZRGB>);
	  			filterCloudPoint(visu_pc, filtrado_global);
	  			cout << "Done Global" << endl;
	  			visu_pc = filtrado_global;
	  			return;
	  		}
	  		
	  		*keypoints_last = *keypoints_right;
	  		*fpfhs_left = *fpfhs_right;


			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    		pcl::transformPointCloud(*visu_pc, *transformed_cloud, transformation);

  			*visu_pc = *transformed_cloud + *cloud_filtered_right;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtrado (new pcl::PointCloud<pcl::PointXYZRGB>);
  			filterCloudPoint(visu_pc, filtrado);

  			visu_pc = filtrado;

		}else{
			*visu_pc = *msg;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_last (new pcl::PointCloud<pcl::PointXYZRGB>);
			filterCloudPoint(visu_pc, cloud_filtered_last);
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sor_last (new pcl::PointCloud<pcl::PointXYZRGB>);
			filterSor(cloud_filtered_last, cloud_filtered_sor_last);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_last (new pcl::PointCloud<pcl::PointNormal>);
	  		estimateNormals(cloud_filtered_sor_last, cloud_normals_last);
			
			calculateKeyPoints(cloud_filtered_sor_last, cloud_normals_last, keypoints_last);
	  		calculateFeatures(cloud_filtered_sor_last, keypoints_last, cloud_normals_last, fpfhs_left);
			first = false;
		}
	}
}

void simpleVis (){
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	while(!viewer.wasStopped()){
		viewer.showCloud (visu_pc);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation");
	ros::NodeHandle nh;
	ros::NodeHandle imageNode;
	RobotDriver driver(nh);
	
	ros::Subscriber imageSub = imageNode.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);
	boost::thread t(simpleVis);
	driver.driveKeyboard();

	ros::shutdown();
}


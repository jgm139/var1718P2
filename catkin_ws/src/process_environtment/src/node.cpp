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
#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>
#include <math.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > filteredClouds;
std::vector<Eigen::Matrix4f> transformations;

void simpleVis (){
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	while(!viewer.wasStopped()){
		viewer.showCloud (visu_pc);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

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
	
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  	fpfh.setInputCloud (keypoints_xyzrgb);
  	fpfh.setInputNormals (cloud_normals);
	fpfh.setSearchSurface (cloud);

  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
	fpfh.setSearchMethod(tree);

  	fpfh.setRadiusSearch (0.07);

  	fpfh.compute (*fpfhs);
}

void matchFeatures(pcl::PointCloud<pcl::FPFHSignature33>::Ptr& source_features, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& target_features,
	pcl::Correspondences& correspondences){

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
	est.setInputSource(source_features);
	est.setInputTarget(target_features);
	est.determineReciprocalCorrespondences (correspondences);
}


void getTransformation(pcl::PointCloud<pcl::PointWithScale>::Ptr& source, pcl::PointCloud<pcl::PointWithScale>::Ptr& target,
	pcl::Correspondences& correspondences, Eigen::Matrix4f& transformation){
	
	pcl::registration::TransformationEstimationSVD<pcl::PointWithScale, pcl::PointWithScale> teSVD;
	teSVD.estimateRigidTransformation(*source, *target, correspondences, transformation);
}

Eigen::Matrix4f correspondencesRejection(pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints_1, pcl::PointCloud<pcl::PointWithScale>::Ptr& keypoints_2,
	pcl::Correspondences& correspondences, pcl::Correspondences& correspondences_out){ 
	
	pcl::CorrespondencesConstPtr correspondences_p(new pcl::Correspondences(correspondences));

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> sac;
    sac.setInputSource(keypoints_1); // reverse? (2 for 1)
    sac.setInputTarget(keypoints_2); // reverse (1 for 2) 
    sac.setInlierThreshold(0.025); // Too much? default 0.01 
    sac.setMaximumIterations(10000); //default 1000 
    sac.setRefineModel(true); // default false 
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


void processAllFiles(){


	int size = 10;

	char* message = " Te quiero Julia ";
	int n = strlen(message);

	int print_line = 4;
	
	printf("\n");
	printf("\n");

	for (int x = 0; x < size; x++)
	{
		for (int y = 0; y <= 4 * size; y++)
		{
			double dist1 = sqrt(pow(x - size, 2) + pow(y - size, 2));
			double dist2 = sqrt(pow(x - size, 2) + pow(y - 3 * size, 2));

			if (dist1 < size + 0.5 || dist2 < size + 0.5)
				printf("\033[1;31m*");
			else
				printf("\033[1;31m ");
		}
		printf("\n");
	}

	for (int x = 1; x < 2 * size; x++)
	{
		for (int y = 0; y < x; y++)
			printf(" ");

		for (int y = 0; y < 4 * size + 1 - 2 * x; y++)
		{
			if (x >= print_line - 1 && x <= print_line + 1)
			{
				int idx = y - (4 * size - 2 * x - n) / 2;
				if (idx < n && idx >= 0)
				{
					if (x == print_line)
						printf("%c", message[idx]);
					else
						printf(" ");
				}
				else
					printf("*");
			}
			else
				printf("*");
		}
		printf("\n");
	}


	for(unsigned i=0;i<clouds.size()-1;i++){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_left (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_right (new pcl::PointCloud<pcl::PointXYZRGB>);
  		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_left (new pcl::PointCloud<pcl::PointNormal>);
  		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_right (new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale> ());
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale> ());
  		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_left (new pcl::PointCloud<pcl::FPFHSignature33> ());
  		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_right (new pcl::PointCloud<pcl::FPFHSignature33> ());
		pcl::Correspondences correspondences;
		pcl::Correspondences correspondencesRejected;
		
		// Filtramos los puntos
		filterCloudPoint(clouds[i], cloud_filtered_left);
		filterCloudPoint(clouds[i+1], cloud_filtered_right);
		
		// Conseguimos las normales
  		estimateNormals(cloud_filtered_left, cloud_normals_left);
  		estimateNormals(cloud_filtered_right, cloud_normals_right);

		// Sacamos puntos característicos
		calculateKeyPoints(cloud_filtered_left, cloud_normals_left, keypoints_left);
		calculateKeyPoints(cloud_filtered_right, cloud_normals_right, keypoints_right);

		// Calculamos features de los keypoints		
  		calculateFeatures(cloud_filtered_left, keypoints_left, cloud_normals_left, fpfhs_left);
  		calculateFeatures(cloud_filtered_right, keypoints_right, cloud_normals_right, fpfhs_right);

  		// Buscamos las correspondencias
  		matchFeatures(fpfhs_left, fpfhs_right, correspondences);


  		
  		// Filtramos las correspondencias
  		// Obtenemos la matriz de transformación
  		Eigen::Matrix4f transformation = correspondencesRejection(keypoints_left, keypoints_right, correspondences, correspondencesRejected);


  		cout << "\033[0m\033[1;33mProgress: " << i << "/" << clouds.size()-2 << "\033[0m" << endl;
  		cout << "\tCloud \033[1;35mleft\033[0m points in cloud after VG: \033[1;35m" << (*cloud_filtered_left).size() << "\033[0m" << endl;
  		cout << "\tCloud \033[1;35mleft\033[0m normal points in cloud: \033[1;35m" << (*cloud_normals_left).size() << "\033[0m" << endl;
  		cout << "\tCloud \033[1;35mleft\033[0m keypoints: \033[1;35m" << (*keypoints_left).size() << "\033[0m" << endl;
  		cout << "\tCloud \033[1;35mleft\033[0m descriptors: \033[1;35m" << (*fpfhs_left).size() << "\033[0m" << endl;
  		cout << "\tCloud \033[1;36mright\033[0m points in cloud after VG: \033[1;36m" << (*cloud_filtered_right).size() << "\033[0m" << endl;
  		cout << "\tCloud \033[1;36mright\033[0m normal points in cloud: \033[1;36m" << (*cloud_normals_right).size() << "\033[0m" << endl;
  		cout << "\tCloud \033[1;36mright\033[0m keypoints: \033[1;36m" << (*keypoints_right).size() << "\033[0m" << endl;
  		cout << "\tCloud \033[1;36mright\033[0m descriptors: \033[1;36m" << (*fpfhs_right).size() << "\033[0m" << endl;
  		cout << "------------------------------------------------" << endl;
  		cout << "\033[1;32mCorrespondences found: " << correspondences.size() << endl;
  		cout << "\033[1;31mCorrespondences rejected: " << correspondencesRejected.size() << "\033[0m" << endl;
  		cout << "------------------------------------------------" << endl << endl;
  		
  		//getTransformation(keypoints_left, keypoints_right, correspondences, transformation);
		transformations.push_back(transformation);
		filteredClouds.push_back(cloud_filtered_left);
		if(i==clouds.size()-2){
			filteredClouds.push_back(cloud_filtered_right);
		}
	}
}

void alignAll(){
	cout << "Aligning..." << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*filteredClouds[0], *transformed_cloud_out, transformations[0]);
	*visu_pc = *transformed_cloud_out+*filteredClouds[1];
	cout << "\033[1;33m\t" << 0 << setw(2) << "/ " << transformations.size()-1 << "\r\033[0m" << std::flush;
	for(unsigned int i=1;i<transformations.size();i++){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_inside (new pcl::PointCloud<pcl::PointXYZRGB>);
    	pcl::transformPointCloud (*visu_pc, *transformed_cloud_inside, transformations[i]);
		*visu_pc = *filteredClouds[i+1]+*transformed_cloud_inside;
		cout << "\033[1;33m\t" << i << setw(2) << "/ " << transformations.size()-1 << "\r\033[0m" << std::flush;
	}
	cout << endl;
	cout << "\033[1;31mAligning Done.\033[0m" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
	filterCloudPoint(visu_pc, final);
	visu_pc = final;
}


bool loadPointClouds(){
	std::ifstream f;
	std::string fileName;
	f.open("listOfPointClouds.txt");
	if(f.is_open()){
		while(std::getline(f, fileName)){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fileName, *cloud) == -1){
				cout << "Couldn't read file " << fileName << "\n";
				clouds.clear();
				return false;
			}
			clouds.push_back(cloud);
		}
	}
	f.close();
	return true;
}

int main(int argc, char** argv){
	
	if(loadPointClouds()){
		processAllFiles();
		alignAll();
		simpleVis();
	}

	return -1;
}

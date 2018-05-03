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
#include <pcl/features/board.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/keypoints/iss_3d.h>
#include <vector>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds;
std::vector<pcl::PointCloud<pcl::SHOT352>::Ptr > descriptors;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > keypoints;
bool capture = false;
bool align = false;

void simpleVis (){
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	while(!viewer.wasStopped()){
		viewer.showCloud (visu_pc);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

void saveCloudPoints(){
	for(unsigned int i = 0;i<clouds.size();i++){
		pcl::io::savePCDFileBinary("test_pcd.pcd", *clouds[i]);
		std::cerr << "Saved " << (*clouds[i]).size() << " data points to test_pcd.pcd." << std::endl;
	}
}

void captureImage(){
	while(true){
		std::cout << "Pulsar para capturar nube de puntos e incluirla al vector" << std::endl;
		char option = getchar(); 
		switch(option){
			case 'a':
				align = true;
				break;
			case 's':
				saveCloudPoints();
				break;
			case 10: //Enter, no hacer nada
				break;
			default:
				capture = true;
				break;
		}
	}
}

void calculateKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints){
		// ISS keypoint detector object.
		pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> detector;
		detector.setInputCloud(cloud);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		detector.setSearchMethod(kdtree);
		
		double resolution = computeCloudResolution(cloud);
		// Set the radius of the spherical neighborhood used to compute the scatter matrix.
		detector.setSalientRadius(6 * resolution);
		// Set the radius for the application of the non maxima supression algorithm.
		detector.setNonMaxRadius(4 * resolution);
		// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
		detector.setMinNeighbors(5);
		// Set the upper bound on the ratio between the second and the first eigenvalue.
		detector.setThreshold21(0.975);
		// Set the upper bound on the ratio between the third and the second eigenvalue.
		detector.setThreshold32(0.975);
		// Set the number of prpcessing threads to use. 0 sets it to automatic.
		detector.setNumberOfThreads(4);
		detector.compute(*keypoints);
}

// Using NormalFeature
void calculateFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals){
	
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud_filtered);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.07);

	// Compute the features
	ne.compute (*cloud_normals);
}


void ransac/*?*/(){
	// Método de mío - Sin testear
	/*std::vector<int> inliers;

  	pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGB> (cloud));
  	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *final);*/
}

/*void matchFeatures(){
	pcl::CorrespondenceEstimation<pcl::Normal, pcl::Normal> est;
	est.setInputCloud(source_features);
	est.setInputTarget(target_features);
	est.determineCorrespondences(correspondences);
}*/
// Using Sphere model
void performAlign(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_cloud, pcl::PointCloud<pcl::SHOT352>::Ptr& source_features, 
	pcl::PointCloud<pcl::SHOT352>::Ptr& target_features, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aligned_cloud, 
	Eigen::Matrix4f& transformation){

	
    // Método de mío
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource (input_cloud);
	icp.setInputTarget (target_cloud);
	icp.align (*aligned_cloud);

	transformation = icp.getFinalTransformation ();
    

    // Método de mío - Refinar (supuestos buenos resultados)
    /*pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::SHOT352> sac_ia;
	sac_ia.setNumberOfSamples (3);
	sac_ia.setMinSampleDistance (0.07f);
	sac_ia.setCorrespondenceRandomness (5);
	sac_ia.setMaximumIterations (5000);
	sac_ia.setInputSource (input_cloud);
	sac_ia.setInputTarget (target_cloud);
	sac_ia.setSourceFeatures (source_features);
	sac_ia.setTargetFeatures (target_features);
	sac_ia.align(*aligned_cloud);
	transformation = sac_ia.getFinalTransformation ();*/

	//std::vector<int> inliers;
	/* Método de david
    registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> sac;
    sac.setInputSource(keypoints_2); // reverse? (2 for 1) 
    sac.setInputTarget(keypoints_1); // reverse (1 for 2) 
    sac.setInlierThreshold(0.07); // Too much? default 0.01 
    sac.setMaximumIterations(2000); //default 1000 
    sac.setRefineModel(false); // default false 
    sac.setInputCorrespondences(correspondences); 
    sac.getCorrespondences(*inliers); 
    Eigen::Matrix4f transformation = refiner.getBestTransformation();
	*/

    //pcl::transformPointCloud (*input_cloud, *transformed_cloud, transformation);

}

void calculateDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints,
 pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals, pcl::PointCloud<pcl::SHOT352>::Ptr& result){

	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descr_est;
    descr_est.setRadiusSearch (0.07f); //descr_rad
	descr_est.setInputCloud (keypoints);
	descr_est.setInputNormals (cloud_normals);
	descr_est.setSearchSurface (cloud);
	descr_est.compute (*result);
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	if(capture){
		capture = false;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

		// Filtramos los puntos
		pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
		vGrid.setInputCloud (cloud);
		vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
		vGrid.filter (*cloud_filtered);

		std::cout << "Cloud: " << cloud_filtered->size();

		// Conseguimos los KeyPoints
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint(new pcl::PointCloud<pcl::PointXYZRGB>);
		calculateKeyPoints(cloud_filtered, keypoint);
		std::cout << "KeyPoints: " << keypoint->size();
		// Calculamos las Features
		/// Mirar otras que no sean normales///
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		calculateFeatures(cloud, cloud_filtered, cloud_normals);
		cout << "Normal Clouds: " << cloud_normals->size();

		// Calculamos los descriptores
		pcl::PointCloud<pcl::SHOT352>::Ptr descriptor (new pcl::PointCloud<pcl::SHOT352> ());
		calculateDescriptors(cloud_filtered, keypoint, cloud_normals, descriptor);
		cout << "Descriptor: " << descriptor->size();
	
		// Alineamos
		visu_pc = cloud_filtered;
		clouds.push_back(cloud_filtered);
		keypoints.push_back(keypoint);
		descriptors.push_back(descriptor);
	}
	if(align){
		align = false;
		if(!clouds.size() < 2){
			Eigen::Matrix4f transformation;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			for(unsigned i = 0;i<keypoints.size()-1;i++){
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
				performAlign(keypoints[i]/*1*/, keypoints[i+1]/*2*/, descriptors[i]/*1*/, descriptors[i+1]/*2*/, final, transformation);
				//pcl::transformPointCloud (*clouds[i], *transformed_cloud, transformation);
				*visu_pc = *clouds[i]+*clouds[i+1];
			}
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

	boost::thread t(simpleVis);
	boost::thread t2(captureImage);

	while(ros::ok()){
		ros::spinOnce();
	}

}

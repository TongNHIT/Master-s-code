// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

/* This examples shows how to estimate the SIFT points based on the
 * z gradient of the 3D points than using the Intensity gradient as
 * usually used for SIFT keypoint estimation.
 */

namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float
			operator () (const PointXYZ &p) const
		{
			return p.z;
		}
	};
}

int
main(int, char** argv)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/pointcloud/dianyunliming/rabbit_t.pcd", *cloud_xyz) == -1) // load the file
	{
		PCL_ERROR("Couldn't read file");
		return -1;
	}
	std::cout << "points: " << cloud_xyz->points.size() << std::endl;

	// Parameters for sift computation
	const float min_scale = 0.005f; //the standard deviation of the smallest scale in the scale space
	const int n_octaves = 6;//the number of octaves (i.e. doublings of scale) to compute,6
	const int n_scales_per_octave = 4;//the number of scales to compute within each octave,4
	const float min_contrast = 0.005f;//the minimum contrast required for detection


	pcl::console::TicToc time;
	time.tic();
	// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_xyz);
	sift.compute(result);
	std::cout << "Computing the SIFT points takes " << time.toc() / 1000 << "seconds" << std::endl;
	std::cout << "No of SIFT points in the result are " << result.points.size() << std::endl;


	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(result, *cloud_temp);
	std::cout << "SIFT points in the result are " << cloud_temp->points.size() << std::endl;
	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud_temp, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_xyz, 255, 0, 0);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud_xyz, cloud_color_handler, "cloud");//add point cloud
	viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");//add the keypoints 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}


	return 0;

}
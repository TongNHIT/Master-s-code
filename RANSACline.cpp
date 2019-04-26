#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>  //随机样本一致性算法 方法类型
#include <pcl/sample_consensus/model_types.h>       //随机样本一致性算法 模型类型
#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法

using namespace std::chrono_literals;

int
main(int argc, char** argv)
{
	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/pointcloud/RGBD/kinect4.10/yiziboudary1.pcd", *cloud) == -1)
	{
		PCL_ERROR("COULD NOT READ FILE mid.pcl \n");
		return (-1);
	}

	std::cout << "points sieze is:" << cloud->size() << std::endl;

	//std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::ModelCoefficients line;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.005);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, line);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud1);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	int v1 = 0;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addCoordinateSystem(0.1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v1_color(cloud1, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(cloud1, v1_color, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(trackpoint, slicenormal, 1, 1, "normal", v1);
	int v2 = 0;
	//viewer->addCoordinateSystem(1.0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, v2_color, "slices", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "slices");
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(slicecloud, slicenormal, 1, 1, "normals", v2);
	viewer->initCameraParameters();
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
	return 0;
}
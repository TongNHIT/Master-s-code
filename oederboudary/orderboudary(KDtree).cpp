
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <iostream>
#include <vector>
#include <ctime>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "reading pcd file...\n";
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCLData/pointcloud/cloud_cluster_1.pcd", *cloud) == -1)   //打开点云文件
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}                                                                   //从磁盘上加载点云数据到二进制存储块中
	std::cout << "Loaded " << cloud->width*cloud->height << " data points from test_pcd.pcd .  "
		<< "  width: " << cloud->width << "  height: " << cloud->height << std::endl;



	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointXYZ searchPoint;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point(new pcl::PointCloud<pcl::PointXYZ>);
	searchPoint.x = cloud->points[0].x;
	searchPoint.y = cloud->points[0].y;
	searchPoint.z = cloud->points[0].z;
	pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
	cloud->erase(index);
	point->points.push_back(searchPoint);
	std::cout << "  searchpoint0 is   " << point->points[0].x
		<< " " << point->points[0].y << " " << point->points[0].z << "left points" << cloud->points.size() << std::endl;
	int K = 2;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	int ps = cloud->points.size();
	for (size_t i = 0; i < ps - 1; ++i)
	{
		kdtree.setInputCloud(cloud);
		kdtree.nearestKSearch(point->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
		{
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
				<< " " << cloud->points[pointIdxNKNSearch[i]].y
				<< " " << cloud->points[pointIdxNKNSearch[i]].z
				<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << pointIdxNKNSearch[i] << std::endl;
		}
		searchPoint.x = cloud->points[pointIdxNKNSearch[0]].x;
		searchPoint.y = cloud->points[pointIdxNKNSearch[0]].y;
		searchPoint.z = cloud->points[pointIdxNKNSearch[0]].z;
		index = cloud->begin() + pointIdxNKNSearch[0];
		cloud->erase(index);
		point->points.push_back(searchPoint);
		std::cout << "  point is   " << point->points[i + 1].x << " " << point->points[i + 1].y << " " << point->points[i + 1].z << " " << i + 1 << "left points" << cloud->points.size() << std::endl;

	}
	searchPoint.x = cloud->points[0].x;
	searchPoint.y = cloud->points[0].y;
	searchPoint.z = cloud->points[0].z;
	point->points.push_back(searchPoint);
	point->width = (int)point->points.size();
	point->height = 1;

	pcl::io::savePCDFileASCII("F:/PCLData/pointcloud/orderboundary1.pcd", *point);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	int v1 = 0;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->addCoordinateSystem(0.1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v1_color(cloud, 0, 0, 255);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> slice_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, v1_color, "cloud", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, slice_color, "slice", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "slice");
	int v2 = 0;
	//viewer->addCoordinateSystem(1.0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(point, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(point, v2_color, "slices", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "slices");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spin();
	}
	system("pause");
	return 0;

}
#include "vtkAutoInit.h"   //VTK显示法向量报错
#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/normal_refinement.h>
void normalref(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points,
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal,
	pcl::PointCloud<pcl::Normal>::Ptr normal_out)
{
	//kdtree search
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_points);
	int k = 10;
	std::vector<std::vector<int> > k_indices;
	std::vector<std::vector<float> > k_sqr_distances;
	// Run search
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud_points);
	tree->nearestKSearch(*cloud_points, std::vector<int>(), k, k_indices, k_sqr_distances);

	unsigned int max_iterations = 20;
	for (unsigned int it = 0; it < max_iterations; ++it)
	{
		pcl::PointCloud<pcl::Normal>temp = *cloud_normal;
		for (size_t i = 0; i < cloud_points->points.size(); ++i)
		{
			pcl::Normal tempnorm;
			tempnorm.normal_x = 0.0f;
			tempnorm.normal_y = 0.0f;
			tempnorm.normal_z = 0.0f;
			double G = 0;  //归一化系数（分母）
			//计算空间光顺高斯函数
			for (int j = 0; j < k_indices[i].size(); j++)
			{
				//计算G1
				double G1 = exp(-(k_sqr_distances[i][j]) / 2.25);  //2.25为权重因子
				//计算G2
				//法向量点积
				double nxn = cloud_normal->points[i].normal_x * cloud_normal->points[k_indices[i][j]].normal_x
					+ cloud_normal->points[i].normal_y * cloud_normal->points[k_indices[i][j]].normal_y
					+ cloud_normal->points[i].normal_z * cloud_normal->points[k_indices[i][j]].normal_z;
				double G2 = exp(-(1 - nxn)*(1 - nxn) / (1 - cos(M_PI / 12))*(1 - cos(M_PI / 12)));
				//G += G1 * G2;
				G += G1;
				tempnorm.normal_x += cloud_normal->points[k_indices[i][j]].normal_x * G1;
				tempnorm.normal_y += cloud_normal->points[k_indices[i][j]].normal_y * G1;
				tempnorm.normal_z += cloud_normal->points[k_indices[i][j]].normal_z * G1;
			}
			tempnorm.normal_x = tempnorm.normal_x / G;
			tempnorm.normal_y = tempnorm.normal_y / G;
			tempnorm.normal_z = tempnorm.normal_z / G;
			//normalize...
			const float norm = std::sqrt(tempnorm.normal_x * tempnorm.normal_x + tempnorm.normal_y * tempnorm.normal_y + tempnorm.normal_z * tempnorm.normal_z);

			temp.points[i].normal_x = tempnorm.normal_x / norm;
			temp.points[i].normal_y = tempnorm.normal_y / norm;
			temp.points[i].normal_z = tempnorm.normal_z / norm;
		}
		*cloud_normal = temp;
	}
	*normal_out = *cloud_normal;
}

void getslicenormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points,
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal,
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice,
	pcl::PointCloud<pcl::Normal>::Ptr slicenormal)
{
	int k = 10;
	std::vector<std::vector<int> > k_indices;
	std::vector<std::vector<float> > k_sqr_distances;
	// Run search
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud_points);
	tree->nearestKSearch(*slice, std::vector<int>(), k, k_indices, k_sqr_distances);
		
	for (size_t i = 0; i < slice->points.size(); ++i)
	{
		pcl::Normal tempnorm;
		tempnorm.normal_x = 0.0f;
		tempnorm.normal_y = 0.0f;
		tempnorm.normal_z = 0.0f;
		double G = 0;  //归一化系数（分母）
		//计算空间光顺高斯函数
		for (int j = 0; j < k_indices[i].size(); j++)
		{
			//计算G1
			double G1 = exp(-(k_sqr_distances[i][j]) / 2.25);  //2.25为权重因子
			//计算G2
			//法向量点积
		//	double nxn = cloud_normal->points[i].normal_x * cloud_normal->points[k_indices[i][j]].normal_x
		//		+ cloud_normal->points[i].normal_y * cloud_normal->points[k_indices[i][j]].normal_y
		//		+ cloud_normal->points[i].normal_z * cloud_normal->points[k_indices[i][j]].normal_z;

			//double G2 = exp(-(1 - nxn)*(1 - nxn) / (1 - cos(M_PI / 12))*(1 - cos(M_PI / 12)));
			//G += G1 * G2;

			G += G1;
			tempnorm.normal_x += cloud_normal->points[k_indices[i][j]].normal_x * G1;
			tempnorm.normal_y += cloud_normal->points[k_indices[i][j]].normal_y * G1;
			tempnorm.normal_z += cloud_normal->points[k_indices[i][j]].normal_z * G1;
		}
		tempnorm.normal_x = tempnorm.normal_x / G;
		tempnorm.normal_y = tempnorm.normal_y / G;
		tempnorm.normal_z = tempnorm.normal_z / G;
		//normalize...
		const float norm = std::sqrt(tempnorm.normal_x * tempnorm.normal_x + tempnorm.normal_y * tempnorm.normal_y + tempnorm.normal_z * tempnorm.normal_z);

		tempnorm.normal_x = tempnorm.normal_x / norm;
		tempnorm.normal_y = tempnorm.normal_y / norm;
		tempnorm.normal_z = tempnorm.normal_z / norm;
		slicenormal->points.push_back(tempnorm);
	}
		slicenormal->width = (int)slicenormal->points.size();
		slicenormal->height = 1;
}
int main()
{
	VTK_MODULE_INIT(vtkRenderingOpenGL);   //VTK显示法向量报错
	VTK_MODULE_INIT(vtkInteractionStyle);  //VTK显示法向量报错
	// Input point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>());
	// Fill cloud...
	pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/pointcloud/test/cloudout.pcd", *cloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/pointcloud/test/slices.pcd", *slice);
	// Estimated and refined normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_refined(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_refined1(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr slicenormal(new pcl::PointCloud<pcl::Normal>);
	// Search 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud);
	//normalEstimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setViewPoint(-1, -1, 0);
	ne.setSearchMethod(tree);
	ne.setKSearch(25);
	ne.compute(*normals);
	*normals1 = *normals;
	normalref(cloud, normals1, normals_refined);
	
	getslicenormal(cloud, normals_refined,slice, slicenormal);

	//const int k = 5;
	//std::vector<std::vector<int> > k_indices;
	//std::vector<std::vector<float> > k_sqr_distances;
	// Run search
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
	//tree1->setInputCloud(cloud);
	//tree1->nearestKSearch(*cloud, std::vector<int>(), k, k_indices, k_sqr_distances);
	//pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
//	nr.setInputCloud(normals);
	//nr.filter(*normals_refined1);


	//pcl::io::savePCDFileASCII("testnormal.pcd", *normals_refined);

	//Visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	int v1 = 0;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addCoordinateSystem(1.0);
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v1_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, v1_color, "cloud1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.02, "normals", v1);
	int v2 = 0;
	//viewer->addCoordinateSystem(1.0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, v2_color, "cloud2", v2);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(slice, slicenormal, 1, 0.02, "normals_refined", v2);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spin();
	}
	return 0;
}
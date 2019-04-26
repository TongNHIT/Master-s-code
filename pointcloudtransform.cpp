/*点云坐标转换*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
	//载入文件
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("F:/PCLData/pointcloud/RGBD/ceshi_chulihou3.pcd", *source_cloud);
	std::cout << "loading point cloud " << std::endl;

	/* Reminder: how transformation matrices work :

			 |-------> This column is the translation
	  | 1 0 0 x |  \
	  | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
	  | 0 0 1 z |  /
	  | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

	  METHOD #1: Using a Matrix4f
	  This is the "manual" method, perfect to understand but error prone !
	*/
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	// Define a rotation matrix 定义旋转的角度  再有角度计算出旋转矩阵
	float theta = M_PI / 4; // The angle of rotation in radians
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);
	//    (row, column)

	// Define a translation of 2.5 meters on the x axis.
	transform_1(0, 3) = 2.5;//意思就是在第一行第四个元素的值为2.5，也就是在x轴的平移为2.5

	// Print the transformation  打印出这个变换矩阵
	printf("Method #1: using a Matrix4f\n");
	std::cout << transform_1 << std::endl;

	/*  METHOD #2: Using a Affine3f  第二种方案
	  This method is easier and less error prone  更简单的方案
	  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << 2.5, 0.0, 0.0;

	// The same rotation matrix as before; theta radians arround Z axis
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

	// Print the transformation
	printf ("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;
	*/
	//方法三，通过轴对应的整数变换
	/*
	PointType point;
	std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
	for (int i = 0; i < cloudSize; i++) {
		point.x = laserCloudIn.points[i].y; // y轴换到x轴
		point.y = laserCloudIn.points[i].z; // z轴换到y轴
		point.z = laserCloudIn.points[i].x; // x轴换到z轴
		float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
		int scanID;
		int roundedAngle = int(angle + (angle < 0.0 ? -0.5 : +0.5));
		if (roundedAngle > 0) {
			scanID = roundedAngle;
		}
		else {
			scanID = roundedAngle + (N_SCANS - 1);
		}
		if (scanID > (N_SCANS - 1) || scanID < 0) {
			count--;
			continue;
		}
		*/

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// 你可以使用 transform_1 或者 transform_2;效果都是一样的 
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

	// 可视化的
	printf("\nPoint cloud colors :  white  = original point cloud\n"
		"                             red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	// 为点云设置RGB的值
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem(1.0,"reference", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); //设置背景颜色
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}
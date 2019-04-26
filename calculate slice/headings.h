#pragma once
#ifndef HEADINGS_H
#define HEADINGS_H


#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_visualizer.h>

void normalsolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr slicein,
	pcl::PointCloud<pcl::PointNormal>::Ptr slice_with_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr sliceout,
	pcl::PointCloud<pcl::Normal>::Ptr slicenormal);

void showslicewithnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudo, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudt,
	pcl::PointCloud<pcl::PointXYZ>::Ptr slicein, pcl::PointCloud<pcl::Normal>::Ptr slicenormal);

void showviewintwodifferentwindow(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right);





#endif//HEADINGS_H







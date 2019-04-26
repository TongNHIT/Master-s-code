#pragma once
#ifndef HEADERFILES_H
#define HEADERFILES_H

// kinect文件
#include "kinect.h"

//OpenCV
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>

//点云文件IO（pcd文件和ply文件）     
#include <iostream> //标准输入输出流  
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件  
#include <pcl/io/ply_io.h>  

#include <pcl/visualization/pcl_visualizer.h>   
#include <pcl/visualization/cloud_viewer.h>  

//多线程    
#include <boost/thread/thread.hpp>    
#include <fstream>    
#include <iostream>    
#include <stdio.h>    
#include <string.h>    
#include <string>   
#include <time.h>
#include <windows.h>
//程序计时    
#include <time.h>  

//矩阵处理
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/visualization/histogram_visualizer.h>

#include <pcl/features/cvfh.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>
#include <cstdlib>

//点云滤波库
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>


//kd树    
#include <pcl/kdtree/kdtree_flann.h>    
//特征提取    
#include <pcl/features/normal_3d_omp.h>       
//重构    
#include <pcl/surface/gp3.h>    
#include <pcl/surface/poisson.h>    
//Bilateral Filter  
#include <pcl/filters/bilateral.h>//required  
#include <pcl/filters/fast_bilateral.h>  
#include <pcl/filters/fast_bilateral_omp.h>  

//形态学滤波
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

//法线
#include <pcl/features/normal_3d.h>
//点云类型转化
//#include <pcl/common/io/io.h>
//有序点云处理
#include <pcl/features/integral_image_normal.h>   

#include <pcl/ModelCoefficients.h>  //模型系数

//随机采样一致性算法
#include <pcl/sample_consensus/method_types.h>  //随机样本一致性算法 方法类型
#include <pcl/sample_consensus/model_types.h>       //随机样本一致性算法 模型类型
#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法

//聚类欧式分割
#include <pcl/segmentation/extract_clusters.h>

#include<pcl/visualization/pcl_plotter.h>

#include <pcl/features/vfh.h>

#include <boost/make_shared.hpp>

//网格滤波用于均一点云密度（可用于去除点云重合部分）
#include <pcl/filters/voxel_grid.h>
#include <limits>
#include <fstream>
#include <vector>
static int user_data;//因为怕重复定义,所以使用静态变量
using namespace cv;
using namespace std;

//Kinect相机参数
const double camera_factor = 1000;
const double camera_cx = 259.28;
const double camera_cy = 207.4573;
const double camera_fx = 371.3605;
const double camera_fy = 368.6749;
const double camera_k1 = 0.1067;
const double camera_k2 = -0.2421;
const double camera_p1 = 0;
const double camera_p2 = 0;



// 安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
#endif//HEADERFILES_H
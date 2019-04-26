#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//kinect文件
#include "kinect.h"

//OpenCV
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <vector>
//点云文件IO（pcd文件和ply文件）     
#include <iostream> //标准输入输出流  
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件  
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>  
//点云滤波库
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
//kd树    
#include <pcl/kdtree/kdtree_flann.h>    
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
//octree
#include <pcl/octree/octree.h>
//特征提取    
#include <pcl/features/normal_3d_omp.h>    
#include <pcl/features/normal_3d.h>    
#include <pcl/features/don.h>
//重构    
#include <pcl/surface/gp3.h>    
#include <pcl/surface/poisson.h>    
//可视化    
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
//Bilateral Filter  
#include <pcl/filters/bilateral.h>//required  
#include <pcl/filters/fast_bilateral.h>  
#include <pcl/filters/fast_bilateral_omp.h>  
//形态学滤波
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
//法线
#include <pcl/features/normal_3d.h>
//MLS
#include <pcl/surface/mls.h>
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
#include <boost/make_shared.hpp>
using namespace std;
static int user_data;//为什么使用静态变量呢，因为怕重复定义
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


//////////////////////////////////////点云处理////////////////////////////////////////////

//利用统计学方法去除点云大噪声，k为k搜索之，t为保存几倍方差之内的点
void reducewrongpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout, int k, int t);
void cloudchange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB);
//利用PCA算法获得点云曲率和法线，搜索区域为K
void getnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointxyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, int k);
//条件滤波
void tiaojian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
//VoxelGrid下采样
void VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

//直通滤波	
void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
//基于法线差的滤波
void filterbynormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double scale1, double scale2, double threshold);
//利用双边滤波方法进行滤波，简单粗暴，求位置求平均值，颜色保存
void shuangbian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const pcl::PointCloud<pcl::Normal>::Ptr normal);

//利用三边滤波算法进行滤波
//三边算法,输入为彩色点云和Normal
void sanbian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, const pcl::PointCloud<pcl::Normal>::Ptr PCLnormal);

//利用改进的三边滤波算法进行滤波
//修正那个点云normal
void normalchange(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, pcl::PointCloud<pcl::Normal>::Ptr normal_out);

void normalrefine(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, pcl::PointCloud<pcl::Normal>::Ptr normal_out);
//半径滤波
void radiusfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double r, int a);

//去除平面(距离、迭代、概率)

void reduceplant(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double d, int Iteration, int Probability);
/////////////////////////////////////点云显示/////////////////////////////////////////////

//为一个仅执行一次的函数，作用为设置背景，放置球体。调用方法为：viewer.runOnVisualizationThreadOnce (viewerOneOff)
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer);

//作为回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串，调用方法是viewer.runOnVisualizationThread (viewerPsycho);每次刷新都调用
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer);

//在两个窗口中显示相同角度的彩色点云
void showviewintwodifferentwindow(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right);


//在4个窗口中显示相同角度的彩色点云XYZRGBA
void showviewinfourdifferentwindow(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4);//输入两个XYZRGBA

//在一个窗口内显示彩色点云和曲率法线(RGBA，XYZ，Normal)
void showcloudwithnormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
//在一个窗口显示自定义颜色的点云和法线
void showrgbcloudwithnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);


#endif MAINWINDOW_H




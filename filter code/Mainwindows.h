#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//kinect�ļ�
#include "kinect.h"

//OpenCV
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <vector>
//�����ļ�IO��pcd�ļ���ply�ļ���     
#include <iostream> //��׼���������  
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�  
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�  
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>  
//�����˲���
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
//kd��    
#include <pcl/kdtree/kdtree_flann.h>    
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
//octree
#include <pcl/octree/octree.h>
//������ȡ    
#include <pcl/features/normal_3d_omp.h>    
#include <pcl/features/normal_3d.h>    
#include <pcl/features/don.h>
//�ع�    
#include <pcl/surface/gp3.h>    
#include <pcl/surface/poisson.h>    
//���ӻ�    
#include <pcl/visualization/pcl_visualizer.h>   
#include <pcl/visualization/cloud_viewer.h>  
//���߳�    
#include <boost/thread/thread.hpp>    
#include <fstream>    
#include <iostream>    
#include <stdio.h>    
#include <string.h>    
#include <string>   
#include <time.h>
#include <windows.h>
//�����ʱ    
#include <time.h>  
//Bilateral Filter  
#include <pcl/filters/bilateral.h>//required  
#include <pcl/filters/fast_bilateral.h>  
#include <pcl/filters/fast_bilateral_omp.h>  
//��̬ѧ�˲�
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
//����
#include <pcl/features/normal_3d.h>
//MLS
#include <pcl/surface/mls.h>
//��������ת��
//#include <pcl/common/io/io.h>
//������ƴ���
#include <pcl/features/integral_image_normal.h>   
#include <pcl/ModelCoefficients.h>  //ģ��ϵ��
//�������һ�����㷨
#include <pcl/sample_consensus/method_types.h>  //�������һ�����㷨 ��������
#include <pcl/sample_consensus/model_types.h>       //�������һ�����㷨 ģ������
#include <pcl/segmentation/sac_segmentation.h>  //�������һ�����㷨 �ָ��
//����ŷʽ�ָ�
#include <pcl/segmentation/extract_clusters.h>
#include <boost/make_shared.hpp>
using namespace std;
static int user_data;//Ϊʲôʹ�þ�̬�����أ���Ϊ���ظ�����
// ��ȫ�ͷ�ָ��
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


//////////////////////////////////////���ƴ���////////////////////////////////////////////

//����ͳ��ѧ����ȥ�����ƴ�������kΪk����֮��tΪ���漸������֮�ڵĵ�
void reducewrongpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout, int k, int t);
void cloudchange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB);
//����PCA�㷨��õ������ʺͷ��ߣ���������ΪK
void getnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointxyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, int k);
//�����˲�
void tiaojian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
//VoxelGrid�²���
void VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

//ֱͨ�˲�	
void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
//���ڷ��߲���˲�
void filterbynormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double scale1, double scale2, double threshold);
//����˫���˲����������˲����򵥴ֱ�����λ����ƽ��ֵ����ɫ����
void shuangbian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const pcl::PointCloud<pcl::Normal>::Ptr normal);

//���������˲��㷨�����˲�
//�����㷨,����Ϊ��ɫ���ƺ�Normal
void sanbian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, const pcl::PointCloud<pcl::Normal>::Ptr PCLnormal);

//���øĽ��������˲��㷨�����˲�
//�����Ǹ�����normal
void normalchange(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, pcl::PointCloud<pcl::Normal>::Ptr normal_out);

void normalrefine(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, pcl::PointCloud<pcl::Normal>::Ptr normal_out);
//�뾶�˲�
void radiusfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double r, int a);

//ȥ��ƽ��(���롢����������)

void reduceplant(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double d, int Iteration, int Probability);
/////////////////////////////////////������ʾ/////////////////////////////////////////////

//Ϊһ����ִ��һ�εĺ���������Ϊ���ñ������������塣���÷���Ϊ��viewer.runOnVisualizationThreadOnce (viewerOneOff)
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer);

//��Ϊ�ص�����������������ע���ÿ֡��ʾ��ִ��һ�Σ���������ʵ���ڿ��ӻ����������һ��ˢ����ʾ�ַ��������÷�����viewer.runOnVisualizationThread (viewerPsycho);ÿ��ˢ�¶�����
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer);

//��������������ʾ��ͬ�ǶȵĲ�ɫ����
void showviewintwodifferentwindow(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right);


//��4����������ʾ��ͬ�ǶȵĲ�ɫ����XYZRGBA
void showviewinfourdifferentwindow(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4);//��������XYZRGBA

//��һ����������ʾ��ɫ���ƺ����ʷ���(RGBA��XYZ��Normal)
void showcloudwithnormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
//��һ��������ʾ�Զ�����ɫ�ĵ��ƺͷ���
void showrgbcloudwithnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);


#endif MAINWINDOW_H




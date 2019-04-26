#pragma once
#ifndef HEADERFILES_H
#define HEADERFILES_H

// kinect�ļ�
#include "kinect.h"

//OpenCV
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>

//�����ļ�IO��pcd�ļ���ply�ļ���     
#include <iostream> //��׼���������  
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�  
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�  
#include <pcl/io/ply_io.h>  

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

//������
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

//�����˲���
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>


//kd��    
#include <pcl/kdtree/kdtree_flann.h>    
//������ȡ    
#include <pcl/features/normal_3d_omp.h>       
//�ع�    
#include <pcl/surface/gp3.h>    
#include <pcl/surface/poisson.h>    
//Bilateral Filter  
#include <pcl/filters/bilateral.h>//required  
#include <pcl/filters/fast_bilateral.h>  
#include <pcl/filters/fast_bilateral_omp.h>  

//��̬ѧ�˲�
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

//����
#include <pcl/features/normal_3d.h>
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

#include<pcl/visualization/pcl_plotter.h>

#include <pcl/features/vfh.h>

#include <boost/make_shared.hpp>

//�����˲����ھ�һ�����ܶȣ�������ȥ�������غϲ��֣�
#include <pcl/filters/voxel_grid.h>
#include <limits>
#include <fstream>
#include <vector>
static int user_data;//��Ϊ���ظ�����,����ʹ�þ�̬����
using namespace cv;
using namespace std;

//Kinect�������
const double camera_factor = 1000;
const double camera_cx = 259.28;
const double camera_cy = 207.4573;
const double camera_fx = 371.3605;
const double camera_fy = 368.6749;
const double camera_k1 = 0.1067;
const double camera_k2 = -0.2421;
const double camera_p1 = 0;
const double camera_p2 = 0;



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
#endif//HEADERFILES_H
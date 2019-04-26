#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//定义一个读取类：读取txt文件中的点数据(控制点按逆时针排序)
//                计算控制多边形的凸包
//                计算两个模型的缩放比例
//                估算点云密度
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_READ_POINTS_H__
#define __LMZ_READ_POINTS_H__

#include <iostream>
#include <fstream>
#include <string>
#include <windows.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include "Function.h"
using namespace std;

///////////////////////////////////////////////////////////////////////////////
//读取类
class CJsGrRead
{
private:
	char filename[20];                 //存放文件名的数组
	float Center[3];                   //所有点的中心
public:
	int num;                           //txt文件中点的数量
	float(*Pts)[3];                   //存放数据点的指针数组 
	float XYZ_Scale[3];                //X,Y,Z方向的缩放比例
	float Max_pts[3];                  //最大坐标值数组(最右、最上、最前)
	float Min_pts[3];                  //最小坐标值数组(最左、最下、最后)
	float Scale;                       //缩放比例
	int N;                             //N：随机点的数量
	int M;                             //M：点云中抽取最近点的数量
	float(*S)[3];	                   //存放随机点集的数组
	float dens;                        //点云的估测密度
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::PointCloud<pcl::PointXYZ>::Ptr output{ new pcl::PointCloud<pcl::PointXYZ> };
	inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudin(char *filename);
	inline Eigen::Matrix4f trans(void);
	inline float Scale_PtCloud(void);  //计算点云模型的缩放比例
	inline float Dens_PtCloud(void);   //估算点云密度
	inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudtransformed(void);         //输出数据
};
CJsGrRead file1;

///////////////////////////////////////////////////////////////////////////////
//计算*.PCD文件中点的数量，读取控制点数据(控制点按逆时针排列)
inline pcl::PointCloud<pcl::PointXYZ>::Ptr CJsGrRead::cloudin(char *filename)
{
	num = 0;
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>(filename, *cloud1);
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud1, pcaCentroid);//计算质心
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud1, pcaCentroid, covariance);//计算3*3的协方差矩阵
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);//特征分解
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//协方差矩阵的特征向量
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//协方差矩阵的特征值
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	//Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud1, *transformedCloud, tm);//将点云变换到原点，主方向与坐标系方向重合

	vector<int> indices;
	pcl::removeNaNFromPointCloud(*transformedCloud, *output, indices);
	//读取文件中的点数据
	num = output->width*output->height;
	cout << filename << "模型共有" << num << "个点。" << endl;

	Pts = new float[num][3];
	for (int i = 0; i < num; i++)
	{
		Pts[i][0] = output->points[i].x;
		Pts[i][1] = output->points[i].y;
		Pts[i][2] = output->points[i].z;
	}
	
	return(cloud1);
}


inline Eigen::Matrix4f CJsGrRead::trans(void)
{
	
	return (tm_inv);
}


///////////////////////////////////////////////////////////////////////////////
//计算点云模型的各方向跨度
inline float CJsGrRead::Scale_PtCloud(void)
{
	int i, j;
	for (i = 0; i < 3; i++)
	{
		Max_pts[i] = Pts[0][i];
		Min_pts[i] = Pts[0][i];
	}
	//求最大坐标值(最右、最上、最前)
	for (j = 0; j < 3; j++)
		for (i = 0; i < num - 1; i++)
			if (Max_pts[j] < Pts[i + 1][j])
				Max_pts[j] = Pts[i + 1][j];
	//求最小坐标值(最左、最下、最后)
	for (j = 0; j < 3; j++)
		for (i = 0; i < num - 1; i++)
			if (Min_pts[j] > Pts[i + 1][j])
				Min_pts[j] = Pts[i + 1][j];
	//计算XYZ方向，并取最大者
	XYZ_Scale[0] = double((fabs(Min_pts[0]) + fabs(Max_pts[0])) / 2.0);
	XYZ_Scale[1] = double((fabs(Min_pts[1]) + fabs(Max_pts[1])) / 2.0);
	XYZ_Scale[2] = double((fabs(Min_pts[2]) + fabs(Max_pts[2])) / 2.0);
	Scale = XYZ_Scale[0];
	for (i = 1; i < 3; i++)
		if (Scale < XYZ_Scale[i])
			Scale = XYZ_Scale[i];
	cout << "X方向跨度为" << XYZ_Scale[0] << ":1" << endl;
	cout << "Y方向跨度为" << XYZ_Scale[1] << ":1" << endl;
	cout << "Z方向跨度为" << XYZ_Scale[2] << ":1" << endl;
	cout << "最大跨度为" << Scale << endl;
	return (Scale);
}

///////////////////////////////////////////////////////////////////////////////
//估算点云密度
inline float CJsGrRead::Dens_PtCloud(void)
{
	cout << "点云密度计算，大约需要5~10秒的时间，请稍候..." << endl; //打印提示
	N = int(num / 100);
	M = int(N / 4);
	//从点云中随机取出N个点作为样本点集
	int(*numlist);                    //存放符合条件的随机数的数组
	numlist = new int[N];                //初始化
	S = new float[N][3];

	int i = 0, j = 0;
	int randnum = 0;

	srand((unsigned)time(NULL));               ////随机数种子,time()作种子
	while (i < N)
	{
		randnum = rand() % num;                    //生成一个随机数
		while (InNumList(randnum, numlist, i))   //判断是否有重复
			randnum = rand() % num;                //若有重复，则返回TRUE，继续生成
		numlist[i] = randnum;
		i++;
	}
	// for(j=0;j<N;j++)
	// 	cout<<numlist[j]<<endl;               //test01：输出随机数              
	for (i = 0; i < N; i++)
		for (j = 0; j < 3; j++)
			S[i][j] = Pts[numlist[i]][j];         //赋值给S数组

	//对于S中任一点P在点云中找与其距离最近的M个点
	float init_v;                   //测试以P为中心，
									//init_v为1/2边长的立方体中有多少个点?
	int num_p;                      //立方体内点的数量
	float min_cube[3];              //立方体x,y,z方向上的最小值
	float max_cube[3];              //立方体x,y,z方向上的最大值
	float(*In_Pts)[3];             //存放立方体内点的坐标
	float(*D); //存放距离的数组 
	float NUM = num;

	const float value = (Scale*2.0) / sqrt(NUM);
	dens = 0;
	for (i = 0; i < N; i++)
	{
		//测试init_v为初始值时，立方体内点的数量
		init_v = value;
		num_p = 0;
		for (int k = 0; k < 3; k++)
		{
			min_cube[k] = S[i][k] - init_v;
			max_cube[k] = S[i][k] + init_v;
		}
		for (j = 0; j < num; j++)
		{
			if ((Pts[j][0] > min_cube[0]) &&
				(Pts[j][0] < max_cube[0]) &&
				(Pts[j][1] > min_cube[1]) &&
				(Pts[j][1] < max_cube[1]) &&
				(Pts[j][2] > min_cube[2]) &&
				(Pts[j][2] < max_cube[2]))
				num_p++;
		}
		//如果不满足条件：num_p>=M+1，将init_v加上步长，继续测试
		while (num_p < M + 1)
		{
			init_v += 0.25*init_v;          //步长为0.25*init_v

			for (int k = 0; k < 3; k++)
			{
				min_cube[k] = S[i][k] - init_v;
				max_cube[k] = S[i][k] + init_v;
			}

			num_p = 0;                      //重新置为0
			for (j = 0; j < num; j++)
			{
				if ((Pts[j][0] > min_cube[0]) &&
					(Pts[j][0] < max_cube[0]) &&
					(Pts[j][1] > min_cube[1]) &&
					(Pts[j][1] < max_cube[1]) &&
					(Pts[j][2] > min_cube[2]) &&
					(Pts[j][2] < max_cube[2]))
					num_p++;
			}
		}
		In_Pts = new float[num_p][3];
		num_p = 0;
		for (j = 0; j < num; j++)
		{
			if ((Pts[j][0] > min_cube[0]) &&
				(Pts[j][0] < max_cube[0]) &&
				(Pts[j][1] > min_cube[1]) &&
				(Pts[j][1] < max_cube[1]) &&
				(Pts[j][2] > min_cube[2]) &&
				(Pts[j][2] < max_cube[2]))
			{
				for (int k = 0; k < 3; k++)
					In_Pts[num_p][k] = Pts[j][k];
				num_p++;
			}
		}

		//计算立方体内每一点到P点的距离，求最近的M个点的距离平均值
		D = new float[num_p];
		for (j = 0; j < num_p; j++)
			D[j] = Distance(&S[i][0], &In_Pts[j][0]);

		//冒泡法排序
		for (j = 0; j < num_p - 1; j++)
			for (int k = 0; k < num_p - 1 - j; k++)
				if (D[k] > D[k + 1])
				{
					float t = D[k];
					D[k] = D[k + 1];
					D[k + 1] = t;
				}
		// 		cout<<"D:"<<D[0]<<endl;              //测试最小值是否为0

		for (j = 1; j < M + 1; j++)
			dens += float(D[j] / M);
	}

	dens /= N;
	cout << "点云密度为：" << dens << endl;
	cout << endl;
	return(dens);
}

inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CJsGrRead::cloudtransformed(void)
{
	/*
	cloud2->width = num;
	cloud2->height = 1;
	cloud2->points.resize(cloud2->width * cloud2->height);
	
	for (int i = 0; i < num; i++)
	{
		cloud2->points[i].x = Pts[i][0];
		cloud2->points[i].y = Pts[i][1];
		cloud2->points[i].z = Pts[i][2];
		
	}
	*/
	//pcl::io::savePCDFileASCII("cloudout.pcd", *output);
	return(output);
}

#endif //__LMZ_READ_POINTS_H__

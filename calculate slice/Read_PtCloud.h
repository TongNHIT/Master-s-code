#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//����һ����ȡ�ࣺ��ȡtxt�ļ��еĵ�����(���Ƶ㰴��ʱ������)
//                ������ƶ���ε�͹��
//                ��������ģ�͵����ű���
//                ��������ܶ�
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
//��ȡ��
class CJsGrRead
{
private:
	char filename[20];                 //����ļ���������
	float Center[3];                   //���е������
public:
	int num;                           //txt�ļ��е������
	float(*Pts)[3];                   //������ݵ��ָ������ 
	float XYZ_Scale[3];                //X,Y,Z��������ű���
	float Max_pts[3];                  //�������ֵ����(���ҡ����ϡ���ǰ)
	float Min_pts[3];                  //��С����ֵ����(�������¡����)
	float Scale;                       //���ű���
	int N;                             //N������������
	int M;                             //M�������г�ȡ����������
	float(*S)[3];	                   //�������㼯������
	float dens;                        //���ƵĹ����ܶ�
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::PointCloud<pcl::PointXYZ>::Ptr output{ new pcl::PointCloud<pcl::PointXYZ> };
	inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudin(char *filename);
	inline Eigen::Matrix4f trans(void);
	inline float Scale_PtCloud(void);  //�������ģ�͵����ű���
	inline float Dens_PtCloud(void);   //��������ܶ�
	inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudtransformed(void);         //�������
};
CJsGrRead file1;

///////////////////////////////////////////////////////////////////////////////
//����*.PCD�ļ��е����������ȡ���Ƶ�����(���Ƶ㰴��ʱ������)
inline pcl::PointCloud<pcl::PointXYZ>::Ptr CJsGrRead::cloudin(char *filename)
{
	num = 0;
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>(filename, *cloud1);
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud1, pcaCentroid);//��������
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud1, pcaCentroid, covariance);//����3*3��Э�������
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);//�����ֽ�
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//Э����������������
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//Э������������ֵ
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //У��������䴹ֱ
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	std::cout << "����ֵva(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "��������ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "���ĵ�(4x1):\n" << pcaCentroid << std::endl;
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	//Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	std::cout << "�任����tm(4x4):\n" << tm << std::endl;
	std::cout << "������tm'(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud1, *transformedCloud, tm);//�����Ʊ任��ԭ�㣬������������ϵ�����غ�

	vector<int> indices;
	pcl::removeNaNFromPointCloud(*transformedCloud, *output, indices);
	//��ȡ�ļ��еĵ�����
	num = output->width*output->height;
	cout << filename << "ģ�͹���" << num << "���㡣" << endl;

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
//�������ģ�͵ĸ�������
inline float CJsGrRead::Scale_PtCloud(void)
{
	int i, j;
	for (i = 0; i < 3; i++)
	{
		Max_pts[i] = Pts[0][i];
		Min_pts[i] = Pts[0][i];
	}
	//���������ֵ(���ҡ����ϡ���ǰ)
	for (j = 0; j < 3; j++)
		for (i = 0; i < num - 1; i++)
			if (Max_pts[j] < Pts[i + 1][j])
				Max_pts[j] = Pts[i + 1][j];
	//����С����ֵ(�������¡����)
	for (j = 0; j < 3; j++)
		for (i = 0; i < num - 1; i++)
			if (Min_pts[j] > Pts[i + 1][j])
				Min_pts[j] = Pts[i + 1][j];
	//����XYZ���򣬲�ȡ�����
	XYZ_Scale[0] = double((fabs(Min_pts[0]) + fabs(Max_pts[0])) / 2.0);
	XYZ_Scale[1] = double((fabs(Min_pts[1]) + fabs(Max_pts[1])) / 2.0);
	XYZ_Scale[2] = double((fabs(Min_pts[2]) + fabs(Max_pts[2])) / 2.0);
	Scale = XYZ_Scale[0];
	for (i = 1; i < 3; i++)
		if (Scale < XYZ_Scale[i])
			Scale = XYZ_Scale[i];
	cout << "X������Ϊ" << XYZ_Scale[0] << ":1" << endl;
	cout << "Y������Ϊ" << XYZ_Scale[1] << ":1" << endl;
	cout << "Z������Ϊ" << XYZ_Scale[2] << ":1" << endl;
	cout << "�����Ϊ" << Scale << endl;
	return (Scale);
}

///////////////////////////////////////////////////////////////////////////////
//��������ܶ�
inline float CJsGrRead::Dens_PtCloud(void)
{
	cout << "�����ܶȼ��㣬��Լ��Ҫ5~10���ʱ�䣬���Ժ�..." << endl; //��ӡ��ʾ
	N = int(num / 100);
	M = int(N / 4);
	//�ӵ��������ȡ��N������Ϊ�����㼯
	int(*numlist);                    //��ŷ��������������������
	numlist = new int[N];                //��ʼ��
	S = new float[N][3];

	int i = 0, j = 0;
	int randnum = 0;

	srand((unsigned)time(NULL));               ////���������,time()������
	while (i < N)
	{
		randnum = rand() % num;                    //����һ�������
		while (InNumList(randnum, numlist, i))   //�ж��Ƿ����ظ�
			randnum = rand() % num;                //�����ظ����򷵻�TRUE����������
		numlist[i] = randnum;
		i++;
	}
	// for(j=0;j<N;j++)
	// 	cout<<numlist[j]<<endl;               //test01����������              
	for (i = 0; i < N; i++)
		for (j = 0; j < 3; j++)
			S[i][j] = Pts[numlist[i]][j];         //��ֵ��S����

	//����S����һ��P�ڵ�������������������M����
	float init_v;                   //������PΪ���ģ�
									//init_vΪ1/2�߳������������ж��ٸ���?
	int num_p;                      //�������ڵ������
	float min_cube[3];              //������x,y,z�����ϵ���Сֵ
	float max_cube[3];              //������x,y,z�����ϵ����ֵ
	float(*In_Pts)[3];             //����������ڵ������
	float(*D); //��ž�������� 
	float NUM = num;

	const float value = (Scale*2.0) / sqrt(NUM);
	dens = 0;
	for (i = 0; i < N; i++)
	{
		//����init_vΪ��ʼֵʱ���������ڵ������
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
		//���������������num_p>=M+1����init_v���ϲ�������������
		while (num_p < M + 1)
		{
			init_v += 0.25*init_v;          //����Ϊ0.25*init_v

			for (int k = 0; k < 3; k++)
			{
				min_cube[k] = S[i][k] - init_v;
				max_cube[k] = S[i][k] + init_v;
			}

			num_p = 0;                      //������Ϊ0
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

		//������������ÿһ�㵽P��ľ��룬�������M����ľ���ƽ��ֵ
		D = new float[num_p];
		for (j = 0; j < num_p; j++)
			D[j] = Distance(&S[i][0], &In_Pts[j][0]);

		//ð�ݷ�����
		for (j = 0; j < num_p - 1; j++)
			for (int k = 0; k < num_p - 1 - j; k++)
				if (D[k] > D[k + 1])
				{
					float t = D[k];
					D[k] = D[k + 1];
					D[k + 1] = t;
				}
		// 		cout<<"D:"<<D[0]<<endl;              //������Сֵ�Ƿ�Ϊ0

		for (j = 1; j < M + 1; j++)
			dens += float(D[j] / M);
	}

	dens /= N;
	cout << "�����ܶ�Ϊ��" << dens << endl;
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

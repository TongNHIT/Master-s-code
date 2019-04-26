#pragma once
///////////////////////////////////////////////////////////////////////////////
//定义程序中共用的函数
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_FUNCTION_H__
#define __LMZ_FUNCTION_H__
#include "global.h"
#include "linkedListType.h"
#include <math.h>

#define PI 3.1415926

///////////////////////////////////////////////////////////////////////////////
//定义计算距离的函数：数组
/*
	  p1：存放第一点坐标信息的数组
	  p2：存放第二点坐标信息的数组
*/
float Distance(float *p1, float *p2)
{
	float Dist;
	Dist = sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) +
		(p1[1] - p2[1])*(p1[1] - p2[1]) +
		(p1[2] - p2[2])*(p1[2] - p2[2]));
	return (Dist);
}

//定义计算距离的函数：链表
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
*/
float Dist_List(FPoint p1, FPoint p2)
{
	double Dist;
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	float dz = p1.z - p2.z;
	Dist = sqrt(dx*dx + dy * dy + dz * dz);
	return (Dist);
}

///////////////////////////////////////////////////////////////////////////////
//判断随机数是否重复的函数
/*
	  n：新生成的数字
	  srcNumList：存放数字（不重复）的数组
	  nNums：当前共有多少个数字在srcNumList中
*/
bool InNumList(int n, int* srcNumList, int nNums)
{
	for (int j = 0; j < nNums; j++)
		if (n == srcNumList[j])
			return true;
	return false;
}

///////////////////////////////////////////////////////////////////////////////
//搜索各点，比较ds、de取较小值D，当D为最小值时得到最近距离点，插入前或尾节点
/*
	  startpt：起点的指针
	  endpt：终点的指针
	  pts：点集链表的头指针
*/
nodeType *researchpt(nodeType* startpt, nodeType* endpt, nodeType *pts)
{
	nodeType *currentpt = pts;
	float ds, de, D, minD;      //ds：当前点与起点的距离    de：当前点与终点的距离
							 //D：取ds与de的较小值       minD：所有的D的最小值
	nodeType *minpt = nullptr;         //最近距离点的指针
	int flag, minflag;

	while (currentpt != NULL)
	{
		//判断ptID是否为0：若不为0，搜索下一个；若为0，取当前点
		if (currentpt->CBasicPoint.ptID == 0)
		{
			ds = Dist_List(startpt->CBasicPoint, currentpt->CBasicPoint);
			de = Dist_List(endpt->CBasicPoint, currentpt->CBasicPoint);
			if (ds < de)
			{
				D = ds;
				flag = 1;      //前节点
			}
			else
			{
				D = de;
				flag = -1;     //后节点
			}
			minD = D;
			minflag = flag;
			minpt = currentpt;
			break;           //搜索第一个符合条件的点，将其距离赋值给minD
		}
		currentpt = currentpt->link;
	}

	//计算最近距离点
	while (currentpt != NULL)
	{
		if (currentpt->CBasicPoint.ptID == 0)
		{
			ds = Dist_List(startpt->CBasicPoint, currentpt->CBasicPoint);
			de = Dist_List(endpt->CBasicPoint, currentpt->CBasicPoint);
			if (ds < de)
			{
				D = ds;
				flag = 1;      //前节点
			}
			else
			{
				D = de;
				flag = -1;     //后节点
			}
			if (minD > D)
			{
				minD = D;
				minflag = flag;
				minpt = currentpt;
			}
		}
		currentpt = currentpt->link;
	}

	minpt->CBasicPoint.ptID = minflag;
	return (minpt);
}

///////////////////////////////////////////////////////////////////////////////
//定义计算点到直线距离的函数（海伦公式）
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
	  p3：存放第三点坐标信息的结构体
*/
float Dist_pt_line(FPoint p1, FPoint p2, FPoint p3)
{
	float a, b, c;             //分别为三角形三条边的边长
	double S;                //三角形的面积，面积计算采用海伦公式
	float Dist;
	a = Dist_List(p1, p2);                //调用Dist_List函数
	b = Dist_List(p2, p3);
	c = Dist_List(p1, p3);
	float s = (a + b + c) / 2;
	S = sqrt(s*(s - a)*(s - b)*(s - c));
	if (c == 0)
		Dist = 0;
	else
		Dist = 2 * S / c;
	return (Dist);
}

///////////////////////////////////////////////////////////////////////////////
//定义夹角计算函数（余弦定理）
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
	  p3：存放第三点坐标信息的结构体
*/
float Cos_Angle(FPoint p1, FPoint p2, FPoint p3)
{
	float a, b, c;             //分别为三角形三条边的边长
	float cosA;

	a = Dist_List(p1, p2);                //调用Dist_List函数
	b = Dist_List(p2, p3);
	c = Dist_List(p1, p3);
	if (a == 0 || b == 0)
		cosA = -1;
	else
		cosA = (a*a + b * b - c * c) / (2 * a*b);
	return (cosA);
}

///////////////////////////////////////////////////////////////////////////////
//定义两相邻向量叉乘的函数(Cross Product)
/*
	  vector1：向量1
	  vector2：向量2
	  normal：存放法向量的数组
*/
void Cal_vNormal(float vector1[3], float vector2[3], float normal[3])
{
	normal[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
	normal[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
	normal[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];

	float length = sqrt(normal[0] * normal[0] +
		normal[1] * normal[1] +
		normal[2] * normal[2]);
	if (length != 0.0)
	{
		normal[0] /= length;
		normal[1] /= length;
		normal[2] /= length;
	}
	else
	{
		normal[0] = 0.0;
		normal[1] = 0.0;
		normal[2] = 0.0;
	}
}

//定义相邻三点构成两向量叉乘的函数
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
	  p3：存放第三点坐标信息的结构体
	  normal：存放法向量的数组
*/
void Cal_pNormal(FPoint p1, FPoint p2, FPoint p3, float normal[3])
{
	float vector1[3], vector2[3];

	vector1[0] = p2.x - p1.x;
	vector1[1] = p2.y - p1.y;
	vector1[2] = p2.z - p1.z;
	vector2[0] = p3.x - p1.x;
	vector2[1] = p3.y - p1.y;
	vector2[2] = p3.z - p1.z;

	Cal_vNormal(&vector1[0], &vector2[0], &normal[0]);
	//调用Cal_vNormal函数
}

//由相邻矢量的叉乘判定多义线方向
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
	  p3：存放第三点坐标信息的结构体
	  flag：切片方向的标记
*/
int Cal_Direction(FPoint p1, FPoint p2, FPoint p3, int flag)
{
	float normal[3];
	int nFlag = 0;                       //初始化nFlag

	Cal_pNormal(p1, p2, p3, &normal[0]);   //调用Cal_pNormal函数

	switch (flag)
	{
	case 0:
		if (normal[0] == 1)
			nFlag = 1;
		else if (normal[0] == -1)
			nFlag = -1;
		break;
	case 1:
		if (normal[1] == 1)
			nFlag = 1;
		else if (normal[1] == -1)
			nFlag = -1;
		break;
	case 2:
		if (normal[2] == 1)
			nFlag = 1;
		else if (normal[2] == -1)
			nFlag = -1;
		break;
	}
	return (nFlag);
}

///////////////////////////////////////////////////////////////////////////////
//定义计算离散曲率的函数：中间点
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
	  p3：存放第三点坐标信息的结构体
	  flag：切片方向的标记
*/
float Cal_Curvature(FPoint p1, FPoint p2, FPoint p3, int flag)
{
	float a, b, c;             //分别为三角形三条边的边长
	double S;                //三角形的面积，面积计算采用海伦公式
	float curv;              //曲率
	int sign;                //符号的标记

	//计算三角形面积
	a = Dist_List(p1, p2);                //调用Dist_List函数
	b = Dist_List(p2, p3);
	c = Dist_List(p1, p3);
	float s = (a + b + c) / 2;
	S = sqrt(s*(s - a)*(s - b)*(s - c));

	//计算符号
	sign = Cal_Direction(p1, p2, p3, flag); //调用Cal_Direction函数
	curv = sign * 4 * S / (a*b*c);

	return (curv);
}

///////////////////////////////////////////////////////////////////////////////
//定义计算切矢的函数：首末端点
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
	  p3：存放第三点坐标信息的结构体
	  flag：切片方向的标记
*/
void Cal_Tangent(FPoint p1, FPoint p2, FPoint p3, int flag, float tang[3])
{
	float a, b, c, d, e, f, g;
	FPoint p0;
	float r[3], n[3];

	switch (flag)
	{
	case 0:
		a = (p1.y + p2.y)*(p2.y - p1.y)*(p3.z - p2.z);
		b = (p2.y + p3.y)*(p3.y - p2.y)*(p2.z - p1.z);
		c = (p1.z - p3.z)*(p2.z - p1.z)*(p3.z - p2.z);
		d = 2 * ((p2.y - p1.y)*(p3.z - p2.z) - (p3.y - p2.y)*(p2.z - p1.z));
		e = (p1.z + p2.z)*(p2.z - p1.z)*(p3.y - p2.y);
		f = (p2.z + p3.z)*(p3.z - p2.z)*(p2.y - p1.y);
		g = (p1.y - p3.y)*(p2.y - p1.y)*(p3.y - p2.y);
		p0.x = p1.x;
		p0.y = (a - b + c) / d;
		p0.z = (f - e - g) / d;
		r[0] = p0.x - p1.x;
		r[1] = p0.y - p1.y;
		r[2] = p0.z - p1.z;

		Cal_pNormal(p1, p3, p2, &n[0]);
		Cal_vNormal(&r[0], &n[0], &tang[0]);

		//cout<<tang[0]<<" "<<tang[1]<<" "<<tang[2]<<endl;
		break;
	case 1:
		cout << "待编程……" << endl;
		break;
	case 2:
		cout << "待编程……" << endl;
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////
//基于射线的匹配方法
/*
	  p1：存放第一点坐标信息的结构体
	  p2：存放第二点坐标信息的结构体
	  p3：存放第三点坐标信息的结构体
*/
float Ray_Firing(FPoint p1, FPoint p2, FPoint p3)
{
	float cosA, angle;

	cosA = Cos_Angle(p1, p2, p3);          //调用Cos_Angle函数
	angle = acos(cosA);
	return (angle);
}

#endif //__LMZ_FUNCTION_H__
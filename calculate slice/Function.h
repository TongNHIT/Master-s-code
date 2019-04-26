#pragma once
///////////////////////////////////////////////////////////////////////////////
//��������й��õĺ���
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_FUNCTION_H__
#define __LMZ_FUNCTION_H__
#include "global.h"
#include "linkedListType.h"
#include <math.h>

#define PI 3.1415926

///////////////////////////////////////////////////////////////////////////////
//����������ĺ���������
/*
	  p1����ŵ�һ��������Ϣ������
	  p2����ŵڶ���������Ϣ������
*/
float Distance(float *p1, float *p2)
{
	float Dist;
	Dist = sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) +
		(p1[1] - p2[1])*(p1[1] - p2[1]) +
		(p1[2] - p2[2])*(p1[2] - p2[2]));
	return (Dist);
}

//����������ĺ���������
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
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
//�ж�������Ƿ��ظ��ĺ���
/*
	  n�������ɵ�����
	  srcNumList��������֣����ظ���������
	  nNums����ǰ���ж��ٸ�������srcNumList��
*/
bool InNumList(int n, int* srcNumList, int nNums)
{
	for (int j = 0; j < nNums; j++)
		if (n == srcNumList[j])
			return true;
	return false;
}

///////////////////////////////////////////////////////////////////////////////
//�������㣬�Ƚ�ds��deȡ��СֵD����DΪ��Сֵʱ�õ��������㣬����ǰ��β�ڵ�
/*
	  startpt������ָ��
	  endpt���յ��ָ��
	  pts���㼯�����ͷָ��
*/
nodeType *researchpt(nodeType* startpt, nodeType* endpt, nodeType *pts)
{
	nodeType *currentpt = pts;
	float ds, de, D, minD;      //ds����ǰ�������ľ���    de����ǰ�����յ�ľ���
							 //D��ȡds��de�Ľ�Сֵ       minD�����е�D����Сֵ
	nodeType *minpt = nullptr;         //���������ָ��
	int flag, minflag;

	while (currentpt != NULL)
	{
		//�ж�ptID�Ƿ�Ϊ0������Ϊ0��������һ������Ϊ0��ȡ��ǰ��
		if (currentpt->CBasicPoint.ptID == 0)
		{
			ds = Dist_List(startpt->CBasicPoint, currentpt->CBasicPoint);
			de = Dist_List(endpt->CBasicPoint, currentpt->CBasicPoint);
			if (ds < de)
			{
				D = ds;
				flag = 1;      //ǰ�ڵ�
			}
			else
			{
				D = de;
				flag = -1;     //��ڵ�
			}
			minD = D;
			minflag = flag;
			minpt = currentpt;
			break;           //������һ�����������ĵ㣬������븳ֵ��minD
		}
		currentpt = currentpt->link;
	}

	//������������
	while (currentpt != NULL)
	{
		if (currentpt->CBasicPoint.ptID == 0)
		{
			ds = Dist_List(startpt->CBasicPoint, currentpt->CBasicPoint);
			de = Dist_List(endpt->CBasicPoint, currentpt->CBasicPoint);
			if (ds < de)
			{
				D = ds;
				flag = 1;      //ǰ�ڵ�
			}
			else
			{
				D = de;
				flag = -1;     //��ڵ�
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
//�������㵽ֱ�߾���ĺ��������׹�ʽ��
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
	  p3����ŵ�����������Ϣ�Ľṹ��
*/
float Dist_pt_line(FPoint p1, FPoint p2, FPoint p3)
{
	float a, b, c;             //�ֱ�Ϊ�����������ߵı߳�
	double S;                //�����ε���������������ú��׹�ʽ
	float Dist;
	a = Dist_List(p1, p2);                //����Dist_List����
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
//����нǼ��㺯�������Ҷ���
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
	  p3����ŵ�����������Ϣ�Ľṹ��
*/
float Cos_Angle(FPoint p1, FPoint p2, FPoint p3)
{
	float a, b, c;             //�ֱ�Ϊ�����������ߵı߳�
	float cosA;

	a = Dist_List(p1, p2);                //����Dist_List����
	b = Dist_List(p2, p3);
	c = Dist_List(p1, p3);
	if (a == 0 || b == 0)
		cosA = -1;
	else
		cosA = (a*a + b * b - c * c) / (2 * a*b);
	return (cosA);
}

///////////////////////////////////////////////////////////////////////////////
//����������������˵ĺ���(Cross Product)
/*
	  vector1������1
	  vector2������2
	  normal����ŷ�����������
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

//�����������㹹����������˵ĺ���
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
	  p3����ŵ�����������Ϣ�Ľṹ��
	  normal����ŷ�����������
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
	//����Cal_vNormal����
}

//������ʸ���Ĳ���ж������߷���
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
	  p3����ŵ�����������Ϣ�Ľṹ��
	  flag����Ƭ����ı��
*/
int Cal_Direction(FPoint p1, FPoint p2, FPoint p3, int flag)
{
	float normal[3];
	int nFlag = 0;                       //��ʼ��nFlag

	Cal_pNormal(p1, p2, p3, &normal[0]);   //����Cal_pNormal����

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
//���������ɢ���ʵĺ������м��
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
	  p3����ŵ�����������Ϣ�Ľṹ��
	  flag����Ƭ����ı��
*/
float Cal_Curvature(FPoint p1, FPoint p2, FPoint p3, int flag)
{
	float a, b, c;             //�ֱ�Ϊ�����������ߵı߳�
	double S;                //�����ε���������������ú��׹�ʽ
	float curv;              //����
	int sign;                //���ŵı��

	//�������������
	a = Dist_List(p1, p2);                //����Dist_List����
	b = Dist_List(p2, p3);
	c = Dist_List(p1, p3);
	float s = (a + b + c) / 2;
	S = sqrt(s*(s - a)*(s - b)*(s - c));

	//�������
	sign = Cal_Direction(p1, p2, p3, flag); //����Cal_Direction����
	curv = sign * 4 * S / (a*b*c);

	return (curv);
}

///////////////////////////////////////////////////////////////////////////////
//���������ʸ�ĺ�������ĩ�˵�
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
	  p3����ŵ�����������Ϣ�Ľṹ��
	  flag����Ƭ����ı��
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
		cout << "����̡���" << endl;
		break;
	case 2:
		cout << "����̡���" << endl;
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////
//�������ߵ�ƥ�䷽��
/*
	  p1����ŵ�һ��������Ϣ�Ľṹ��
	  p2����ŵڶ���������Ϣ�Ľṹ��
	  p3����ŵ�����������Ϣ�Ľṹ��
*/
float Ray_Firing(FPoint p1, FPoint p2, FPoint p3)
{
	float cosA, angle;

	cosA = Cos_Angle(p1, p2, p3);          //����Cos_Angle����
	angle = acos(cosA);
	return (angle);
}

#endif //__LMZ_FUNCTION_H__
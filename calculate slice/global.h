#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//����ṹ��
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_STRUCT_H__
#define __LMZ_STRUCT_H__

///////////////////////////////////////////////////////////////////////////////
struct FPoint
{
	int ptID;                          //��Ƭ���
	int ptID_s;                        //ÿ����Ƭ�Ľڵ���
	float x;                           //x����
	float y;                           //y����
	float z;                           //z����
	float curv;                        //����ֵ
	int normal;                        //��λ����������Ƭ�����ֵ
};

struct nodeType
{
	struct FPoint CBasicPoint;         //�����еĽڵ�
	struct nodeType *link;             //ָ����һ���ڵ�
	struct nodeType *head;             //ָ����һ���ڵ�
};


#endif//__LMZ_STRUCT_H__
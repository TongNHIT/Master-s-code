#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//定义结构体
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_STRUCT_H__
#define __LMZ_STRUCT_H__

///////////////////////////////////////////////////////////////////////////////
struct FPoint
{
	int ptID;                          //切片编号
	int ptID_s;                        //每层切片的节点编号
	float x;                           //x坐标
	float y;                           //y坐标
	float z;                           //z坐标
	float curv;                        //曲率值
	int normal;                        //单位法向量在切片方向的值
};

struct nodeType
{
	struct FPoint CBasicPoint;         //链表中的节点
	struct nodeType *link;             //指向下一个节点
	struct nodeType *head;             //指向上一个节点
};


#endif//__LMZ_STRUCT_H__
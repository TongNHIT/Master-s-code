#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//定义一个切片类：切片方向(自定义)
//                切片厚度(点云密度估算)
//                切片曲线的构造(最近点搜索)
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_CONTOUR_H__
#define __LMZ_CONTOUR_H__
#include <iostream>
#include <math.h>
#include <iomanip>
#include "Read_PtCloud.h"
#include "Preprocessing.h"
#include "linkedListType.h"
#include "Function.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
/////////////////////////////////////////////////////////////////////////////// 
//切片类
class CJsGrSlicing
{
private:
	float Min_pFlag;                   //切片方向上的最小值
	linkedListType E, El, Er, El_corr, Er_corr;
	//定义链表类的对象
	linkedListType Extreme;            //同上，存储极值点

public:
	float tkness;                      //切片厚度
	int flag;                          //切片方向的标记
	linkedListType Ptsio;              //定义链表类的对象，存储有序切片数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3{ new pcl::PointCloud<pcl::PointXYZ> };
	inline void Tkness(float Scale);   //切片厚度计算
	inline void Direct(float Scale, float *XYZ_Scale);//切片方向定义

	inline void InterS(int num, float Pts[][3], int flag, float tkness);//平面与点云求交

	inline void Curves(int flag, float dens);//切片多义线构建

	inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudout(void);         //显示函数

};
CJsGrSlicing slice1, slice2;            //定义点云切片类的对象


///////////////////////////////////////////////////////////////////////////////
//切片厚度计算-步长
inline void CJsGrSlicing::Tkness(float Scale)
{

	tkness = (2.0*Scale) / ppro.FinalSect;
	cout << endl;
	cout << "切片厚度为：" << tkness << endl;
}

///////////////////////////////////////////////////////////////////////////////
//切片方向：暂时以X,Y,Z方向的最大跨度的轴向作为切片方向
inline void  CJsGrSlicing::Direct(float Scale, float *XYZ_Scale)
{
	for (int i = 0; i < 3; i++)
		if (XYZ_Scale[i] == Scale)
		{
			flag = i;
			break;
		}

	switch (flag)
	{
	case 0:
		cout << "切片方向为：X 轴" << endl;
		break;
	case 1:
		cout << "切片方向为：Y 轴" << endl;
		break;
	case 2:
		cout << "切片方向为：Z 轴" << endl;
		break;
	}

}

///////////////////////////////////////////////////////////////////////////////
//平面与点云求交
inline void CJsGrSlicing::InterS(int num, float Pts[][3],
	int flag, float tkness)
{
	cout << "点云切片计算，大约需要4~8秒的时间，请稍候..." << endl;
	//打印提示
//点云切片计算，构建编号为0~ppro.FinalSect-1的切片数据
	E.initList();                      //初始化
	float ss = 0;                        //切片步长step of slicing
	int i, j;

	//计算切片方向的最小值
	Min_pFlag = Pts[0][flag];
	for (i = 0; i < num - 1; i++)
		if (Min_pFlag > Pts[i + 1][flag])
			Min_pFlag = Pts[i + 1][flag];

	FPoint ptNode;
	j = 0;                               //从编号为0的切片开始
	do
	{
		//链表初始化
		El.initList();
		Er.initList();
		El_corr.initList();
		Er_corr.initList();

		//点云数据分层，ptID的值为切片的编号
		for (i = 0; i < num; i++)
		{
			//将位于切片平面左边的点集存储到El链表中
			if (Pts[i][flag] >= (Min_pFlag + ss) &&
				Pts[i][flag] < (Min_pFlag + ss + 0.5*tkness))
			{
				ptNode.ptID = j;
				ptNode.x = Pts[i][0];
				ptNode.y = Pts[i][1];
				ptNode.z = Pts[i][2];
				El.insertNode(ptNode);
			}
			//将位于切片平面的点集存储到E链表中
			else if (Pts[i][flag] == (Min_pFlag + ss + 0.5*tkness))
			{
				ptNode.ptID = j;
				ptNode.x = Pts[i][0];
				ptNode.y = Pts[i][1];
				ptNode.z = Pts[i][2];
				E.insertNode(ptNode);
			}
			//将位于切片平面右边的点集存储到Er链表中
			else if (Pts[i][flag] > (Min_pFlag + ss + 0.5*tkness) &&
				Pts[i][flag] < (Min_pFlag + ss + tkness))
			{
				ptNode.ptID = j;
				ptNode.x = Pts[i][0];
				ptNode.y = Pts[i][1];
				ptNode.z = Pts[i][2];
				Er.insertNode(ptNode);
			}
		}

		//对El中的每一点，求其在Er中的最近点，反过来再求其在El中的最近点
		nodeType *curr_l = El.front();
		nodeType *curr_lt = El.front();  //用以测试两点是否为匹配对的指针
		nodeType *curr_r = Er.front();

		while (curr_l != NULL)
		{
			//El中的点，求在Er中的最近点
			//------------------------>Start here!
						//计算Er中位于空间立方体内的点的数量
			float sc = 0;                //立方体步长step of cube
			int num_in = 0;
			while (num_in == 0)
			{
				sc += 0.5*tkness;        //设置步长为0.5*tkness
				curr_r = Er.front();
				while (curr_r != NULL)
				{
					if (curr_r->CBasicPoint.x > curr_l->CBasicPoint.x - sc &&
						curr_r->CBasicPoint.x < curr_l->CBasicPoint.x + sc &&
						curr_r->CBasicPoint.y > curr_l->CBasicPoint.y - sc &&
						curr_r->CBasicPoint.y < curr_l->CBasicPoint.y + sc &&
						curr_r->CBasicPoint.z > curr_l->CBasicPoint.z - sc &&
						curr_r->CBasicPoint.z < curr_l->CBasicPoint.z + sc)
						num_in++;
					curr_r = curr_r->link;
				}//end while
			}//end while

			//计算立方体内的最近点
			float minD, D;                //最小距离值，距离值
			FPoint ptNode_r;             //存储Er内最近点的信息

			curr_r = Er.front();
			while (curr_r != NULL)
			{
				if (curr_r->CBasicPoint.x > curr_l->CBasicPoint.x - sc &&
					curr_r->CBasicPoint.x < curr_l->CBasicPoint.x + sc &&
					curr_r->CBasicPoint.y > curr_l->CBasicPoint.y - sc &&
					curr_r->CBasicPoint.y < curr_l->CBasicPoint.y + sc &&
					curr_r->CBasicPoint.z > curr_l->CBasicPoint.z - sc &&
					curr_r->CBasicPoint.z < curr_l->CBasicPoint.z + sc)
				{
					minD = Dist_List(curr_r->CBasicPoint, curr_l->CBasicPoint);
					ptNode_r.ptID = curr_r->CBasicPoint.ptID;
					ptNode_r.x = curr_r->CBasicPoint.x;
					ptNode_r.y = curr_r->CBasicPoint.y;
					ptNode_r.z = curr_r->CBasicPoint.z;
					break;             //将第一个符合条件的点赋给minD，ptNode_r
				}//end if
				curr_r = curr_r->link;
			}//end while

			curr_r = Er.front();
			while (curr_r != NULL)
			{
				if (curr_r->CBasicPoint.x > curr_l->CBasicPoint.x - sc &&
					curr_r->CBasicPoint.x < curr_l->CBasicPoint.x + sc &&
					curr_r->CBasicPoint.y > curr_l->CBasicPoint.y - sc &&
					curr_r->CBasicPoint.y < curr_l->CBasicPoint.y + sc &&
					curr_r->CBasicPoint.z > curr_l->CBasicPoint.z - sc &&
					curr_r->CBasicPoint.z < curr_l->CBasicPoint.z + sc)
				{
					D = Dist_List(curr_r->CBasicPoint, curr_l->CBasicPoint);
					if (minD > D)
					{
						//最近点的信息
						minD = D;
						ptNode_r.ptID = curr_r->CBasicPoint.ptID;
						ptNode_r.x = curr_r->CBasicPoint.x;
						ptNode_r.y = curr_r->CBasicPoint.y;
						ptNode_r.z = curr_r->CBasicPoint.z;
					}//end if
				}//end if
				curr_r = curr_r->link;
			}//end while
//<------------------------End here!

//反过来再求其在El中的最近点
//------------------------>Start here!
			sc = 0;
			num_in = 0;
			while (num_in == 0)
			{
				sc += 0.5*tkness;        //设置步长为0.5*tkness
				curr_lt = El.front();
				while (curr_lt != NULL)
				{
					if (curr_lt->CBasicPoint.x > ptNode_r.x - sc &&
						curr_lt->CBasicPoint.x < ptNode_r.x + sc &&
						curr_lt->CBasicPoint.y > ptNode_r.y - sc &&
						curr_lt->CBasicPoint.y < ptNode_r.y + sc &&
						curr_lt->CBasicPoint.z > ptNode_r.z - sc &&
						curr_lt->CBasicPoint.z < ptNode_r.z + sc)
						num_in++;
					curr_lt = curr_lt->link;
				}//end while
			}//end while

			FPoint ptNode_l;           //存储El内最近点的信息
			curr_lt = El.front();
			while (curr_lt != NULL)
			{
				if (curr_lt->CBasicPoint.x > ptNode_r.x - sc &&
					curr_lt->CBasicPoint.x < ptNode_r.x + sc &&
					curr_lt->CBasicPoint.y > ptNode_r.y - sc &&
					curr_lt->CBasicPoint.y < ptNode_r.y + sc &&
					curr_lt->CBasicPoint.z > ptNode_r.z - sc &&
					curr_lt->CBasicPoint.z < ptNode_r.z + sc)
				{
					minD = Dist_List(curr_lt->CBasicPoint, ptNode_r);
					ptNode_l.ptID = curr_lt->CBasicPoint.ptID;
					ptNode_l.x = curr_lt->CBasicPoint.x;
					ptNode_l.y = curr_lt->CBasicPoint.y;
					ptNode_l.z = curr_lt->CBasicPoint.z;
					break;             //将第一个符合条件的点赋给minD，ptNode_l
				}//end if
				curr_lt = curr_lt->link;
			}//end while

			curr_lt = El.front();
			while (curr_lt != NULL)
			{
				if (curr_lt->CBasicPoint.x > ptNode_r.x - sc &&
					curr_lt->CBasicPoint.x < ptNode_r.x + sc &&
					curr_lt->CBasicPoint.y > ptNode_r.y - sc &&
					curr_lt->CBasicPoint.y < ptNode_r.y + sc &&
					curr_lt->CBasicPoint.z > ptNode_r.z - sc &&
					curr_lt->CBasicPoint.z < ptNode_r.z + sc)
				{
					D = Dist_List(curr_lt->CBasicPoint, ptNode_r);
					if (minD > D)
					{
						//最近点的信息
						minD = D;
						ptNode_l.ptID = curr_lt->CBasicPoint.ptID;
						ptNode_l.x = curr_lt->CBasicPoint.x;
						ptNode_l.y = curr_lt->CBasicPoint.y;
						ptNode_l.z = curr_lt->CBasicPoint.z;
					}//end if
				}//end if
				curr_lt = curr_lt->link;
			}//end while
//<------------------------End here!
			if (ptNode_l.x == curr_l->CBasicPoint.x &&
				ptNode_l.y == curr_l->CBasicPoint.y &&
				ptNode_l.z == curr_l->CBasicPoint.z)
			{
				//插入匹配点对
				El_corr.insertNode(ptNode_l);
				Er_corr.insertNode(ptNode_r);
			}
			curr_l = curr_l->link;
		}//end while

//求交运算：已知求交平面和直线的空间位置，求直线的交点
//------------------------>Start here!
		nodeType *curr_lc = El_corr.front();
		nodeType *curr_rc = Er_corr.front();
		float k;

		while (curr_lc != NULL)
		{
			switch (flag)
			{
			case 0:
				k = (Min_pFlag + ss + 0.5*tkness - curr_lc->CBasicPoint.x) /
					(curr_rc->CBasicPoint.x - curr_lc->CBasicPoint.x);
				ptNode.ptID = j;
				ptNode.x = Min_pFlag + ss + 0.5*tkness;
				ptNode.y = k * (curr_rc->CBasicPoint.y - curr_lc->CBasicPoint.y) +
					curr_lc->CBasicPoint.y;
				ptNode.z = k * (curr_rc->CBasicPoint.z - curr_lc->CBasicPoint.z) +
					curr_lc->CBasicPoint.z;
				E.insertNode(ptNode);
				break;
			case 1:
				k = (Min_pFlag + ss + 0.5*tkness - curr_lc->CBasicPoint.y) /
					(curr_rc->CBasicPoint.y - curr_lc->CBasicPoint.y);
				ptNode.ptID = j;
				ptNode.x = k * (curr_rc->CBasicPoint.x - curr_lc->CBasicPoint.x) +
					curr_lc->CBasicPoint.x;
				ptNode.y = Min_pFlag + ss + 0.5*tkness;
				ptNode.z = k * (curr_rc->CBasicPoint.z - curr_lc->CBasicPoint.z) +
					curr_lc->CBasicPoint.z;
				E.insertNode(ptNode);
				break;
			case 2:
				k = (Min_pFlag + ss + 0.5*tkness - curr_lc->CBasicPoint.z) /
					(curr_rc->CBasicPoint.z - curr_lc->CBasicPoint.z);
				ptNode.ptID = j;
				ptNode.x = k * (curr_rc->CBasicPoint.x - curr_lc->CBasicPoint.x) +
					curr_lc->CBasicPoint.x;
				ptNode.y = k * (curr_rc->CBasicPoint.y - curr_lc->CBasicPoint.y) +
					curr_lc->CBasicPoint.y;
				ptNode.z = Min_pFlag + ss + 0.5*tkness;
				E.insertNode(ptNode);
				break;
			}
			curr_lc = curr_lc->link;
			curr_rc = curr_rc->link;
		}
		//<------------------------End here!
		ss += tkness;
		j++;
	}//end do-while
	while (j <= ppro.FinalSect - 1);

	El.destroyList();
	Er.destroyList();
	El_corr.destroyList();
	Er_corr.destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//切片多义线构建

inline void CJsGrSlicing::Curves(int flag, float dens)
{
	int i, j;
	linkedListType Ptss;               //Points in each section
	linkedListType Ptsso;              //Points in each section in order

	int temp;
	int sFlag;                         //层切片数据方向的标记
									   //-1：顺时针; 1：逆时针; 0：三点共线
	int sFlag1;
	float max_ps[2];                   //层切片数据上非切片方向的极大值
	float min_ps[2];                   //层切片数据上非切片方向的极小值
	int mark[4];                       //存储四个极值的序号

	FPoint ptsNode;
	nodeType *current, *current1, *currents;
	Ptsio.initList();

	for (i = 0; i < ppro.FinalSect; i++)
	{
		Ptss.initList();
		Ptsso.initList();

		current = E.front();             //置为链表的头节点

		//将每层数据存储到链表Ptss中
		while (current != NULL)
		{
			if (current->CBasicPoint.ptID == i)
			{
				ptsNode.ptID = 0;
				ptsNode.x = current->CBasicPoint.x;
				ptsNode.y = current->CBasicPoint.y;
				ptsNode.z = current->CBasicPoint.z;
				Ptss.insertNode(ptsNode);
			}
			current = current->link;
		}

		//最近点搜索法构建切片多义线
		current = Ptss.front();
		current->CBasicPoint.ptID = 1;
		Ptsso.insertFirst(current->CBasicPoint);
		while (Ptsso.count < Ptss.count)
		{
			currents = researchpt(Ptsso.front(), Ptsso.back(), current);
			if (currents->CBasicPoint.ptID == 1)
				Ptsso.insertFirst(currents->CBasicPoint);
			else
				Ptsso.insertLast(currents->CBasicPoint);
		}

		//将顶点的连接顺序调整为逆时针
		//-------------------------->start here!
				//节点编号赋值
		current = Ptsso.front();
		j = 0;
		while (current != NULL)
		{
			current->CBasicPoint.ptID = j;
			j++;
			current = current->link;
		}

		//根据切片方向求截面数据的极大极小值
		Extreme.initList();
		switch (flag)
		{
			//沿X轴方向切片
		case 0:
			for (j = 0; j < 4; j++)
				mark[j] = 0;
			current = Ptsso.front();
			max_ps[0] = current->CBasicPoint.y;
			max_ps[1] = current->CBasicPoint.z;
			min_ps[0] = current->CBasicPoint.y;
			min_ps[1] = current->CBasicPoint.z;

			while (current != NULL)
			{
				if (max_ps[0] < current->CBasicPoint.y)
				{
					max_ps[0] = current->CBasicPoint.y;
					mark[0] = current->CBasicPoint.ptID;
				}
				else if (min_ps[0] > current->CBasicPoint.y)
				{
					min_ps[0] = current->CBasicPoint.y;
					mark[1] = current->CBasicPoint.ptID;
				}
				if (max_ps[1] < current->CBasicPoint.z)
				{
					max_ps[1] = current->CBasicPoint.z;
					mark[2] = current->CBasicPoint.ptID;
				}
				else if (min_ps[1] > current->CBasicPoint.z)
				{
					min_ps[1] = current->CBasicPoint.z;
					mark[3] = current->CBasicPoint.ptID;
				}
				current = current->link;
			}

			//将极值点按ptID顺序写入链表Extreme中
			for (j = 0; j < 4; j++)
			{
				current = Ptsso.front();
				while (current != NULL)
				{
					if (current->CBasicPoint.ptID == mark[j])
						Extreme.insertNode(current->CBasicPoint);
					current = current->link;
				}
			}

			//若有重复点，按顺序删除前一点
			current = Extreme.front();
			while (current->link != NULL)
			{
				temp = current->CBasicPoint.ptID;
				if (current->link->CBasicPoint.ptID == temp)
				{
					current = current->link;
					Extreme.deleteNode(current->head->CBasicPoint);
				}
				else
					current = current->link;
			}

			//由矢量叉乘判定多义线方向，分三种情况讨论
			//四个极值点任意不重复
			sFlag = 1;                   //初始化sFlag
			if (Extreme.count == 4)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//四个极值点有一点重复
			else if (Extreme.count == 3)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//四个极值点有两点重复，取其中一点的相邻两点
			else if (Extreme.count == 2)
			{
				current = Extreme.front();
				temp = current->CBasicPoint.ptID;
				Extreme.initList();
				current = Ptsso.front();
				while (current != NULL)
				{
					if (current->CBasicPoint.ptID == temp)
					{
						//如果当前点不是头节点和尾节点
						if (current->head != NULL && current->link != NULL)
						{
							Extreme.insertNode(current->head->CBasicPoint);
							Extreme.insertNode(current->CBasicPoint);
							Extreme.insertNode(current->link->CBasicPoint);
						}
						//如果当前点是头节点
						else if (current->head == NULL)
						{
							Extreme.insertLast(Ptsso.back()->CBasicPoint);
							Extreme.insertLast(current->CBasicPoint);
							Extreme.insertLast(current->link->CBasicPoint);
						}//end else
						break;
					}//end if
					current = current->link;
				}
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//cout<<sFlag<<endl;       //test：输出多义线方向标记

			float Dse;                 //两端点的距离
			switch (sFlag)
			{
				//若为顺时针，从尾节点开始记录
			case -1:
				//判定多义线的封闭性
				Dse = Dist_List(Ptsso.front()->CBasicPoint,
					Ptsso.back()->CBasicPoint);
				if (Dse <= 3 * dens)
					Ptsso.insertLast(Ptsso.front()->CBasicPoint);

				current = Ptsso.back();
				while (current != NULL)
				{
					ptsNode.ptID = i;
					ptsNode.x = current->CBasicPoint.x;
					ptsNode.y = current->CBasicPoint.y;
					ptsNode.z = current->CBasicPoint.z;
					Ptsio.insertLast(ptsNode);
					current = current->head;
				}
				break;

				//若为逆时针，从头节点开始记录
			case 1:
				//判定多义线的封闭性
				Dse = Dist_List(Ptsso.front()->CBasicPoint,
					Ptsso.back()->CBasicPoint);
				if (Dse <= 3 * dens)
					Ptsso.insertLast(Ptsso.front()->CBasicPoint);

				current = Ptsso.front();
				while (current != NULL)
				{
					ptsNode.ptID = i;
					ptsNode.x = current->CBasicPoint.x;
					ptsNode.y = current->CBasicPoint.y;
					ptsNode.z = current->CBasicPoint.z;
					Ptsio.insertLast(ptsNode);
					current = current->link;
				}

				break;
				//若三点共线，输出错误
			case 0:
				cout << "Error!" << endl;
				break;
			}

			break;
			//沿Y轴方向切片
		case 1:
			for (j = 0; j < 4; j++)
				mark[j] = 0;
			current = Ptsso.front();
			max_ps[0] = current->CBasicPoint.x;
			max_ps[1] = current->CBasicPoint.z;
			min_ps[0] = current->CBasicPoint.x;
			min_ps[1] = current->CBasicPoint.z;

			while (current != NULL)
			{
				if (max_ps[0] < current->CBasicPoint.x)
				{
					max_ps[0] = current->CBasicPoint.x;
					mark[0] = current->CBasicPoint.ptID;
				}
				else if (min_ps[0] > current->CBasicPoint.x)
				{
					min_ps[0] = current->CBasicPoint.x;
					mark[1] = current->CBasicPoint.ptID;
				}
				if (max_ps[1] < current->CBasicPoint.z)
				{
					max_ps[1] = current->CBasicPoint.z;
					mark[2] = current->CBasicPoint.ptID;
				}
				else if (min_ps[1] > current->CBasicPoint.z)
				{
					min_ps[1] = current->CBasicPoint.z;
					mark[3] = current->CBasicPoint.ptID;
				}
				current = current->link;
			}

			//将极值点按ptID顺序写入链表Extreme中
			for (j = 0; j < 4; j++)
			{
				current = Ptsso.front();
				while (current != NULL)
				{
					if (current->CBasicPoint.ptID == mark[j])
						Extreme.insertNode(current->CBasicPoint);
					current = current->link;
				}
			}

			//若有重复点，按顺序删除前一点
			current = Extreme.front();
			while (current->link != NULL)
			{
				temp = current->CBasicPoint.ptID;
				if (current->link->CBasicPoint.ptID == temp)
				{
					current = current->link;
					Extreme.deleteNode(current->head->CBasicPoint);
				}
				else
					current = current->link;
			}

			//由矢量叉乘判定多义线方向，分三种情况讨论
			//四个极值点任意不重复
			sFlag = 1;                   //初始化sFlag
			if (Extreme.count == 4)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//四个极值点有一点重复
			else if (Extreme.count == 3)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//四个极值点有两点重复，取其中一点的相邻两点
			else if (Extreme.count == 2)
			{
				current = Extreme.front();
				temp = current->CBasicPoint.ptID;
				Extreme.initList();
				current = Ptsso.front();
				while (current != NULL)
				{
					if (current->CBasicPoint.ptID == temp)
					{
						//如果当前点不是头节点和尾节点
						if (current->head != NULL && current->link != NULL)
						{
							Extreme.insertNode(current->head->CBasicPoint);
							Extreme.insertNode(current->CBasicPoint);
							Extreme.insertNode(current->link->CBasicPoint);
						}
						//如果当前点是头节点
						else if (current->head == NULL)
						{
							Extreme.insertLast(Ptsso.back()->CBasicPoint);
							Extreme.insertLast(current->CBasicPoint);
							Extreme.insertLast(current->link->CBasicPoint);
						}//end else
						break;
					}//end if
					current = current->link;
				}
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//cout<<sFlag<<endl;       //test：输出多义线方向标记

			float Dse1;                 //两端点的距离
			switch (sFlag)
			{
				//若为顺时针，从尾节点开始记录
			case -1:
				//判定多义线的封闭性
				Dse1 = Dist_List(Ptsso.front()->CBasicPoint,
					Ptsso.back()->CBasicPoint);
				if (Dse1 <= 3 * dens)
					Ptsso.insertLast(Ptsso.front()->CBasicPoint);

				current = Ptsso.back();
				while (current != NULL)
				{
					ptsNode.ptID = i;
					ptsNode.x = current->CBasicPoint.x;
					ptsNode.y = current->CBasicPoint.y;
					ptsNode.z = current->CBasicPoint.z;
					Ptsio.insertLast(ptsNode);
					current = current->head;
				}
				break;

				//若为逆时针，从头节点开始记录
			case 1:
				//判定多义线的封闭性
				Dse = Dist_List(Ptsso.front()->CBasicPoint,
					Ptsso.back()->CBasicPoint);
				if (Dse <= 3 * dens)
					Ptsso.insertLast(Ptsso.front()->CBasicPoint);

				current = Ptsso.front();
				while (current != NULL)
				{
					ptsNode.ptID = i;
					ptsNode.x = current->CBasicPoint.x;
					ptsNode.y = current->CBasicPoint.y;
					ptsNode.z = current->CBasicPoint.z;
					Ptsio.insertLast(ptsNode);
					current = current->link;
				}

				break;
				//若三点共线，输出错误
			case 0:
				cout << "Error!" << endl;
				break;
			}
			break;

			//沿Z轴方向切片
		case 2:
			for (j = 0; j < 4; j++)
				mark[j] = 0;
			current = Ptsso.front();
			max_ps[0] = current->CBasicPoint.x;
			max_ps[1] = current->CBasicPoint.y;
			min_ps[0] = current->CBasicPoint.x;
			min_ps[1] = current->CBasicPoint.y;

			while (current != NULL)
			{
				if (max_ps[0] < current->CBasicPoint.x)
				{
					max_ps[0] = current->CBasicPoint.x;
					mark[0] = current->CBasicPoint.ptID;
				}
				else if (min_ps[0] > current->CBasicPoint.x)
				{
					min_ps[0] = current->CBasicPoint.x;
					mark[1] = current->CBasicPoint.ptID;
				}
				if (max_ps[1] < current->CBasicPoint.y)
				{
					max_ps[1] = current->CBasicPoint.y;
					mark[2] = current->CBasicPoint.ptID;
				}
				else if (min_ps[1] > current->CBasicPoint.y)
				{
					min_ps[1] = current->CBasicPoint.y;
					mark[3] = current->CBasicPoint.ptID;
				}
				current = current->link;
			}

			//将极值点按ptID顺序写入链表Extreme中
			for (j = 0; j < 4; j++)
			{
				current = Ptsso.front();
				while (current != NULL)
				{
					if (current->CBasicPoint.ptID == mark[j])
						Extreme.insertNode(current->CBasicPoint);
					current = current->link;
				}
			}

			//若有重复点，按顺序删除前一点
			current = Extreme.front();
			while (current->link != NULL)
			{
				temp = current->CBasicPoint.ptID;
				if (current->link->CBasicPoint.ptID == temp)
				{
					current = current->link;
					Extreme.deleteNode(current->head->CBasicPoint);
				}
				else
					current = current->link;
			}

			//判断当前链表的方向
			sFlag = 1;
			current = Ptsso.front();
			current1 = Ptsso.back();
			if (current->CBasicPoint.y < current1->CBasicPoint.y)
			{
				sFlag = -1;
			}
			else if (current->CBasicPoint.y > current1->CBasicPoint.y)
			{
				sFlag = 1;
			}

			sFlag1 = pow(-1, i);
			float Dse2;                 //两端点的距离
			int F;
			F = sFlag * sFlag1;
			switch (F)
			{
				//若为顺时针，从尾节点开始记录
			case -1:
				//判定多义线的封闭性
				Dse2 = Dist_List(Ptsso.front()->CBasicPoint,
					Ptsso.back()->CBasicPoint);
				if (Dse2 <= 3 * dens)
					Ptsso.insertLast(Ptsso.front()->CBasicPoint);

				current = Ptsso.back();
				while (current != NULL)
				{
					ptsNode.ptID = i;
					ptsNode.x = current->CBasicPoint.x;
					ptsNode.y = current->CBasicPoint.y;
					ptsNode.z = current->CBasicPoint.z;
					Ptsio.insertLast(ptsNode);
					current = current->head;
				}
				break;

				//若为逆时针，从头节点开始记录
			case 1:
				//判定多义线的封闭性
				Dse = Dist_List(Ptsso.front()->CBasicPoint,
					Ptsso.back()->CBasicPoint);
				if (Dse <= 3 * dens)
					Ptsso.insertLast(Ptsso.front()->CBasicPoint);

				current = Ptsso.front();
				while (current != NULL)
				{
					ptsNode.ptID = i;
					ptsNode.x = current->CBasicPoint.x;
					ptsNode.y = current->CBasicPoint.y;
					ptsNode.z = current->CBasicPoint.z;
					Ptsio.insertLast(ptsNode);
					current = current->link;
				}
				break;
				//若三点共线，输出错误
			case 0:
				cout << "Error!" << endl;
				break;
			}
			break;
		}//end switch*/
  // <--------------------------end here!
	}//end for
	cout<<Ptsio.count<<endl;

	E.destroyList();
	Ptss.destroyList();
	Ptsso.destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//绘制点云切片的函数

inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CJsGrSlicing::cloudout(void)
{
	nodeType *currentio;
	for (int i = 0; i < ppro.FinalSect; i++)
	{
		currentio = Ptsio.front();
		while (currentio != NULL)
		{
			if (currentio->CBasicPoint.ptID == i)
			{
				pcl::PointXYZ basic_point;
				basic_point.x = currentio->CBasicPoint.x;
				basic_point.y = currentio->CBasicPoint.y;
				basic_point.z = currentio->CBasicPoint.z;

				cloud3->points.push_back(basic_point);

			}
			currentio = currentio->link;
			//std::cout<< currentio->CBasicPoint.x << " " << currentio->CBasicPoint.y << " " << currentio->CBasicPoint.z << endl;
		}
	}
	cloud3->width = (int)cloud3->points.size();
	cloud3->height = 1;
	cout << "The slice has " << cloud3->size() << " points" << endl;
	return(cloud3);
}



#endif //__LMZ_CONTOUR_H__
#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//定义一个基于射线的匹配类：初始化、多顶点匹配、丢失点匹配、等比例映射
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_CORRESPONDENCE_H__
#define __LMZ_CORRESPONDENCE_H__

//#include "S_Preprocessing.h"
#include "linkedListType.h"
#include "Function.h"

///////////////////////////////////////////////////////////////////////////////
class CJsGrCorr
{
private:
	linkedListType Ptss_1, Ptss_2;      //每层切片数据(凸包数据)
	linkedListType Ptss1, Ptss2;        //每层切片数据(原始数据)
	linkedListType Init_Pts1, Init_Pts2;//初始化后的切片数据(凸包数据)
	linkedListType InitPts1, InitPts2;  //初始化后的切片数据(原始数据)
	linkedListType Sing_Pts1, Sing_Pts2;//多顶点匹配后的切片数据(凸包数据)
	linkedListType SingPts1, SingPts2;  //多顶点匹配后的切片数据(原始数据)
	linkedListType Mult_Pts1, Mult_Pts2;//丢失点匹配后的切片数据(凸包数据)
	float(*Pts1)[3], (*Pts2)[3];       //等比例映射得到的数组

	bool *flag1, *flag2;                //多义线是否封闭的标记
	bool *numFlag;                     //顶点数量大小的标记
	float(*center1)[3], (*center2)[3]; //每层切片数据的中心点

public:
	linkedListType MultPts1, MultPts2;  //丢失点匹配匹配后的切片数据(原始数据)
	linkedListType FinaPts1, FinaPts2;  //等比例映射后的切片数据(原始数据)	
	int *lostN;                        //每层切片上丢失点的个数

	void Init_Corr(void);              //初始化：封闭性检查、比较顶点数量、对齐
	void Sing_Corr(void);              //建立多义线顶点的单匹配：多顶点匹配
	void Mult_Corr(void);              //建立丢失点的重匹配：丢失点匹配
	void Fina_Corr(void);              //建立控制点的最终匹配：等比例映射
	void Display(void);                //显示函数
};
CJsGrCorr corr;

///////////////////////////////////////////////////////////////////////////////
//判断多义线的封闭性，比较两条多义线的顶点数量
void CJsGrCorr::Init_Corr(void)
{
	int i, j;
	nodeType *current1, *current2;
	nodeType *startPt;
	linkedListType temp_1, temp_2, temp1, temp2;
	float Angle, MinA;
	int mark;

	Init_Pts1.initList();
	Init_Pts2.initList();
	InitPts1.initList();
	InitPts2.initList();

	flag1 = new bool[ppro.FinalSect];
	flag2 = new bool[ppro.FinalSect];
	numFlag = new bool[ppro.FinalSect];
	center1 = new float[ppro.FinalSect][3];
	center2 = new float[ppro.FinalSect][3];
	for (i = 0; i < ppro.FinalSect; i++)
		for (j = 0; j < 3; j++)
		{
			center1[i][j] = 0;
			center2[i][j] = 0;
		}

	for (i = 0; i < ppro.FinalSect; i++)
	{
		Ptss_1.initList();
		Ptss_2.initList();
		Ptss1.initList();
		Ptss2.initList();

		//切片数据赋值
		//-------------------------->start here!
				//凸包
		current1 = sppro1.CH_Pts.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss_1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = sppro2.CH_Pts.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss_2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//原始数据
		current1 = sppro1.C_Pts.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = sppro2.C_Pts.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//<--------------------------end here!

				//判断切片多义线的封闭性，若封闭删除最后一点，返回flag1[i]/flag2[i]
		if (Ptss_1.front()->CBasicPoint.x == Ptss_1.back()->CBasicPoint.x &&
			Ptss_1.front()->CBasicPoint.y == Ptss_1.back()->CBasicPoint.y &&
			Ptss_1.front()->CBasicPoint.z == Ptss_1.back()->CBasicPoint.z)
		{
			//凸包
			Ptss_1.back()->CBasicPoint.ptID = -1;
			Ptss_1.deleteNode(Ptss_1.back()->CBasicPoint);
			//原始数据
			Ptss1.back()->CBasicPoint.ptID = -1;
			Ptss1.deleteNode(Ptss1.back()->CBasicPoint);
			flag1[i] = 1;
		}
		else
			flag1[i] = 0;

		if (Ptss_2.front()->CBasicPoint.x == Ptss_2.back()->CBasicPoint.x &&
			Ptss_2.front()->CBasicPoint.y == Ptss_2.back()->CBasicPoint.y &&
			Ptss_2.front()->CBasicPoint.z == Ptss_2.back()->CBasicPoint.z)
		{
			Ptss_2.back()->CBasicPoint.ptID = -1;
			Ptss_2.deleteNode(Ptss_2.back()->CBasicPoint);
			Ptss2.back()->CBasicPoint.ptID = -1;
			Ptss2.deleteNode(Ptss2.back()->CBasicPoint);
			flag2[i] = 1;
		}
		else
			flag2[i] = 0;
		//cout<<flag1[i]<<","<<flag2[i]<<endl;

		//比较多义线的顶点数量
		if (Ptss_1.count > Ptss_2.count)
			numFlag[i] = 0;
		else
			numFlag[i] = 1;
		//cout<<i<<","<<numFlag[i]<<endl;

		//对齐(凸包顶点)
		current1 = Ptss_1.front();
		while (current1 != NULL)
		{
			center1[i][0] += current1->CBasicPoint.x;
			center1[i][1] += current1->CBasicPoint.y;
			center1[i][2] += current1->CBasicPoint.z;
			current1 = current1->link;
		}
		center1[i][0] /= Ptss_1.count;
		center1[i][1] /= Ptss_1.count;
		center1[i][2] /= Ptss_1.count;

		current1 = Ptss_1.front();
		while (current1 != NULL)
		{
			current1->CBasicPoint.x -= center1[i][0];
			current1->CBasicPoint.y -= center1[i][1];
			current1->CBasicPoint.z -= center1[i][2];
			current1 = current1->link;
		}

		current2 = Ptss_2.front();
		while (current2 != NULL)
		{
			center2[i][0] += current2->CBasicPoint.x;
			center2[i][1] += current2->CBasicPoint.y;
			center2[i][2] += current2->CBasicPoint.z;
			current2 = current2->link;
		}
		center2[i][0] /= Ptss_2.count;
		center2[i][1] /= Ptss_2.count;
		center2[i][2] /= Ptss_2.count;

		current2 = Ptss_2.front();
		while (current2 != NULL)
		{
			current2->CBasicPoint.x -= center2[i][0];
			current2->CBasicPoint.y -= center2[i][1];
			current2->CBasicPoint.z -= center2[i][2];
			current2 = current2->link;
		}

		//调整封闭多义线的起点
		FPoint center;
		center.x = 0.0;
		center.y = 0.0;
		center.z = 0.0;
		if (flag1[i] == 0 && flag2[i] != 0)
		{
			current2 = Ptss_2.front();
			MinA = Ray_Firing(Ptss_1.front()->CBasicPoint, center,
				current2->CBasicPoint);
			startPt = current2;
			mark = 0;
			while (current2 != NULL)
			{
				Angle = Ray_Firing(Ptss_1.front()->CBasicPoint, center,
					current2->CBasicPoint);
				if (MinA > Angle)
				{
					MinA = Angle;
					startPt = current2;
					mark = current2->CBasicPoint.ptID_s;
				}
				current2 = current2->link;
			}

			//写入temp链表
			temp_2.initList();
			j = 0;
			current2 = startPt;
			while (current2 != NULL)
			{
				current2->CBasicPoint.ptID_s = j;
				temp_2.insertLast(current2->CBasicPoint);
				j++;
				current2 = current2->link;
			}
			current2 = Ptss_2.front();
			while (current2 != startPt)
			{
				current2->CBasicPoint.ptID_s = j;
				temp_2.insertLast(current2->CBasicPoint);
				j++;
				current2 = current2->link;
			}
			Ptss_2.initList();
			current2 = temp_2.front();
			while (current2 != NULL)
			{
				Ptss_2.insertLast(current2->CBasicPoint);
				current2 = current2->link;
			}

			temp2.initList();
			current2 = Ptss2.front();
			while (current2 != NULL)
			{
				if (current2->CBasicPoint.ptID_s == mark)
				{
					startPt = current2;
					break;
				}
				current2 = current2->link;
			}
			j = 0;
			current2 = startPt;
			while (current2 != NULL)
			{
				current2->CBasicPoint.ptID_s = j;
				temp2.insertLast(current2->CBasicPoint);
				j++;
				current2 = current2->link;
			}
			current2 = Ptss2.front();
			while (current2 != startPt)
			{
				current2->CBasicPoint.ptID_s = j;
				temp2.insertLast(current2->CBasicPoint);
				j++;
				current2 = current2->link;
			}
			Ptss2.initList();
			current2 = temp2.front();
			while (current2 != NULL)
			{
				Ptss2.insertLast(current2->CBasicPoint);
				current2 = current2->link;
			}

			temp_2.destroyList();
			temp2.destroyList();
		}
		else if (flag1[i] != 0 && flag2[i] == 0)
		{
			current1 = Ptss_1.front();
			MinA = Ray_Firing(Ptss_2.front()->CBasicPoint, center,
				current1->CBasicPoint);
			startPt = current1;
			mark = 0;
			while (current1 != NULL)
			{
				Angle = Ray_Firing(Ptss_2.front()->CBasicPoint, center,
					current1->CBasicPoint);
				if (MinA > Angle)
				{
					MinA = Angle;
					startPt = current1;
					mark = current1->CBasicPoint.ptID_s;
				}
				current1 = current1->link;
			}

			//写入temp链表
			temp_1.initList();
			j = 0;
			current1 = startPt;
			while (current1 != NULL)
			{
				current1->CBasicPoint.ptID_s = j;
				temp_1.insertLast(current1->CBasicPoint);
				j++;
				current1 = current1->link;
			}
			current1 = Ptss_1.front();
			while (current1 != startPt)
			{
				current1->CBasicPoint.ptID_s = j;
				temp_1.insertLast(current1->CBasicPoint);
				j++;
				current1 = current1->link;
			}
			Ptss_1.initList();
			current1 = temp_1.front();
			while (current1 != NULL)
			{
				Ptss_1.insertLast(current1->CBasicPoint);
				current1 = current1->link;
			}

			temp1.initList();
			current1 = Ptss1.front();
			while (current1 != NULL)
			{
				if (current1->CBasicPoint.ptID_s == mark)
				{
					startPt = current1;
					break;
				}
				current1 = current1->link;
			}

			j = 0;
			current1 = startPt;
			while (current1 != NULL)
			{
				current1->CBasicPoint.ptID_s = j;
				temp1.insertLast(current1->CBasicPoint);
				j++;
				current1 = current1->link;
			}
			current1 = Ptss1.front();
			while (current1 != startPt)
			{
				current1->CBasicPoint.ptID_s = j;
				temp1.insertLast(current1->CBasicPoint);
				j++;
				current1 = current1->link;
			}
			Ptss1.initList();
			current1 = temp1.front();
			while (current1 != NULL)
			{
				Ptss1.insertLast(current1->CBasicPoint);
				current1 = current1->link;
			}

			temp_1.destroyList();
			temp1.destroyList();
		}

		//写入链表
		//-------------------------->start here!
				//凸包
		current1 = Ptss_1.front();
		while (current1 != NULL)
		{
			Init_Pts1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = Ptss_2.front();
		while (current2 != NULL)
		{
			Init_Pts2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//原始数据
		current1 = Ptss1.front();
		while (current1 != NULL)
		{
			InitPts1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = Ptss2.front();
		while (current2 != NULL)
		{
			InitPts2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//<--------------------------end here!
	}//end for
	Ptss_1.destroyList();
	Ptss_2.destroyList();
	Ptss1.destroyList();
	Ptss2.destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//遍历顶点数量多的多义线凸包各顶点，建立顶点的单匹配
void CJsGrCorr::Sing_Corr(void)
{
	int i, j, k;
	nodeType *current1, *current2;
	int *sing_m1, *sing_m2;             //单匹配对应的ptID_s号
	float *Angle, MinA;
	FPoint center;

	Sing_Pts1.initList();
	Sing_Pts2.initList();
	SingPts1.initList();
	SingPts2.initList();
	center.x = 0.0;
	center.y = 0.0;
	center.z = 0.0;

	for (i = 0; i < ppro.FinalSect; i++)
	{
		Ptss_1.initList();
		Ptss_2.initList();
		Ptss1.initList();
		Ptss2.initList();

		//切片数据赋值(凸包)
		current1 = Init_Pts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss_1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = Init_Pts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss_2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//切片数据赋值(原始)
		current1 = InitPts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = InitPts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}

		//根据顶点数量大小的标记，建立顶点数量多的多义线顶点的单匹配
		switch (numFlag[i])
		{
		case 0:
			sing_m1 = new int[Ptss_1.count];
			for (j = 0; j < Ptss_1.count; j++)
				sing_m1[j] = j;
			sing_m2 = new int[Ptss_1.count];
			Angle = new float[Ptss_2.count];

			j = 0;
			current1 = Ptss_1.front();
			while (current1 != NULL)
			{
				k = 0;
				current2 = Ptss_2.front();
				while (current2 != NULL)
				{
					Angle[k] = Ray_Firing(current1->CBasicPoint, center,
						current2->CBasicPoint);
					k++;
					current2 = current2->link;
				}
				MinA = Angle[0];
				sing_m2[j] = 0;
				for (k = 1; k < Ptss_2.count; k++)
				{
					if (MinA > Angle[k])
					{
						MinA = Angle[k];
						sing_m2[j] = k;
					}
				}
				j++;
				current1 = current1->link;
			}
			//for(j=0;j<Ptss_1.count;j++)
			//    cout<<sing_m1[j]<<","<<sing_m2[j]<<endl;

			//单匹配的赋值(凸包)
			current1 = Ptss_1.front();
			while (current1 != NULL)
			{
				Sing_Pts1.insertLast(current1->CBasicPoint);
				current1 = current1->link;
			}
			for (j = 0; j < Ptss_1.count; j++)
			{
				current2 = Ptss_2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == sing_m2[j])
					{
						Sing_Pts2.insertLast(current2->CBasicPoint);
						break;
					}
					current2 = current2->link;
				}
			}
			//单匹配的赋值(原始)
			current1 = Ptss1.front();
			while (current1 != NULL)
			{
				SingPts1.insertLast(current1->CBasicPoint);
				current1 = current1->link;
			}
			for (j = 0; j < Ptss_1.count; j++)
			{
				current2 = Ptss2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == sing_m2[j])
					{
						SingPts2.insertLast(current2->CBasicPoint);
						break;
					}
					current2 = current2->link;
				}
			}
			break;
		case 1:
			sing_m1 = new int[Ptss_2.count];
			sing_m2 = new int[Ptss_2.count];
			for (j = 0; j < Ptss_2.count; j++)
				sing_m2[j] = j;
			Angle = new float[Ptss_1.count];

			j = 0;
			current2 = Ptss_2.front();
			while (current2 != NULL)
			{
				k = 0;
				current1 = Ptss_1.front();
				while (current1 != NULL)
				{
					Angle[k] = Ray_Firing(current2->CBasicPoint, center,
						current1->CBasicPoint);
					k++;
					current1 = current1->link;
				}
				MinA = Angle[0];
				sing_m1[j] = 0;
				for (k = 1; k < Ptss_1.count; k++)
				{
					if (MinA > Angle[k])
					{
						MinA = Angle[k];
						sing_m1[j] = k;
					}
				}
				j++;
				current2 = current2->link;
			}

			//单匹配的赋值(凸包)
			current2 = Ptss_2.front();
			while (current2 != NULL)
			{
				Sing_Pts2.insertLast(current2->CBasicPoint);
				current2 = current2->link;
			}
			for (j = 0; j < Ptss2.count; j++)
			{
				current1 = Ptss_1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == sing_m1[j])
					{
						Sing_Pts1.insertLast(current1->CBasicPoint);
						break;
					}
					current1 = current1->link;
				}
			}
			//单匹配的赋值(原始)
			current2 = Ptss2.front();
			while (current2 != NULL)
			{
				SingPts2.insertLast(current2->CBasicPoint);
				current2 = current2->link;
			}
			for (j = 0; j < Ptss_2.count; j++)
			{
				current1 = Ptss1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == sing_m1[j])
					{
						SingPts1.insertLast(current1->CBasicPoint);
						break;
					}
					current1 = current1->link;
				}
			}
			break;
		}//end switch
	}//end for

	Ptss_1.destroyList();
	Ptss_2.destroyList();
	Ptss1.destroyList();
	Ptss2.destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//寻找丢失点，建立一一匹配：重匹配
void CJsGrCorr::Mult_Corr(void)
{
	int i, j, k, p, q;
	nodeType *current1, *current2;
	linkedListType P1, P2;
	linkedListType Lost_Ptss;          //每层丢失点数据

	bool *flag;                        //丢失点的标记
	int *lost_m1, *lost_m2;             //丢失点的序号
	int *sing_m1, *sing_m2;             //单匹配对应的ptID_s号
	int *mult_m1, *mult_m2;             //重匹配对应的ptID_s号
	float *Angle, MinA;
	bool found;                        //在中间插入丢失点的标记
	FPoint center;

	Mult_Pts1.initList();
	Mult_Pts2.initList();
	MultPts1.initList();
	MultPts2.initList();
	lostN = new int[ppro.FinalSect];
	center.x = 0.0;
	center.y = 0.0;
	center.z = 0.0;

	for (i = 0; i < ppro.FinalSect; i++)
	{
		P1.initList();
		P2.initList();
		Ptss_1.initList();
		Ptss_2.initList();
		Ptss1.initList();
		Ptss2.initList();

		//初始化后的切片数据赋值(凸包)
		current1 = Init_Pts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				P1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = Init_Pts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				P2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//单顶点匹配后的切片数据赋值(凸包)
		current1 = Sing_Pts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss_1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = Sing_Pts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss_2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//切片数据赋值(原始)
		current1 = InitPts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = InitPts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}

		//根据顶点数量大小的标记，建立丢失点的匹配
		switch (numFlag[i])
		{
		case 0:
			//sing_m1/sing_m2的重新赋值
			sing_m1 = new int[Ptss_1.count];
			for (j = 0; j < Ptss_1.count; j++)
				sing_m1[j] = j;
			sing_m2 = new int[Ptss_1.count];

			j = 0;
			current2 = Ptss_2.front();
			while (current2 != NULL)
			{
				sing_m2[j] = current2->CBasicPoint.ptID_s;
				j++;
				current2 = current2->link;
			}
			//for(j=0;j<Ptss_1.count;j++)
			//    cout<<sing_m1[j]<<","<<sing_m2[j]<<endl; //test

			//lostN[i]的计算
			flag = new bool[P2.count];
			for (j = 0; j < P2.count; j++)
				flag[j] = 0;

			for (j = 0; j < P2.count; j++)
			{
				current2 = Ptss_2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == j)
					{
						flag[j] = 1;
						break;
					}
					current2 = current2->link;
				}
			}
			lostN[i] = 0;
			for (j = 0; j < P2.count; j++)
				if (flag[j] == 0)
					lostN[i]++;
			//cout<<"丢失点的数量为"<<lostN[i]<<endl;      //test

			//如果存在丢失点
			if (lostN[i] > 0)
			{
				Lost_Ptss.initList();

				//将sing_m1/sing_m2赋给mult_m1/mult_m2
				mult_m1 = new int[Ptss_1.count + lostN[i]];
				mult_m2 = new int[Ptss_1.count + lostN[i]];
				for (j = 0; j < Ptss_1.count; j++)
				{
					mult_m1[j] = sing_m1[j];
					mult_m2[j] = sing_m2[j];
				}
				//将丢失点的序号赋值给lost_m2
				lost_m1 = new int[lostN[i]];
				lost_m2 = new int[lostN[i]];
				k = 0;
				for (j = 0; j < P2.count; j++)
					if (flag[j] == 0)
					{
						lost_m2[k] = j;
						k++;
					}

				//在P2中找到丢失点，存入Lost_Ptss中
				for (j = 0; j < lostN[i]; j++)
				{
					current2 = P2.front();
					while (current2 != NULL)
					{
						if (current2->CBasicPoint.ptID_s == lost_m2[j])
						{
							Lost_Ptss.insertLast(current2->CBasicPoint);
							break;
						}
						current2 = current2->link;
					}
				}
				//丢失点的匹配
				k = 0;
				Angle = new float[P1.count];
				current2 = Lost_Ptss.front();
				while (current2 != NULL)
				{
					j = 0;
					current1 = P1.front();
					while (current1 != NULL)
					{
						Angle[j] = Ray_Firing(current2->CBasicPoint, center,
							current1->CBasicPoint);
						j++;
						current1 = current1->link;
					}
					MinA = Angle[0];
					lost_m1[k] = 0;
					for (j = 1; j < P1.count; j++)
						if (MinA > Angle[j])
						{
							MinA = Angle[j];
							lost_m1[k] = j;
						}
					k++;
					current2 = current2->link;
				}

				//for(j=0;j<lostN[i];j++)
				//	cout<<lost_m1[j]<<","<<lost_m2[j]<<endl; //test

				p = 0;
				for (j = 0; j < lostN[i]; j++)
				{
					found = false;
					for (k = 0; k < Ptss_1.count + p; k++)
						if (mult_m2[k] > lost_m2[j])
						{
							for (q = Ptss_1.count + p; q > k; q--)
							{
								mult_m2[q] = mult_m2[q - 1];
								mult_m1[q] = mult_m1[q - 1];
							}
							mult_m2[k] = lost_m2[j];
							mult_m1[k] = lost_m1[j];
							p++;
							found = true;
							break;
						}
					if (!found)
					{
						mult_m1[Ptss_1.count + p] = lost_m1[j];
						mult_m2[Ptss_1.count + p] = lost_m2[j];
						p++;
					}
				}
				//for(j=0;j<Ptss_1.count+lostN[i];j++)
				//	cout<<mult_m1[j]<<","<<mult_m2[j]<<endl;
			}//end if
			else                       //如果不存在丢失点
			{
				for (j = 0; j < Ptss_1.count; j++)
				{
					mult_m1[j] = sing_m1[j];
					mult_m2[j] = sing_m2[j];
				}
			}

			//重匹配的赋值(凸包)
			for (j = 0; j < Ptss_1.count + lostN[i]; j++)
			{
				current1 = P1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == mult_m1[j])
					{
						Mult_Pts1.insertLast(current1->CBasicPoint);
						break;
					}
					current1 = current1->link;
				}
			}
			for (j = 0; j < Ptss_1.count + lostN[i]; j++)
			{
				current2 = P2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == mult_m2[j])
					{
						Mult_Pts2.insertLast(current2->CBasicPoint);
						break;
					}
					current2 = current2->link;
				}
			}
			//重匹配的赋值(原始)
			for (j = 0; j < Ptss_1.count + lostN[i]; j++)
			{
				current1 = Ptss1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == mult_m1[j])
					{
						MultPts1.insertLast(current1->CBasicPoint);
						break;
					}
					current1 = current1->link;
				}
			}
			for (j = 0; j < Ptss_1.count + lostN[i]; j++)
			{
				current2 = Ptss2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == mult_m2[j])
					{
						MultPts2.insertLast(current2->CBasicPoint);
						break;
					}
					current2 = current2->link;
				}
			}
			break;

		case 1:
			//sing_m1/sing_m2的重新赋值
			sing_m1 = new int[Ptss_2.count];
			sing_m2 = new int[Ptss_2.count];
			for (j = 0; j < Ptss_2.count; j++)
				sing_m2[j] = j;

			j = 0;
			current1 = Ptss_1.front();
			while (current1 != NULL)
			{
				sing_m1[j] = current1->CBasicPoint.ptID_s;
				j++;
				current1 = current1->link;
			}
			//for(j=0;j<Ptss_2.count;j++)
			//    cout<<sing_m1[j]<<","<<sing_m2[j]<<endl; //test

			//lostN[i]的计算
			flag = new bool[P1.count];
			for (j = 0; j < P1.count; j++)
				flag[j] = 0;

			for (j = 0; j < P1.count; j++)
			{
				current1 = Ptss_1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == j)
					{
						flag[j] = 1;
						break;
					}
					current1 = current1->link;
				}
			}
			lostN[i] = 0;
			for (j = 0; j < P1.count; j++)
				if (flag[j] == 0)
					lostN[i]++;
			//cout<<"丢失点的数量为"<<lostN[i]<<endl;      //test

			//如果存在丢失点
			if (lostN[i] > 0)
			{
				Lost_Ptss.initList();

				//将sing_m1/sing_m2赋给mult_m1/mult_m2
				mult_m1 = new int[Ptss_2.count + lostN[i]];
				mult_m2 = new int[Ptss_2.count + lostN[i]];
				for (j = 0; j < Ptss_2.count; j++)
				{
					mult_m1[j] = sing_m1[j];
					mult_m2[j] = sing_m2[j];
				}
				//将丢失点的序号赋值给lost_m1
				lost_m1 = new int[lostN[i]];
				lost_m2 = new int[lostN[i]];
				k = 0;
				for (j = 0; j < P1.count; j++)
					if (flag[j] == 0)
					{
						lost_m1[k] = j;
						k++;
					}

				//在P1中找到丢失点，存入Lost_Ptss中
				for (j = 0; j < lostN[i]; j++)
				{
					current1 = P1.front();
					while (current1 != NULL)
					{
						if (current1->CBasicPoint.ptID_s == lost_m1[j])
						{
							Lost_Ptss.insertLast(current1->CBasicPoint);
							break;
						}
						current1 = current1->link;
					}
				}
				//丢失点的匹配
				k = 0;
				Angle = new float[P2.count];
				current1 = Lost_Ptss.front();
				while (current1 != NULL)
				{
					j = 0;
					current2 = P2.front();
					while (current2 != NULL)
					{
						Angle[j] = Ray_Firing(current1->CBasicPoint, center,
							current2->CBasicPoint);
						j++;
						current2 = current2->link;
					}
					MinA = Angle[0];
					lost_m2[k] = 0;
					for (j = 1; j < P2.count; j++)
						if (MinA > Angle[j])
						{
							MinA = Angle[j];
							lost_m2[k] = j;
						}
					k++;
					current1 = current1->link;
				}
				//for(j=0;j<lostN[i];j++)
				//	cout<<lost_m1[j]<<","<<lost_m2[j]<<endl; //test

				p = 0;
				for (j = 0; j < lostN[i]; j++)
				{
					found = false;
					for (k = 0; k < Ptss_2.count + p; k++)
						if (mult_m1[k] > lost_m1[j])
						{
							for (q = Ptss_2.count + p; q > k; q--)
							{
								mult_m1[q] = mult_m1[q - 1];
								mult_m2[q] = mult_m2[q - 1];
							}
							mult_m1[k] = lost_m1[j];
							mult_m2[k] = lost_m2[j];
							p++;
							found = true;
							break;
						}
					if (!found)
					{
						mult_m2[Ptss_2.count + p] = lost_m2[j];
						mult_m1[Ptss_2.count + p] = lost_m1[j];
						p++;
					}
				}
				//for(j=0;j<Ptss_2.count+lostN[i];j++)
				//	cout<<mult_m1[j]<<","<<mult_m2[j]<<endl;
			}//end if
			else                       //如果不存在丢失点
			{
				for (j = 0; j < Ptss_2.count; j++)
				{
					mult_m1[j] = sing_m1[j];
					mult_m2[j] = sing_m2[j];
				}
			}

			//重匹配的赋值(凸包)
			for (j = 0; j < Ptss_2.count + lostN[i]; j++)
			{
				current1 = P1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == mult_m1[j])
					{
						Mult_Pts1.insertLast(current1->CBasicPoint);
						break;
					}
					current1 = current1->link;
				}
			}
			for (j = 0; j < Ptss_2.count + lostN[i]; j++)
			{
				current2 = P2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == mult_m2[j])
					{
						Mult_Pts2.insertLast(current2->CBasicPoint);
						break;
					}
					current2 = current2->link;
				}
			}
			//重匹配的赋值(原始)
			for (j = 0; j < Ptss_2.count + lostN[i]; j++)
			{
				current1 = Ptss1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == mult_m1[j])
					{
						MultPts1.insertLast(current1->CBasicPoint);
						break;
					}
					current1 = current1->link;
				}
			}
			for (j = 0; j < Ptss_2.count + lostN[i]; j++)
			{
				current2 = Ptss2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == mult_m2[j])
					{
						MultPts2.insertLast(current2->CBasicPoint);
						break;
					}
					current2 = current2->link;
				}
			}
			break;

		}//end switch
	}//end for
	P1.destroyList();
	P2.destroyList();
	Ptss_1.destroyList();
	Ptss_2.destroyList();
	Ptss1.destroyList();
	Ptss2.destroyList();//成功，Happy一下下...
}

///////////////////////////////////////////////////////////////////////////////
//等比例映射
void CJsGrCorr::Fina_Corr(void)
{
	int i, j, k, p;
	nodeType *current1, *current2;
	linkedListType P1, P2;              //原始数据
	FPoint ptNode;

	int *mult_m1, *mult_m2;             //重匹配对应的ptID_s号
	int *fina_m1, *fina_m2;             //等比例映射对应的ptID_s号
	int *time;                         //顶点匹配次数(多顶点匹配)
	float Angle, MinA;
	float D, sumD;
	float ratio;
	nodeType *tempF, *tempB, *current;
	int *sing_m1, *sing_m2;             //单匹配对应的ptID_s号
	bool *flag;                        //丢失点的标记
	int lostN;                         //每层切片上丢失点的个数
	int *lost_mark;                    //丢失点的ptID_s号
	FPoint center;

	FinaPts1.initList();
	FinaPts2.initList();
	center.x = 0.0;
	center.y = 0.0;
	center.z = 0.0;

	for (i = 0; i < ppro.FinalSect; i++)
	{
		P1.initList();
		P2.initList();
		Ptss_1.initList();
		Ptss_2.initList();
		Ptss1.initList();
		Ptss2.initList();

		//初始化后的切片数据赋值(原始)
		current1 = InitPts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				P1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = InitPts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				P2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//重匹配后的切片数据赋值(凸包)
		current1 = Mult_Pts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss_1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = Mult_Pts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss_2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}

		//单顶点匹配后的切片数据赋值(凸包)
		current1 = Sing_Pts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				Ptss1.insertLast(current1->CBasicPoint);
			current1 = current1->link;
		}
		current2 = Sing_Pts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				Ptss2.insertLast(current2->CBasicPoint);
			current2 = current2->link;
		}
		//切片数据赋值Pts1/Pts2指针数组(原始)
		Pts1 = new float[Ptss_1.count][3];
		j = 0;
		current1 = MultPts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
			{
				Pts1[j][0] = current1->CBasicPoint.x;
				Pts1[j][1] = current1->CBasicPoint.y;
				Pts1[j][2] = current1->CBasicPoint.z;
				j++;
			}
			current1 = current1->link;
		}
		Pts2 = new float[Ptss_2.count][3];
		j = 0;
		current2 = MultPts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
			{
				Pts2[j][0] = current2->CBasicPoint.x;
				Pts2[j][1] = current2->CBasicPoint.y;
				Pts2[j][2] = current2->CBasicPoint.z;
				j++;
			}
			current2 = current2->link;
		}

		//根据顶点数量大小的标记，建立重匹配点的等比例映射
		switch (numFlag[i])
		{
		case 0:
			//多顶点匹配时产生的重匹配
			//-------------------------->start here!
						//mult_m1/mult_m2的赋值
			mult_m1 = new int[Ptss_1.count];
			j = 0;
			current1 = Ptss_1.front();
			while (current1 != NULL)
			{
				mult_m1[j] = current1->CBasicPoint.ptID_s;
				j++;
				current1 = current1->link;
			}
			mult_m2 = new int[Ptss_2.count];
			j = 0;
			current2 = Ptss_2.front();
			while (current2 != NULL)
			{
				mult_m2[j] = current2->CBasicPoint.ptID_s;
				j++;
				current2 = current2->link;
			}
			//for(j=0;j<Ptss_1.count;j++)
			//    cout<<mult_m1[j]<<","<<mult_m2[j]<<endl; //test

			//初始化
			fina_m1 = new int[P2.count];
			fina_m2 = new int[P2.count];
			time = new int[P2.count];
			for (j = 0; j < P2.count; j++)
			{
				fina_m2[j] = j;
				time[j] = 0;
			}

			//计算多顶点匹配时产生的重匹配点
			for (j = 0; j < P2.count; j++)
				for (k = 0; k < Ptss_1.count; k++)
					if (mult_m2[k] == j)
						time[j]++;

			for (j = 0; j < P2.count; j++)
			{
				if (time[j] > 1)
				{
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m2[k] == j)
						{
							current1 = Ptss_1.front();
							while (current1 != NULL)
							{
								if (current1->CBasicPoint.ptID_s == mult_m1[k])
									break;
								current1 = current1->link;
							}
							current2 = Ptss_2.front();
							while (current2 != NULL)
							{
								if (current2->CBasicPoint.ptID_s == mult_m2[k])
									break;
								current2 = current2->link;
							}
							MinA = Ray_Firing(current1->CBasicPoint, center,
								current2->CBasicPoint);
							fina_m1[j] = mult_m1[k];
							break;     //将第一点的值赋给MinD/fina_m1[j]
						}
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m2[k] == j && mult_m1[k] != fina_m1[j])
						{
							current1 = Ptss_1.front();
							while (current1 != NULL)
							{
								if (current1->CBasicPoint.ptID_s == mult_m1[k])
									break;
								current1 = current1->link;
							}
							current2 = Ptss_2.front();
							while (current2 != NULL)
							{
								if (current2->CBasicPoint.ptID_s == mult_m2[k])
									break;
								current2 = current2->link;
							}
							Angle = Ray_Firing(current1->CBasicPoint, center,
								current2->CBasicPoint);
							if (MinA > Angle)
							{
								MinA = Angle;
								fina_m1[j] = mult_m1[k];
							}
						}
				}
				else
				{
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m2[k] == j)
							fina_m1[j] = mult_m1[k];
				}//end if-else
			}//end for
			//for(j=0;j<P2.count;j++)
			//	cout<<fina_m1[j]<<","<<fina_m2[j]<<endl;

			//等比例映射
			for (j = 0; j < P2.count; j++)
				if (time[j] > 1)
				{
					//cout<<j<<endl;   //test:重匹配点的序号
					if (j == 0)//如果为起点
					{
						for (k = 0; k < Ptss_1.count; k++)
							if (mult_m2[k] == j && mult_m1[k] != fina_m1[j])
							{
								if (mult_m1[k] > fina_m1[j] && mult_m1[k] < fina_m1[j + 1])
								{
									//寻找前后两点
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j + 1])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == mult_m1[k])
										{
											current = current1;
											break;
										}
										current1 = current1->link;
									}

									sumD = 0;
									current1 = tempF;
									while (current1 != tempB)
									{
										sumD += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}
									D = 0;
									current1 = tempF;
									while (current1 != current)
									{
										D += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}
									ratio = D / sumD;
									//cout<<ratio<<endl;

									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j + 1])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									Pts2[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts2[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts2[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}
							}
					}
					else if (j == P2.count - 1)//如果为终点
					{
						for (k = 0; k < Ptss_1.count; k++)
							if (mult_m2[k] == j && mult_m1[k] != fina_m1[j])
							{
								if (mult_m1[k]<fina_m1[j] && mult_m1[k]>fina_m1[j - 1])
								{
									//寻找前后两点
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j - 1])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == mult_m1[k])
										{
											current = current1;
											break;
										}
										current1 = current1->link;
									}

									sumD = 0;
									current1 = tempF;
									while (current1 != tempB)
									{
										sumD += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}
									D = 0;
									current1 = tempF;
									while (current1 != current)
									{
										D += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}
									ratio = D / sumD;
									//cout<<ratio<<endl;

									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j - 1])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									Pts2[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts2[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts2[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}
							}
					}
					else
					{
						for (k = 0; k < Ptss_1.count; k++)
							if (mult_m2[k] == j && mult_m1[k] != fina_m1[j])
							{
								if (mult_m1[k] < fina_m1[j])
								{
									//寻找前后两点
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j - 1])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == mult_m1[k])
										{
											current = current1;
											break;
										}
										current1 = current1->link;
									}

									sumD = 0;
									current1 = tempF;
									while (current1 != tempB)
									{
										sumD += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}
									D = 0;
									current1 = tempF;
									while (current1 != current)
									{
										D += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}

									ratio = D / sumD;
									//cout<<ratio<<endl;

									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j - 1])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									Pts2[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts2[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts2[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}
								else
								{
									//寻找前后两点
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j + 1])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == mult_m1[k])
										{
											current = current1;
											break;
										}
										current1 = current1->link;
									}

									sumD = 0;
									current1 = tempF;
									while (current1 != tempB)
									{
										sumD += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}
									D = 0;
									current1 = tempF;
									while (current1 != current)
									{
										D += Dist_List(current1->CBasicPoint,
											current1->link->CBasicPoint);
										current1 = current1->link;
									}

									ratio = D / sumD;
									//cout<<ratio<<endl;

									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j + 1])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									Pts2[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts2[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts2[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}//end else
							}//end if
					}//end else
				}//end if(time[j]>1)
//<--------------------------end here!

//丢失点匹配时产生的重匹配
//-------------------------->start here!
			//sing_m1/sing_m2的重新赋值
			sing_m1 = new int[Ptss1.count];
			for (j = 0; j < Ptss1.count; j++)
				sing_m1[j] = j;

			sing_m2 = new int[Ptss1.count];
			j = 0;
			current2 = Ptss2.front();
			while (current2 != NULL)
			{
				sing_m2[j] = current2->CBasicPoint.ptID_s;
				j++;
				current2 = current2->link;
			}
			//for(j=0;j<Ptss1.count;j++)
			//    cout<<sing_m1[j]<<","<<sing_m2[j]<<endl; //test

			//lostN的计算
			flag = new bool[P2.count];
			for (j = 0; j < P2.count; j++)
				flag[j] = 0;

			for (j = 0; j < P2.count; j++)
			{
				current2 = Ptss2.front();
				while (current2 != NULL)
				{
					if (current2->CBasicPoint.ptID_s == j)
					{
						flag[j] = 1;
						break;
					}
					current2 = current2->link;
				}
			}
			lostN = 0;
			for (j = 0; j < P2.count; j++)
				if (flag[j] == 0)
					lostN++;
			//cout<<"丢失点的数量为"<<lostN<<endl;        //test

			if (lostN > 0)
			{
				lost_mark = new int[lostN];
				k = 0;
				for (j = 0; j < P2.count; j++)
					if (flag[j] == 0)
					{
						lost_mark[k] = j;
						k++;
					}
				for (j = 0; j < lostN; j++)
				{
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m2[k] == lost_mark[j])
						{
							//cout<<mult_m2[k]<<endl;
							for (p = 0; p < Ptss1.count; p++)
								if (sing_m2[p] > lost_mark[j])
								{
									if (p != 0)
									{
										//寻找前后两点
										current2 = P2.front();
										while (current2 != NULL)
										{
											if (current2->CBasicPoint.ptID_s == sing_m2[p - 1])
											{
												tempF = current2;
												break;
											}
											current2 = current2->link;
										}
										current2 = P2.front();
										while (current2 != NULL)
										{
											if (current2->CBasicPoint.ptID_s == sing_m2[p])
											{
												tempB = current2;
												break;
											}
											current2 = current2->link;
										}
										current2 = P2.front();
										while (current2 != NULL)
										{
											if (current2->CBasicPoint.ptID_s == lost_mark[j])
											{
												current = current2;
												break;
											}
											current2 = current2->link;
										}

										sumD = 0;
										current2 = tempF;
										while (current2 != tempB)
										{
											sumD += Dist_List(current2->CBasicPoint,
												current2->link->CBasicPoint);
											current2 = current2->link;
										}
										D = 0;
										current2 = tempF;
										while (current2 != current)
										{
											D += Dist_List(current2->CBasicPoint,
												current2->link->CBasicPoint);
											current2 = current2->link;
										}
										ratio = D / sumD;
										//cout<<ratio<<endl;

										current1 = P1.front();
										while (current1 != NULL)
										{
											if (current1->CBasicPoint.ptID_s == sing_m1[p - 1])
											{
												tempF = current1;
												break;
											}
											current1 = current1->link;
										}
										current1 = P1.front();
										while (current1 != NULL)
										{
											if (current1->CBasicPoint.ptID_s == sing_m1[p])
											{
												tempB = current1;
												break;
											}
											current1 = current1->link;
										}
										Pts1[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
											ratio * tempB->CBasicPoint.x;
										Pts1[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
											ratio * tempB->CBasicPoint.y;
										Pts1[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
											ratio * tempB->CBasicPoint.z;
									}
									break;
								}
						}
				}
			}
			//<--------------------------end here!
						//存入链表
			for (j = 0; j < Ptss_1.count; j++)
			{
				ptNode.ptID = i;
				ptNode.ptID_s = j;
				ptNode.x = Pts1[j][0];
				ptNode.y = Pts1[j][1];
				ptNode.z = Pts1[j][2];
				FinaPts1.insertLast(ptNode);
			}
			for (j = 0; j < Ptss_2.count; j++)
			{
				ptNode.ptID = i;
				ptNode.ptID_s = j;
				ptNode.x = Pts2[j][0];
				ptNode.y = Pts2[j][1];
				ptNode.z = Pts2[j][2];
				FinaPts2.insertLast(ptNode);
			}
			break;

		case 1:
			//多顶点匹配时产生的重匹配
//-------------------------->start here!
			//mult_m1/mult_m2的赋值
			mult_m1 = new int[Ptss_1.count];
			j = 0;
			current1 = Ptss_1.front();
			while (current1 != NULL)
			{
				mult_m1[j] = current1->CBasicPoint.ptID_s;
				j++;
				current1 = current1->link;
			}
			mult_m2 = new int[Ptss_2.count];
			j = 0;
			current2 = Ptss_2.front();
			while (current2 != NULL)
			{
				mult_m2[j] = current2->CBasicPoint.ptID_s;
				j++;
				current2 = current2->link;
			}
			//for(j=0;j<Ptss_1.count;j++)
			//    cout<<mult_m1[j]<<","<<mult_m2[j]<<endl; //test

			//初始化
			fina_m1 = new int[P1.count];
			fina_m2 = new int[P1.count];
			time = new int[P1.count];
			for (j = 0; j < P1.count; j++)
			{
				fina_m1[j] = j;
				time[j] = 0;
			}

			//计算多顶点匹配时产生的重匹配点
			for (j = 0; j < P1.count; j++)
				for (k = 0; k < Ptss_1.count; k++)
					if (mult_m1[k] == j)
						time[j]++;

			for (j = 0; j < P1.count; j++)
			{
				if (time[j] > 1)
				{
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m1[k] == j)
						{
							current1 = Ptss_1.front();
							while (current1 != NULL)
							{
								if (current1->CBasicPoint.ptID_s == mult_m1[k])
									break;
								current1 = current1->link;
							}
							current2 = Ptss_2.front();
							while (current2 != NULL)
							{
								if (current2->CBasicPoint.ptID_s == mult_m2[k])
									break;
								current2 = current2->link;
							}
							MinA = Ray_Firing(current1->CBasicPoint, center,
								current2->CBasicPoint);
							fina_m2[j] = mult_m2[k];
							break;     //将第一点的值赋给MinD/fina_m2[j]
						}
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m1[k] == j && mult_m2[k] != fina_m2[j])
						{
							current1 = Ptss_1.front();
							while (current1 != NULL)
							{
								if (current1->CBasicPoint.ptID_s == mult_m1[k])
									break;
								current1 = current1->link;
							}
							current2 = Ptss_2.front();
							while (current2 != NULL)
							{
								if (current2->CBasicPoint.ptID_s == mult_m2[k])
									break;
								current2 = current2->link;
							}
							Angle = Ray_Firing(current1->CBasicPoint, center,
								current2->CBasicPoint);
							if (MinA > Angle)
							{
								MinA = Angle;
								fina_m2[j] = mult_m2[k];
							}
						}
				}
				else
				{
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m1[k] == j)
							fina_m2[j] = mult_m2[k];
				}//end if-else
			}//end for
			//for(j=0;j<P1.count;j++)
			//	cout<<fina_m1[j]<<","<<fina_m2[j]<<endl;

			//等比例映射
			for (j = 0; j < P1.count; j++)
				if (time[j] > 1)
				{
					//cout<<j<<endl;                       //test
					if (j == 0)//如果为起点
					{
						for (k = 0; k < Ptss_1.count; k++)
							if (mult_m1[k] == j && mult_m2[k] != fina_m2[j])
							{
								if (mult_m2[k] > fina_m2[j] && mult_m2[k] < fina_m2[j + 1])
								{
									//寻找前后两点
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j + 1])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == mult_m2[k])
										{
											current = current2;
											break;
										}
										current2 = current2->link;
									}

									sumD = 0;
									current2 = tempF;
									while (current2 != tempB)
									{
										sumD += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									D = 0;
									current2 = tempF;
									while (current2 != current)
									{
										D += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									ratio = D / sumD;
									//cout<<ratio<<endl;

									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j + 1])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									Pts1[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts1[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts1[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}
							}
					}
					else if (j == P1.count - 1)//如果为终点
					{
						for (k = 0; k < Ptss_1.count; k++)
							if (mult_m1[k] == j && mult_m2[k] != fina_m2[j])
							{
								if (mult_m2[k]<fina_m2[j] && mult_m2[k]>fina_m2[j - 1])
								{
									//寻找前后两点
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j - 1])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == mult_m2[k])
										{
											current = current2;
											break;
										}
										current2 = current2->link;
									}

									sumD = 0;
									current2 = tempF;
									while (current2 != tempB)
									{
										sumD += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									D = 0;
									current2 = tempF;
									while (current2 != current)
									{
										D += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									ratio = D / sumD;
									//cout<<ratio<<endl;

									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j - 1])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									Pts1[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts1[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts1[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}
							}
					}
					else
					{
						for (k = 0; k < Ptss_1.count; k++)
							if (mult_m1[k] == j && mult_m2[k] != fina_m2[j])
							{
								if (mult_m2[k] < fina_m2[j])
								{
									//寻找前后两点
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j - 1])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == mult_m2[k])
										{
											current = current2;
											break;
										}
										current2 = current2->link;
									}

									sumD = 0;
									current2 = tempF;
									while (current2 != tempB)
									{
										sumD += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									D = 0;
									current2 = tempF;
									while (current2 != current)
									{
										D += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									ratio = D / sumD;
									//cout<<ratio<<endl;

									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j - 1])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									Pts1[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts1[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts1[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}
								else
								{
									//寻找前后两点
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j])
										{
											tempF = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == fina_m2[j + 1])
										{
											tempB = current2;
											break;
										}
										current2 = current2->link;
									}
									current2 = P2.front();
									while (current2 != NULL)
									{
										if (current2->CBasicPoint.ptID_s == mult_m2[k])
										{
											current = current2;
											break;
										}
										current2 = current2->link;
									}

									sumD = 0;
									current2 = tempF;
									while (current2 != tempB)
									{
										sumD += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									D = 0;
									current2 = tempF;
									while (current2 != current)
									{
										D += Dist_List(current2->CBasicPoint,
											current2->link->CBasicPoint);
										current2 = current2->link;
									}
									ratio = D / sumD;
									//cout<<ratio<<endl;

									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j])
										{
											tempF = current1;
											break;
										}
										current1 = current1->link;
									}
									current1 = P1.front();
									while (current1 != NULL)
									{
										if (current1->CBasicPoint.ptID_s == fina_m1[j + 1])
										{
											tempB = current1;
											break;
										}
										current1 = current1->link;
									}
									Pts1[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
										ratio * tempB->CBasicPoint.x;
									Pts1[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
										ratio * tempB->CBasicPoint.y;
									Pts1[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
										ratio * tempB->CBasicPoint.z;
								}//end else
							}//end if
					}//end else
				}//end if(time[j]>1)
//<--------------------------end here!

//丢失点匹配时产生的重匹配
//-------------------------->start here!
			//sing_m1/sing_m2的重新赋值
			sing_m2 = new int[Ptss2.count];
			for (j = 0; j < Ptss2.count; j++)
				sing_m2[j] = j;

			sing_m1 = new int[Ptss2.count];
			j = 0;
			current1 = Ptss1.front();
			while (current1 != NULL)
			{
				sing_m1[j] = current1->CBasicPoint.ptID_s;
				j++;
				current1 = current1->link;
			}
			//for(j=0;j<Ptss2.count;j++)
			//    cout<<sing_m1[j]<<","<<sing_m2[j]<<endl; //test

			//lostN的计算
			flag = new bool[P1.count];
			for (j = 0; j < P1.count; j++)
				flag[j] = 0;

			for (j = 0; j < P1.count; j++)
			{
				current1 = Ptss1.front();
				while (current1 != NULL)
				{
					if (current1->CBasicPoint.ptID_s == j)
					{
						flag[j] = 1;
						break;
					}
					current1 = current1->link;
				}
			}
			lostN = 0;
			for (j = 0; j < P1.count; j++)
				if (flag[j] == 0)
					lostN++;
			//cout<<"丢失点的数量为"<<lostN<<endl;        //test

			if (lostN > 0)
			{
				lost_mark = new int[lostN];
				k = 0;
				for (j = 0; j < P1.count; j++)
					if (flag[j] == 0)
					{
						lost_mark[k] = j;
						k++;
					}
				for (j = 0; j < lostN; j++)
				{
					for (k = 0; k < Ptss_1.count; k++)
						if (mult_m1[k] == lost_mark[j])
						{
							//cout<<mult_m1[k]<<endl;
							for (p = 0; p < Ptss2.count; p++)
								if (sing_m1[p] > lost_mark[j])
								{
									if (p != 0)
									{
										//寻找前后两点
										current1 = P1.front();
										while (current1 != NULL)
										{
											if (current1->CBasicPoint.ptID_s == sing_m1[p - 1])
											{
												tempF = current1;
												break;
											}
											current1 = current1->link;
										}
										current1 = P1.front();
										while (current1 != NULL)
										{
											if (current1->CBasicPoint.ptID_s == sing_m1[p])
											{
												tempB = current1;
												break;
											}
											current1 = current1->link;
										}
										current1 = P1.front();
										while (current1 != NULL)
										{
											if (current1->CBasicPoint.ptID_s == lost_mark[j])
											{
												current = current1;
												break;
											}
											current1 = current1->link;
										}

										sumD = 0;
										current1 = tempF;
										while (current1 != tempB)
										{
											sumD += Dist_List(current1->CBasicPoint,
												current1->link->CBasicPoint);
											current1 = current1->link;
										}
										D = 0;
										current1 = tempF;
										while (current1 != current)
										{
											D += Dist_List(current1->CBasicPoint,
												current1->link->CBasicPoint);
											current1 = current1->link;
										}
										ratio = D / sumD;
										//cout<<ratio<<endl;

										current2 = P2.front();
										while (current2 != NULL)
										{
											if (current2->CBasicPoint.ptID_s == sing_m2[p - 1])
											{
												tempF = current2;
												break;
											}
											current2 = current2->link;
										}
										current2 = P2.front();
										while (current2 != NULL)
										{
											if (current2->CBasicPoint.ptID_s == sing_m2[p])
											{
												tempB = current2;
												break;
											}
											current2 = current2->link;
										}
										Pts2[k][0] = (1 - ratio)*tempF->CBasicPoint.x +
											ratio * tempB->CBasicPoint.x;
										Pts2[k][1] = (1 - ratio)*tempF->CBasicPoint.y +
											ratio * tempB->CBasicPoint.y;
										Pts2[k][2] = (1 - ratio)*tempF->CBasicPoint.z +
											ratio * tempB->CBasicPoint.z;
									}
									break;
								}
						}
				}
			}
			//<--------------------------end here!
						//存入链表
			for (j = 0; j < Ptss_1.count; j++)
			{
				ptNode.ptID = i;
				ptNode.ptID_s = j;
				ptNode.x = Pts1[j][0];
				ptNode.y = Pts1[j][1];
				ptNode.z = Pts1[j][2];
				FinaPts1.insertLast(ptNode);
			}
			for (j = 0; j < Ptss_2.count; j++)
			{
				ptNode.ptID = i;
				ptNode.ptID_s = j;
				ptNode.x = Pts2[j][0];
				ptNode.y = Pts2[j][1];
				ptNode.z = Pts2[j][2];
				FinaPts2.insertLast(ptNode);
			}
			break;
		}//end switch
	}//end for
}

///////////////////////////////////////////////////////////////////////////////
void CJsGrCorr::Display(void)
{
	nodeType *current;
	nodeType *current1, *current2;
	int i;
	glPointSize(4.0);

	//O点
	glBegin(GL_POINTS);
	glVertex3f(0.0, 0.0, 0.0);
	glEnd();

	int output_s = 12;
	/*	//初始化(凸包)
		glColor3f(1.0,0.0,0.0);
		for(i=output_s;i<output_s+1;i++)
		{
			current=Init_Pts1.front();
			glBegin(GL_POINTS);
			while(current!=NULL)
			{
				if(current->CBasicPoint.ptID==i)
					glVertex3f(current->CBasicPoint.x,
							   current->CBasicPoint.y,
							   current->CBasicPoint.z);
				current=current->link;
			}
			glEnd();
		}
		glColor3f(0.0,0.0,1.0);
		for(i=output_s;i<output_s+1;i++)
		{
			current=Init_Pts2.front();
			glBegin(GL_POINTS);
			while(current!=NULL)
			{
				if(current->CBasicPoint.ptID==i)
					glVertex3f(current->CBasicPoint.x,
							   current->CBasicPoint.y,
							   current->CBasicPoint.z);
				current=current->link;
			}
			glEnd();
		}//*/
		//*	//初始化(原始数据)
	glColor3f(1.0, 0.0, 0.0);
	for (i = output_s; i < output_s + 1; i++)
	{
		current = InitPts1.front();
		glBegin(GL_POINTS);
		while (current != NULL)
		{
			if (current->CBasicPoint.ptID == i)
				glVertex3f(current->CBasicPoint.x,
					current->CBasicPoint.y,
					current->CBasicPoint.z);
			current = current->link;
		}
		glEnd();
	}
	glColor3f(0.0, 0.0, 1.0);
	for (i = output_s; i < output_s + 1; i++)
	{
		current = InitPts2.front();
		glBegin(GL_POINTS);
		while (current != NULL)
		{
			if (current->CBasicPoint.ptID == i)
				glVertex3f(current->CBasicPoint.x,
					current->CBasicPoint.y,
					current->CBasicPoint.z);
			current = current->link;
		}
		glEnd();
	}//*/

/*	//单匹配(凸包)
	glColor3f(0.7,0.7,0.7);
	glBegin(GL_LINES);
	for(i=output_s;i<output_s+1;i++)
	{
		current1=Sing_Pts1.front();
		while(current1!=NULL)
		{
			if(current1->CBasicPoint.ptID==i)
				break;
			current1=current1->link;
		}
		current2=Sing_Pts2.front();
		while(current2!=NULL)
		{
			if(current2->CBasicPoint.ptID==i)
				break;
			current2=current2->link;
		}

		while(current1!=NULL)
		{
			if(current1->CBasicPoint.ptID==i)
			{
				glVertex3f(current1->CBasicPoint.x,
						   current1->CBasicPoint.y,
						   current1->CBasicPoint.z);
				glVertex3f(current2->CBasicPoint.x,
						   current2->CBasicPoint.y,
						   current2->CBasicPoint.z);
			}
			current1=current1->link;
			current2=current2->link;
		}
	}
	glEnd();

/*	//单匹配(原始)
	glBegin(GL_LINES);
	for(i=output_s;i<output_s+1;i++)
	{
		current1=SingPts1.front();
		while(current1!=NULL)
		{
			if(current1->CBasicPoint.ptID==i)
				break;
			current1=current1->link;
		}
		current2=SingPts2.front();
		while(current2!=NULL)
		{
			if(current2->CBasicPoint.ptID==i)
				break;
			current2=current2->link;
		}

		while(current1!=NULL)
		{
			if(current1->CBasicPoint.ptID==i)
			{
				glVertex3f(current1->CBasicPoint.x,
						   current1->CBasicPoint.y,
						   current1->CBasicPoint.z);
				glVertex3f(current2->CBasicPoint.x,
						   current2->CBasicPoint.y,
						   current2->CBasicPoint.z);
			}
			current1=current1->link;
			current2=current2->link;
		}
	}
	glEnd();*/

	/*	//重匹配(凸包)
		glColor3f(0.7,0.7,0.7);
		glBegin(GL_LINES);
		for(i=output_s;i<output_s+1;i++)
		{
			current1=Mult_Pts1.front();
			while(current1!=NULL)
			{
				if(current1->CBasicPoint.ptID==i)
					break;
				current1=current1->link;
			}
			current2=Mult_Pts2.front();
			while(current2!=NULL)
			{
				if(current2->CBasicPoint.ptID==i)
					break;
				current2=current2->link;
			}

			while(current1!=NULL)
			{
				if(current1->CBasicPoint.ptID==i)
				{
					glVertex3f(current1->CBasicPoint.x,
							   current1->CBasicPoint.y,
							   current1->CBasicPoint.z);
					glVertex3f(current2->CBasicPoint.x,
							   current2->CBasicPoint.y,
							   current2->CBasicPoint.z);
				}
				current1=current1->link;
				current2=current2->link;
			}
		}
		glEnd();//*/

		/*	//重匹配(原始)
			glColor3f(0.7,0.7,0.7);
			glBegin(GL_LINES);
			for(i=output_s;i<output_s+1;i++)
			{
				current1=MultPts1.front();
				while(current1!=NULL)
				{
					if(current1->CBasicPoint.ptID==i)
						break;
					current1=current1->link;
				}
				current2=MultPts2.front();
				while(current2!=NULL)
				{
					if(current2->CBasicPoint.ptID==i)
						break;
					current2=current2->link;
				}

				while(current1!=NULL)
				{
					if(current1->CBasicPoint.ptID==i)
					{
						glVertex3f(current1->CBasicPoint.x,
								   current1->CBasicPoint.y,
								   current1->CBasicPoint.z);
						glVertex3f(current2->CBasicPoint.x,
								   current2->CBasicPoint.y,
								   current2->CBasicPoint.z);
					}
					current1=current1->link;
					current2=current2->link;
				}
			}
			glEnd();//*/

			//*	//等比例映射(原始)
	glColor3f(0.7, 0.7, 0.7);
	glBegin(GL_LINES);
	for (i = output_s; i < output_s + 1; i++)
	{
		current1 = FinaPts1.front();
		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
				break;
			current1 = current1->link;
		}
		current2 = FinaPts2.front();
		while (current2 != NULL)
		{
			if (current2->CBasicPoint.ptID == i)
				break;
			current2 = current2->link;
		}

		while (current1 != NULL)
		{
			if (current1->CBasicPoint.ptID == i)
			{
				glVertex3f(current1->CBasicPoint.x,
					current1->CBasicPoint.y,
					current1->CBasicPoint.z);
				glVertex3f(current2->CBasicPoint.x,
					current2->CBasicPoint.y,
					current2->CBasicPoint.z);
			}
			current1 = current1->link;
			current2 = current2->link;
		}
	}
	glEnd();//*/
}

#endif //__LMZ_CORRESPONDENCE_H__
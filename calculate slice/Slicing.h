#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//����һ����Ƭ�ࣺ��Ƭ����(�Զ���)
//                ��Ƭ���(�����ܶȹ���)
//                ��Ƭ���ߵĹ���(���������)
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
//��Ƭ��
class CJsGrSlicing
{
private:
	float Min_pFlag;                   //��Ƭ�����ϵ���Сֵ
	linkedListType E, El, Er, El_corr, Er_corr;
	//����������Ķ���
	linkedListType Extreme;            //ͬ�ϣ��洢��ֵ��

public:
	float tkness;                      //��Ƭ���
	int flag;                          //��Ƭ����ı��
	linkedListType Ptsio;              //����������Ķ��󣬴洢������Ƭ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3{ new pcl::PointCloud<pcl::PointXYZ> };
	inline void Tkness(float Scale);   //��Ƭ��ȼ���
	inline void Direct(float Scale, float *XYZ_Scale);//��Ƭ������

	inline void InterS(int num, float Pts[][3], int flag, float tkness);//ƽ���������

	inline void Curves(int flag, float dens);//��Ƭ�����߹���

	inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudout(void);         //��ʾ����

};
CJsGrSlicing slice1, slice2;            //���������Ƭ��Ķ���


///////////////////////////////////////////////////////////////////////////////
//��Ƭ��ȼ���-����
inline void CJsGrSlicing::Tkness(float Scale)
{

	tkness = (2.0*Scale) / ppro.FinalSect;
	cout << endl;
	cout << "��Ƭ���Ϊ��" << tkness << endl;
}

///////////////////////////////////////////////////////////////////////////////
//��Ƭ������ʱ��X,Y,Z���������ȵ�������Ϊ��Ƭ����
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
		cout << "��Ƭ����Ϊ��X ��" << endl;
		break;
	case 1:
		cout << "��Ƭ����Ϊ��Y ��" << endl;
		break;
	case 2:
		cout << "��Ƭ����Ϊ��Z ��" << endl;
		break;
	}

}

///////////////////////////////////////////////////////////////////////////////
//ƽ���������
inline void CJsGrSlicing::InterS(int num, float Pts[][3],
	int flag, float tkness)
{
	cout << "������Ƭ���㣬��Լ��Ҫ4~8���ʱ�䣬���Ժ�..." << endl;
	//��ӡ��ʾ
//������Ƭ���㣬�������Ϊ0~ppro.FinalSect-1����Ƭ����
	E.initList();                      //��ʼ��
	float ss = 0;                        //��Ƭ����step of slicing
	int i, j;

	//������Ƭ�������Сֵ
	Min_pFlag = Pts[0][flag];
	for (i = 0; i < num - 1; i++)
		if (Min_pFlag > Pts[i + 1][flag])
			Min_pFlag = Pts[i + 1][flag];

	FPoint ptNode;
	j = 0;                               //�ӱ��Ϊ0����Ƭ��ʼ
	do
	{
		//�����ʼ��
		El.initList();
		Er.initList();
		El_corr.initList();
		Er_corr.initList();

		//�������ݷֲ㣬ptID��ֵΪ��Ƭ�ı��
		for (i = 0; i < num; i++)
		{
			//��λ����Ƭƽ����ߵĵ㼯�洢��El������
			if (Pts[i][flag] >= (Min_pFlag + ss) &&
				Pts[i][flag] < (Min_pFlag + ss + 0.5*tkness))
			{
				ptNode.ptID = j;
				ptNode.x = Pts[i][0];
				ptNode.y = Pts[i][1];
				ptNode.z = Pts[i][2];
				El.insertNode(ptNode);
			}
			//��λ����Ƭƽ��ĵ㼯�洢��E������
			else if (Pts[i][flag] == (Min_pFlag + ss + 0.5*tkness))
			{
				ptNode.ptID = j;
				ptNode.x = Pts[i][0];
				ptNode.y = Pts[i][1];
				ptNode.z = Pts[i][2];
				E.insertNode(ptNode);
			}
			//��λ����Ƭƽ���ұߵĵ㼯�洢��Er������
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

		//��El�е�ÿһ�㣬������Er�е�����㣬��������������El�е������
		nodeType *curr_l = El.front();
		nodeType *curr_lt = El.front();  //���Բ��������Ƿ�Ϊƥ��Ե�ָ��
		nodeType *curr_r = Er.front();

		while (curr_l != NULL)
		{
			//El�еĵ㣬����Er�е������
			//------------------------>Start here!
						//����Er��λ�ڿռ��������ڵĵ������
			float sc = 0;                //�����岽��step of cube
			int num_in = 0;
			while (num_in == 0)
			{
				sc += 0.5*tkness;        //���ò���Ϊ0.5*tkness
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

			//�����������ڵ������
			float minD, D;                //��С����ֵ������ֵ
			FPoint ptNode_r;             //�洢Er����������Ϣ

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
					break;             //����һ�����������ĵ㸳��minD��ptNode_r
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
						//��������Ϣ
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

//��������������El�е������
//------------------------>Start here!
			sc = 0;
			num_in = 0;
			while (num_in == 0)
			{
				sc += 0.5*tkness;        //���ò���Ϊ0.5*tkness
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

			FPoint ptNode_l;           //�洢El����������Ϣ
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
					break;             //����һ�����������ĵ㸳��minD��ptNode_l
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
						//��������Ϣ
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
				//����ƥ����
				El_corr.insertNode(ptNode_l);
				Er_corr.insertNode(ptNode_r);
			}
			curr_l = curr_l->link;
		}//end while

//�����㣺��֪��ƽ���ֱ�ߵĿռ�λ�ã���ֱ�ߵĽ���
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
//��Ƭ�����߹���

inline void CJsGrSlicing::Curves(int flag, float dens)
{
	int i, j;
	linkedListType Ptss;               //Points in each section
	linkedListType Ptsso;              //Points in each section in order

	int temp;
	int sFlag;                         //����Ƭ���ݷ���ı��
									   //-1��˳ʱ��; 1����ʱ��; 0�����㹲��
	int sFlag1;
	float max_ps[2];                   //����Ƭ�����Ϸ���Ƭ����ļ���ֵ
	float min_ps[2];                   //����Ƭ�����Ϸ���Ƭ����ļ�Сֵ
	int mark[4];                       //�洢�ĸ���ֵ�����

	FPoint ptsNode;
	nodeType *current, *current1, *currents;
	Ptsio.initList();

	for (i = 0; i < ppro.FinalSect; i++)
	{
		Ptss.initList();
		Ptsso.initList();

		current = E.front();             //��Ϊ�����ͷ�ڵ�

		//��ÿ�����ݴ洢������Ptss��
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

		//�����������������Ƭ������
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

		//�����������˳�����Ϊ��ʱ��
		//-------------------------->start here!
				//�ڵ��Ÿ�ֵ
		current = Ptsso.front();
		j = 0;
		while (current != NULL)
		{
			current->CBasicPoint.ptID = j;
			j++;
			current = current->link;
		}

		//������Ƭ������������ݵļ���Сֵ
		Extreme.initList();
		switch (flag)
		{
			//��X�᷽����Ƭ
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

			//����ֵ�㰴ptID˳��д������Extreme��
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

			//�����ظ��㣬��˳��ɾ��ǰһ��
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

			//��ʸ������ж������߷��򣬷������������
			//�ĸ���ֵ�����ⲻ�ظ�
			sFlag = 1;                   //��ʼ��sFlag
			if (Extreme.count == 4)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//�ĸ���ֵ����һ���ظ�
			else if (Extreme.count == 3)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//�ĸ���ֵ���������ظ���ȡ����һ�����������
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
						//�����ǰ�㲻��ͷ�ڵ��β�ڵ�
						if (current->head != NULL && current->link != NULL)
						{
							Extreme.insertNode(current->head->CBasicPoint);
							Extreme.insertNode(current->CBasicPoint);
							Extreme.insertNode(current->link->CBasicPoint);
						}
						//�����ǰ����ͷ�ڵ�
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
			//cout<<sFlag<<endl;       //test����������߷�����

			float Dse;                 //���˵�ľ���
			switch (sFlag)
			{
				//��Ϊ˳ʱ�룬��β�ڵ㿪ʼ��¼
			case -1:
				//�ж������ߵķ����
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

				//��Ϊ��ʱ�룬��ͷ�ڵ㿪ʼ��¼
			case 1:
				//�ж������ߵķ����
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
				//�����㹲�ߣ��������
			case 0:
				cout << "Error!" << endl;
				break;
			}

			break;
			//��Y�᷽����Ƭ
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

			//����ֵ�㰴ptID˳��д������Extreme��
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

			//�����ظ��㣬��˳��ɾ��ǰһ��
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

			//��ʸ������ж������߷��򣬷������������
			//�ĸ���ֵ�����ⲻ�ظ�
			sFlag = 1;                   //��ʼ��sFlag
			if (Extreme.count == 4)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//�ĸ���ֵ����һ���ظ�
			else if (Extreme.count == 3)
			{
				current = Extreme.front();
				sFlag = Cal_Direction(current->CBasicPoint,
					current->link->CBasicPoint,
					current->link->link->CBasicPoint, flag);
			}
			//�ĸ���ֵ���������ظ���ȡ����һ�����������
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
						//�����ǰ�㲻��ͷ�ڵ��β�ڵ�
						if (current->head != NULL && current->link != NULL)
						{
							Extreme.insertNode(current->head->CBasicPoint);
							Extreme.insertNode(current->CBasicPoint);
							Extreme.insertNode(current->link->CBasicPoint);
						}
						//�����ǰ����ͷ�ڵ�
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
			//cout<<sFlag<<endl;       //test����������߷�����

			float Dse1;                 //���˵�ľ���
			switch (sFlag)
			{
				//��Ϊ˳ʱ�룬��β�ڵ㿪ʼ��¼
			case -1:
				//�ж������ߵķ����
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

				//��Ϊ��ʱ�룬��ͷ�ڵ㿪ʼ��¼
			case 1:
				//�ж������ߵķ����
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
				//�����㹲�ߣ��������
			case 0:
				cout << "Error!" << endl;
				break;
			}
			break;

			//��Z�᷽����Ƭ
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

			//����ֵ�㰴ptID˳��д������Extreme��
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

			//�����ظ��㣬��˳��ɾ��ǰһ��
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

			//�жϵ�ǰ����ķ���
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
			float Dse2;                 //���˵�ľ���
			int F;
			F = sFlag * sFlag1;
			switch (F)
			{
				//��Ϊ˳ʱ�룬��β�ڵ㿪ʼ��¼
			case -1:
				//�ж������ߵķ����
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

				//��Ϊ��ʱ�룬��ͷ�ڵ㿪ʼ��¼
			case 1:
				//�ж������ߵķ����
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
				//�����㹲�ߣ��������
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
//���Ƶ�����Ƭ�ĺ���

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
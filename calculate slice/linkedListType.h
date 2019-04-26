#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//����һ������������(����Ϊģ����)
//Thanks to TangLei                               
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_LINKEDLISTTYPE_H__
#define __LMZ_LINKEDLISTTYPE_H__

#include "global.h"
#include <assert.h>
using namespace std;
/////////////////////////////////////////////////////////////////////////////// 
class linkedListType
{
public:
	linkedListType();                            //Ĭ�ϵĹ��캯��     
	virtual ~linkedListType();                   //��������

public:
	int count;                                   //������Ԫ������
	void initList();                             //��ʼ������
	void destroyList();                          //��������
	bool isEmptyList();                          //�ж������Ƿ�Ϊ��
	int length();                                //ȷ������ĳ���
	struct nodeType *front();                    //������һ���ڵ�
	struct nodeType *back();                     //�������һ���ڵ�
	struct nodeType *search(const FPoint& searchItem);
	//���������е�һָ����
	void insertFirst(const FPoint& newItem);     //�������в���ͷ�ڵ�
	void insertLast(const FPoint& newItem);      //�������в���β�ڵ�
	void insertNode(const FPoint& newItem);      //�������в����м�ڵ�
	void deleteNode(const FPoint& deleteItem);   //��������ɾ���ڵ�
	void printNode();                            //�����������
	void reversePrint();                         //�����������

protected:
	struct nodeType *first;                      //��һ���ڵ�
	struct nodeType *last;                       //���һ���ڵ�

};

///////////////////////////////////////////////////////////////////////////////
//���캯��                                                            
linkedListType::linkedListType()
{
	first = NULL;
	last = NULL;
	count = 0;
}

///////////////////////////////////////////////////////////////////////////////
//���������������˳�������ʱ���ͷ�����ڵ���ռ�õĿռ�
linkedListType::~linkedListType()
{
	destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//��ʼ������                                      
void linkedListType::initList()
{
	destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//��������                                                         
void linkedListType::destroyList()
{
	nodeType *temp;                    //������ʱָ�����ͷ��ڴ�
	while (first != NULL)
	{
		temp = first;
		first = first->link;
		delete temp;
	}
	last = NULL;
	count = 0;
}

///////////////////////////////////////////////////////////////////////////////
//�ж������Ƿ�Ϊ��  
bool linkedListType::isEmptyList()
{
	return(first == NULL);               //�����Ϊ�գ�����true�����򣬷���false
}

///////////////////////////////////////////////////////////////////////////////
//��������ĳ���
int linkedListType::length()
{
	return count;
}

///////////////////////////////////////////////////////////////////////////////
//������һ���ڵ�
struct nodeType *linkedListType::front()
{
	assert(first != NULL);               //�������Ϊ�գ�assert����������?????
	return first;
}

///////////////////////////////////////////////////////////////////////////////
//�������һ���ڵ�
struct nodeType *linkedListType::back()
{
	assert(last != NULL);
	return last;
}

///////////////////////////////////////////////////////////////////////////////
//���������е�һָ����
struct nodeType *linkedListType::search(const FPoint& searchItem)
{
	nodeType *current;
	current = first;

	//���������뵱ǰ�ڵ�Ƚϣ������ͬ�����ص�ǰ�ڵ㣻����������һ�ڵ�
	while (current != NULL)
		if (current->CBasicPoint.ptID == searchItem.ptID)
			return current;
		else
			current = current->link;
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////
//�������в���ͷ�ڵ�
void linkedListType::insertFirst(const FPoint& newItem)
{
	nodeType* newNode;
	newNode = new struct nodeType;
	assert(newNode != NULL);
	newNode->CBasicPoint.ptID = newItem.ptID;
	newNode->CBasicPoint.ptID_s = newItem.ptID_s;
	newNode->CBasicPoint.x = newItem.x;
	newNode->CBasicPoint.y = newItem.y;
	newNode->CBasicPoint.z = newItem.z;
	newNode->CBasicPoint.curv = newItem.curv;
	newNode->CBasicPoint.normal = newItem.normal;
	newNode->link = first;
	newNode->head = NULL;
	if (first == NULL)                    //�������Ϊ��
	{
		first = newNode;
		last = newNode;
		count++;
	}
	else                               //�������Ϊ��
	{
		first->head = newNode;
		first = newNode;
		count++;
	}
}

//�������в���β�ڵ�
void linkedListType::insertLast(const FPoint& newItem)
{
	nodeType* newNode;
	newNode = new struct nodeType;
	assert(newNode != NULL);
	newNode->CBasicPoint.ptID = newItem.ptID;
	newNode->CBasicPoint.ptID_s = newItem.ptID_s;
	newNode->CBasicPoint.x = newItem.x;
	newNode->CBasicPoint.y = newItem.y;
	newNode->CBasicPoint.z = newItem.z;
	newNode->CBasicPoint.curv = newItem.curv;
	newNode->CBasicPoint.normal = newItem.normal;
	newNode->head = last;
	newNode->link = NULL;
	if (first == NULL)                     //�������Ϊ��
	{
		first = newNode;
		last = newNode;
		count++;
	}
	else                                //�������Ϊ��
	{
		last->link = newNode;
		last = newNode;
		count++;
	}
}

//�������в����м�ڵ�
void linkedListType::insertNode(const FPoint& newItem)
{
	nodeType *newNode;
	nodeType *preCurrent = nullptr;
	nodeType *current;
	bool found;

	newNode = new struct nodeType;       //�����½ڵ�
	assert(newNode != NULL);

	newNode->CBasicPoint.ptID = newItem.ptID;
	newNode->CBasicPoint.ptID_s = newItem.ptID_s;
	newNode->CBasicPoint.x = newItem.x;
	newNode->CBasicPoint.y = newItem.y;
	newNode->CBasicPoint.z = newItem.z;
	newNode->CBasicPoint.curv = newItem.curv;
	newNode->CBasicPoint.normal = newItem.normal;
	newNode->link = NULL;
	newNode->head = NULL;

	if (first == NULL)                    //case1���ڿ������в���
	{
		first = newNode;
		last = newNode;
		count++;
	}
	else                               //�������Ϊ��
	{
		found = false;
		current = first;
		while (current != NULL && !found)
			if (current->CBasicPoint.ptID >= newItem.ptID)
				found = true;
			else
			{
				preCurrent = current;
				current = current->link;
			}

		if (current == first)             //case2���ڷǿձ�ı��ײ���
		{
			first->head = newNode;
			newNode->link = current;
			first = newNode;
			count++;
		}
		else
		{
			if (current != NULL)          //case3���ڷǿձ��ĳ��λ�ò���
									   //preCurrent��current֮��
			{
				preCurrent->link = newNode;
				newNode->head = preCurrent;
				newNode->link = current;
				current->head = newNode;
			}
			else                       //case4���ڷǿձ�ı�β����
			{
				last->link = newNode;
				newNode->head = preCurrent;
				last = newNode;
			}
			count++;
		}//end else
	}//end else
}//end insertNode

///////////////////////////////////////////////////////////////////////////////
//��������ɾ���ڵ�
void linkedListType::deleteNode(const FPoint& deleteItem)
{
	nodeType *current;
	nodeType *preCurrent;
	bool found;

	if (first == NULL)
		cout << "����ɾ���ձ�" << endl;    //case1����Ϊ��
	else if (first->CBasicPoint.ptID == deleteItem.ptID)
		//case2����ɾ�����Ǳ��еĵ�һ���ڵ�
	{
		current = first;
		first = first->link;
		if (first != NULL)
			first->head = NULL;
		else
			last = NULL;

		count--;
		delete current;
	}
	else
	{
		found = false;
		current = first;

		while (current != NULL && !found)
			if (current->CBasicPoint.ptID == deleteItem.ptID)
				found = true;
			else
			{
				preCurrent = current;
				current = current->link;
			}

		if (current == NULL)
			cout << "�ýڵ㲻�������С�" << endl;
		//case4����ɾ����ڱ���
		else if (current->CBasicPoint.ptID == deleteItem.ptID)
			//case3����ɾ�����ڱ��е�ĳһ��λ��
		{
			preCurrent = current->head;
			preCurrent->link = current->link;

			if (current->link != NULL)
				current->link->head = preCurrent;
			else
				last = preCurrent;

			count--;
			delete current;
		}
		else
			cout << "�ýڵ㲻��������" << endl;
		//case4����ɾ����ڱ���
	}//end else
}//end deleteNode

///////////////////////////////////////////////////////////////////////////////
//�����������
void linkedListType::printNode()
{
	nodeType* current;

	assert(first != NULL);
	current = first;
	while (current != NULL)
	{
		cout << "P[" << current->CBasicPoint.ptID << "]=(";
		cout << current->CBasicPoint.x << ",";
		cout << current->CBasicPoint.y << ",";
		cout << current->CBasicPoint.z << ")" << endl;
		current = current->link;
	}
}

///////////////////////////////////////////////////////////////////////////////
//�����������
void linkedListType::reversePrint()
{
	nodeType* current;

	assert(last != NULL);
	current = last;
	while (current != NULL)
	{
		cout << "P[" << current->CBasicPoint.ptID << "]=(";
		cout << current->CBasicPoint.x << ",";
		cout << current->CBasicPoint.y << ",";
		cout << current->CBasicPoint.z << ")" << endl;
		current = current->head;
	}
}


#endif //__LMZ_LINKEDLISTTYPE_H__

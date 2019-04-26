#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//定义一个线性链表类(可作为模板类)
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
	linkedListType();                            //默认的构造函数     
	virtual ~linkedListType();                   //析构函数

public:
	int count;                                   //链表中元素数量
	void initList();                             //初始化链表
	void destroyList();                          //销毁链表
	bool isEmptyList();                          //判断链表是否为空
	int length();                                //确定链表的长度
	struct nodeType *front();                    //检索第一个节点
	struct nodeType *back();                     //检索最后一个节点
	struct nodeType *search(const FPoint& searchItem);
	//搜索链表中的一指定项
	void insertFirst(const FPoint& newItem);     //在链表中插入头节点
	void insertLast(const FPoint& newItem);      //在链表中插入尾节点
	void insertNode(const FPoint& newItem);      //在链表中插入中间节点
	void deleteNode(const FPoint& deleteItem);   //从链表中删除节点
	void printNode();                            //正向输出链表
	void reversePrint();                         //反向输出链表

protected:
	struct nodeType *first;                      //第一个节点
	struct nodeType *last;                       //最后一个节点

};

///////////////////////////////////////////////////////////////////////////////
//构造函数                                                            
linkedListType::linkedListType()
{
	first = NULL;
	last = NULL;
	count = 0;
}

///////////////////////////////////////////////////////////////////////////////
//析构函数：当类退出作用域时，释放链表节点所占用的空间
linkedListType::~linkedListType()
{
	destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//初始化链表                                      
void linkedListType::initList()
{
	destroyList();
}

///////////////////////////////////////////////////////////////////////////////
//销毁链表                                                         
void linkedListType::destroyList()
{
	nodeType *temp;                    //定义临时指针来释放内存
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
//判断链表是否为空  
bool linkedListType::isEmptyList()
{
	return(first == NULL);               //如果表为空，返回true；否则，返回false
}

///////////////////////////////////////////////////////////////////////////////
//返回链表的长度
int linkedListType::length()
{
	return count;
}

///////////////////////////////////////////////////////////////////////////////
//检索第一个节点
struct nodeType *linkedListType::front()
{
	assert(first != NULL);               //如果链表为空，assert将结束程序?????
	return first;
}

///////////////////////////////////////////////////////////////////////////////
//检索最后一个节点
struct nodeType *linkedListType::back()
{
	assert(last != NULL);
	return last;
}

///////////////////////////////////////////////////////////////////////////////
//搜索链表中的一指定项
struct nodeType *linkedListType::search(const FPoint& searchItem)
{
	nodeType *current;
	current = first;

	//将搜索项与当前节点比较：如果相同，返回当前节点；否则，搜索下一节点
	while (current != NULL)
		if (current->CBasicPoint.ptID == searchItem.ptID)
			return current;
		else
			current = current->link;
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////
//在链表中插入头节点
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
	if (first == NULL)                    //如果链表为空
	{
		first = newNode;
		last = newNode;
		count++;
	}
	else                               //如果链表不为空
	{
		first->head = newNode;
		first = newNode;
		count++;
	}
}

//在链表中插入尾节点
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
	if (first == NULL)                     //如果链表为空
	{
		first = newNode;
		last = newNode;
		count++;
	}
	else                                //如果链表不为空
	{
		last->link = newNode;
		last = newNode;
		count++;
	}
}

//在链表中插入中间节点
void linkedListType::insertNode(const FPoint& newItem)
{
	nodeType *newNode;
	nodeType *preCurrent = nullptr;
	nodeType *current;
	bool found;

	newNode = new struct nodeType;       //创建新节点
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

	if (first == NULL)                    //case1：在空链表中插入
	{
		first = newNode;
		last = newNode;
		count++;
	}
	else                               //如果链表不为空
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

		if (current == first)             //case2：在非空表的表首插入
		{
			first->head = newNode;
			newNode->link = current;
			first = newNode;
			count++;
		}
		else
		{
			if (current != NULL)          //case3：在非空表的某个位置插入
									   //preCurrent与current之间
			{
				preCurrent->link = newNode;
				newNode->head = preCurrent;
				newNode->link = current;
				current->head = newNode;
			}
			else                       //case4：在非空表的表尾插入
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
//从链表中删除节点
void linkedListType::deleteNode(const FPoint& deleteItem)
{
	nodeType *current;
	nodeType *preCurrent;
	bool found;

	if (first == NULL)
		cout << "不能删除空表" << endl;    //case1：表为空
	else if (first->CBasicPoint.ptID == deleteItem.ptID)
		//case2：待删除项是表中的第一个节点
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
			cout << "该节点不在链表中。" << endl;
		//case4：待删除项不在表中
		else if (current->CBasicPoint.ptID == deleteItem.ptID)
			//case3：待删除项在表中的某一个位置
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
			cout << "该节点不在链表中" << endl;
		//case4：待删除项不在表中
	}//end else
}//end deleteNode

///////////////////////////////////////////////////////////////////////////////
//正向输出链表
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
//反向输出链表
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

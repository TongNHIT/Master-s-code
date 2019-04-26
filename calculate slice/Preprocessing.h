#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//����һ��Ԥ�����ࣺ�����������ű���    
//                      
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_PREPROCESSING_H__
#define __LMZ_PREPROCESSING_H__

#include <fstream>
#include <iostream>

#include <windows.h>
#include "Read_PtCloud.h"


///////////////////////////////////////////////////////////////////////////////
//Ԥ������
class CJsGrPpro
{
private:

public:
	float FinalScale;                  //�������ű���
	int FinalSect;                     //���յ���Ƭ����

	inline float Final_Scale(void);    //�����������ű���
	inline float Sections(void);       //����������Ƭ����

};
CJsGrPpro ppro;

///////////////////////////////////////////////////////////////////////////////
//�����������ű������Կ���Nurbs���ߵ���ʾ
inline float CJsGrPpro::Final_Scale(void)
{
	FinalScale = file1.Scale;
	return (FinalScale);
}

///////////////////////////////////////////////////////////////////////////////
//�������յ���Ƭ����
inline float CJsGrPpro::Sections(void)
{
	int sect1;                   //��Ƭ����
	float k;                     //ϵ��k
	k = 5;

	//��Ƭ�����ļ���(��������)
	if (float((file1.Scale) / (k)) -
		int((file1.Scale) / (k)) >= 0.5)
		sect1 = int((file1.Scale) / (k)) + 1;
	else
		sect1 = int((file1.Scale) / (k));
	
	
	
	
//	if (float((2.0*file1.Scale) / (k*file1.dens)) -
//		int((2.0*file1.Scale) / (k*file1.dens)) >= 0.5)
//		sect1 = int((2.0*file1.Scale) / (k*file1.dens)) + 1;
//	else
	//	sect1 = int((2.0*file1.Scale) / (k*file1.dens));

	FinalSect = sect1;
	cout << "��Ƭ����Ϊ��" << FinalSect << endl;

	return (FinalSect);
}

///////////////////////////////////////////////////////////////////////////////
/*inline void CJsGrPpro::display(void)
{
	int i;
	//glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glColor3f(0.0,0.0,0.0);

	glScaled(1/FinalScale,1/FinalScale,1/FinalScale);
	glPointSize(1);

	//model01�ĵ���ģ��
	glBegin(GL_POINTS);
	for(i=0;i<file1.num;i++)
		glVertex3fv(&file1.Pts[i][0]);
	glEnd();
	//model02�ĵ���ģ��
	glBegin(GL_POINTS);
	for(i=0;i<file2.num;i++)
		glVertex3fv(&file2.Pts[i][0]);
	glEnd();

	glColor3f(1.0,0.0,0.0);
	glPointSize(2);
	//model01�г�ȡ�������
	glBegin(GL_POINTS);
		for(i=0;i<file1.N;i++)
			glVertex3fv(&file1.S[i][0]);
	glEnd();
	//model02�г�ȡ�������
	glBegin(GL_POINTS);
		for(i=0;i<file2.N;i++)
			glVertex3fv(&file2.S[i][0]);
	glEnd();

	glFlush();
}*/

#endif //__LMZ_PREPROCESSING_H__


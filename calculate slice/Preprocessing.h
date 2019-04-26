#pragma once
#pragma once
///////////////////////////////////////////////////////////////////////////////
//定义一个预处理类：计算最终缩放比例    
//                      
///////////////////////////////////////////////////////////////////////////////
#ifndef __LMZ_PREPROCESSING_H__
#define __LMZ_PREPROCESSING_H__

#include <fstream>
#include <iostream>

#include <windows.h>
#include "Read_PtCloud.h"


///////////////////////////////////////////////////////////////////////////////
//预处理类
class CJsGrPpro
{
private:

public:
	float FinalScale;                  //最终缩放比例
	int FinalSect;                     //最终的切片层数

	inline float Final_Scale(void);    //计算最终缩放比例
	inline float Sections(void);       //计算最终切片层数

};
CJsGrPpro ppro;

///////////////////////////////////////////////////////////////////////////////
//计算最终缩放比例，以控制Nurbs曲线的显示
inline float CJsGrPpro::Final_Scale(void)
{
	FinalScale = file1.Scale;
	return (FinalScale);
}

///////////////////////////////////////////////////////////////////////////////
//计算最终的切片层数
inline float CJsGrPpro::Sections(void)
{
	int sect1;                   //切片层数
	float k;                     //系数k
	k = 5;

	//切片层数的计算(四舍五入)
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
	cout << "切片层数为：" << FinalSect << endl;

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

	//model01的点云模型
	glBegin(GL_POINTS);
	for(i=0;i<file1.num;i++)
		glVertex3fv(&file1.Pts[i][0]);
	glEnd();
	//model02的点云模型
	glBegin(GL_POINTS);
	for(i=0;i<file2.num;i++)
		glVertex3fv(&file2.Pts[i][0]);
	glEnd();

	glColor3f(1.0,0.0,0.0);
	glPointSize(2);
	//model01中抽取的随机点
	glBegin(GL_POINTS);
		for(i=0;i<file1.N;i++)
			glVertex3fv(&file1.S[i][0]);
	glEnd();
	//model02中抽取的随机点
	glBegin(GL_POINTS);
		for(i=0;i<file2.N;i++)
			glVertex3fv(&file2.S[i][0]);
	glEnd();

	glFlush();
}*/

#endif //__LMZ_PREPROCESSING_H__


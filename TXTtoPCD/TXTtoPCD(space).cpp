/*********************************************

用来转换txt文件，中间用空格隔开

***********************************************/
#include<iostream>
#include<fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

typedef struct tagPOINT_3D
{
	double x;  //mm world coordinate x
	double y;  //mm world coordinate y
	double z;  //mm world coordinate z
	double r;
}POINT_WORLD;

int main()
{
	/////加载txt数据
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen("E:\\matlabcode\\g.txt", "r");

	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "txt数据加载失败！" << endl;
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZ> cloud;


	// Fill in the cloud data
	cloud.width = number_Txt;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);


	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = m_vTxtPoints[i].x;
		cloud.points[i].y = m_vTxtPoints[i].y;
		cloud.points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII("E:\\matlabcode\\g.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to txt2pcd.pcd." << std::endl;


	for (size_t i = 0; i < cloud.points.size(); ++i)
		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;


	system("pause");
	return 0;
}


#include <iostream>  
#include <cmath>  
#include <fstream>
#include <pcl/io/pcd_io.h>     //pcd��д�����ͷ�ļ�
#include <pcl/point_types.h>   //PCL��֧�ֵĵ�����ͷ�ļ�
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

using namespace std;

/*
PointSet[]������ĵ㼯
ch[]�������͹���ϵĵ㼯��������ʱ�뷽������
n��PointSet�еĵ����Ŀ
len�������͹���ϵĵ�ĸ���
*/

struct Point
{
	float x, y;
};

//С��0,˵������p0p1�ļ��Ǵ���p0p2�ļ���  
float multiply(Point p1, Point p2, Point p0)
{
	return((p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y));
}

float dis(Point p1, Point p2)
{
	return(sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)));
}


void Graham_scan(Point PointSet[], Point ch[], int n, int &len)
{
	int i, j, k = 0, top = 2;
	Point tmp;

	//�ҵ�������ƫ����Ǹ���  
	for (i = 1; i < n; i++)
		if ((PointSet[i].y < PointSet[k].y) || ((PointSet[i].y == PointSet[k].y) && (PointSet[i].x < PointSet[k].x)))
			k = i;
	//�������ָ��ΪPointSet[0]  
	tmp = PointSet[0];
	PointSet[0] = PointSet[k];
	PointSet[k] = tmp;

	//�����Ǵ�С����,����ƫ�̽�������  
	for (i = 1; i < n - 1; i++)
	{
		k = i;
		for (j = i + 1; j < n; j++)
			if ((multiply(PointSet[j], PointSet[k], PointSet[0]) > 0)
				|| ((multiply(PointSet[j], PointSet[k], PointSet[0]) == 0)
					&& (dis(PointSet[0], PointSet[j]) < dis(PointSet[0], PointSet[k]))))
				k = j;//k���漫����С���Ǹ���,������ͬ����ԭ�����  
		tmp = PointSet[i];
		PointSet[i] = PointSet[k];
		PointSet[k] = tmp;
	}
	//������������ջ  
	ch[0] = PointSet[0];
	ch[1] = PointSet[1];
	ch[2] = PointSet[2];
	//�ж����������е�Ĺ�ϵ  
	for (i = 3; i < n; i++)
	{
		//����������ת�Ĺ�ϵ,ջ��Ԫ�س�ջ  
		while (multiply(PointSet[i], ch[top], ch[top - 1]) >= 0) top--;
		//��ǰ����ջ�����е����������ϵ,�����ջ.  
		ch[++top] = PointSet[i];
	}
	len = top + 1;
}

const int maxN = 1000;
Point PointSet[maxN];
Point ch[maxN];
int n;
int len;

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  //����һ��PointCloud<PointXYZ>boost����ָ�벢ʵ����
	cout << "reading pcd file...\n";
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCLData/pointcloud/RGBD/kinect4.10/bancaiboudary.pcd", *cloud) == -1)   //�򿪵����ļ�
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}                                                                   //�Ӵ����ϼ��ص������ݵ������ƴ洢����
	std::cout << "Loaded " << cloud->width*cloud->height
		<< " data points from test_pcd.pcd .  "
		<< "  width: " << cloud->width << "  height: " << cloud->height << std::endl;

	int n = cloud->points.size();
	for (int i = 0; i < n; i++)
	{
		PointSet[i].x = cloud->points[i].x;
		PointSet[i].y = cloud->points[i].y;
	}
	Graham_scan(PointSet, ch, n, len);
	//std::fstream fileW("1.txt");
	for (int i = 0; i < len; i++)
	{
		cout << ch[i].x << " " << ch[i].y << endl;
		//fileW << ch[i].x << " " << ch[i].y << endl;
	}
	//fileW.close();
	system("pause");
	return 0;
}
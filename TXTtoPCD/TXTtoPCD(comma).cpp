/*************************************************

�������������м��ö��Ÿ�����txt�ļ�

*************************************************/
#include <pcl/io/pcd_io.h>  
#include<iostream>  
using namespace std;
int numofPoints(const char* fname) {
	int n = 0;
	int c = 0;
	FILE *fp;
	fp = fopen(fname, "r");
	do {
		c = fgetc(fp);
		if (c == '\n') {
			++n;
		}
	} while (c != EOF);
	fclose(fp);
	return n;
}
int main()
{
	int n = 0; //n�������ļ��е����      
	FILE *fp_1;
	fp_1 = fopen("F:/PCLData/pointcloud/finalexperiment/test/02.txt", "r");
	n = numofPoints("replay3.txt");//ʹ��numofPoints���������ļ��е����  
	std::cout << "there are " << n << " points in the file..." << std::endl;
	//�½�һ�������ļ���Ȼ�󽫽ṹ�л�ȡ��xyzֵ���ݵ�����ָ��cloud�С�  
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = n;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	//�����ƶ��벢�����½�����ָ���xyz      
	double x, y, z;
	int i = 0;
	while (3 == fscanf(fp_1, "%lf,%lf,%lf/n", &x,&y,&z))
	{
		//cout << x << " " << y << " " << z << endl;
		cloud.points[i].x = x;
		cloud.points[i].y = y;
		cloud.points[i].z = z;
		++i;
	}
	fclose(fp_1);
	//������ָ��ָ������ݴ���pcd�ļ�  

	pcl::io::savePCDFileASCII("F:/PCLData/pointcloud/finalexperiment/test/02.pcd", cloud);

	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
	system("pause");
	return 0;
}
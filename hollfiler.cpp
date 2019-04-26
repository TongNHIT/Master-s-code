#include <iostream> //��׼���������
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/BoundaryEstimation/BoundaryEstimation/yiziboudary.pcd", *cloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/BoundaryEstimation/BoundaryEstimation/yiziNoBoundpoints.pcd", *cloud1);
	std::cout << "cloud sieze is:" << cloud->size() << std::endl;
	std::cout << "cloud1 sieze is:" << cloud1->size() << std::endl;
	*cloud2 = *cloud;
	*cloud2 += *cloud1;
	
	std::cout << "cloud2 sieze is:" << cloud2->size() << std::endl;
	pcl::io::savePCDFileASCII("yizi4101.pcd", *cloud2);
	return (0);
}

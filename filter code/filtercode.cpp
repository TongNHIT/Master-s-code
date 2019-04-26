
#include "Mainwindows.h"
#include <math.h>
#include "vtkAutoInit.h" 
using namespace std;
int main(int argc, char** argv)
{
	VTK_MODULE_INIT(vtkRenderingOpenGL);
	VTK_MODULE_INIT(vtkInteractionStyle);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	cout << "loading PCD ...\n";
	clock_t s1, f1, s2, s3, s4, s5, s6, s7, f2, f3, f4, f5, f6, f7;
	double t1, t2, t3, t4, t5, t6, t7;
	
	s1 = clock();

	pcl::PCDReader reader;//读取点云数据
	reader.read<pcl::PointXYZ>("F:/PCLData/pointcloud/07-600.pcd", *cloud);
	//reader.read<pcl::PointXYZ>("E:/PCLData/pointcloud/chulihoudianyun/replay112lvbo.pcd", *cloud2);
	f1 = clock();
	t1 = (double)(f1 - s1) / CLOCKS_PER_SEC;
	cout << "\n加载 "<< cloud->width <<" * "<< cloud->height <<" = "
		<< cloud->width*cloud->height <<" 个点云共需 "<< t1 << " s！" << endl;
	reducewrongpoint(cloud, cloud1, 5, 3);
	//reduceplant(cloud, cloud2, 0.003, 1000, 0.95);

	//VoxelGrid(cloud, cloud1);
	//cout << "下采样结束 " << cloud1->width*cloud1->height << " points left" << endl;
	
	//cout << "reducewrongpoint...\n";
	//s2 = clock();
//	reducewrongpoint(cloud1, cloud2, 20,1 );
//	f2 = clock();
//	t2 = (double)(f2 - s2) / CLOCKS_PER_SEC;
//	cout << "统计滤波耗时" << t2 << " s! " << cloud2->width*cloud2->height <<" points left"<< endl;
//	tiaojian(cloud2, cloud3);
	//VoxelGrid(cloud1, cloud2);
	//cout << "下采样结束 " << cloud1->width*cloud1->height << " points left" << endl;
//	std::vector<int> mapping;
//	pcl::removeNaNFromPointCloud(*cloud3, *cloud4, mapping);
	//radiusfilter(cloud4,cloud5,0.25,5);
	/*
	cout << "getnormals...\n";
	s4 = clock();
	getnormal(cloud4 ,normals ,5);
	f4 = clock();
	t4 = (double)(f4 - s4) / CLOCKS_PER_SEC;
	cout << "求法线耗时" << t4 << " s! " << cloud4->width*cloud4->height << " points left!"<< endl;
	
	cout << "shuangbian...\n";
	s3 = clock();
	shuangbian(cloud4, cloud5, normals);
	f3 = clock();
	t3 = (double)(f3 - s3) / CLOCKS_PER_SEC;
	cout << "双边滤波耗时" << t3 << " s! " << cloud5->width*cloud5->height <<" points left!"<<endl;
	//getnormal(cloud2, normals2, 10);
	//cloudchange(cloud1, cloud2);
	//normalchange(cloud1,normals,normals2);
	//reducewrongpoint(cloud4, cloud6, 10, 1);
	*/
	//radiusfilter(cloud3,cloud4,0.01,13);
	//tiaojian(cloud4,cloud5);
	//pcl::io::savePCDFileASCII("E:/PCLData/pointcloud/test/paper/replay112lvbo.pcd", *cloud1); //将点云保存到PCD文件中
	//std::cerr << "Saved " << cloud5->width*cloud5->height<< " data points to test_pcd.pcd." << std::endl;
	
	//reducewrongpoint(cloud3, cloud4, 4, 1);
	cout << "showing PCD...\n";
	//showrgbcloudwithnormal(cloud1, normals);
	//showrgbcloudwithnormal(cloud1, normals2);
	showviewintwodifferentwindow(cloud, cloud1);
	//showrgbcloudwithnormal(cloud2, normals);
	system("pause");
	return (0);
}
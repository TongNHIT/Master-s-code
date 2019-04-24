//头文件
#include <iostream>
#include <pcl/point_types.h>
#include  <pcl/point_cloud.h>

//定义函数体
int main(int argc, char** argv)

//定义点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 1000;
	cloud->height = 1;
	cloud->resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->at(i).x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->at(i).y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->at(i).z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

//读取点云
	#include <pcl/io/pcl_io.h>
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:\office3-after21111.pcd", *cloud);

	//插入和移除点
	pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
	cloud->erase(index);//删除第一个
	index = cloud->begin() + 5;
	cloud->erase(cloud->begin());//删除第5个
	pcl::PointXYZ point = { 1, 1, 1 };
	//在索引号为5的位置1上插入一点，原来的点后移一位
	cloud->insert(cloud->begin() + 5, point);
	cloud->push_back(point);//从点云最后面插入一点
	std::cout << cloud->points[5].x;//输出1

//点云的坐标变换（全局以及局部）
  //全局变化
	  //构造变化矩阵
	  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	  float theta = M_PI / 4;   //旋转的度数，这里是45度
	  transform_1(0, 0) = cos(theta);  //这里是绕的Z轴旋转
	  transform_1(0, 1) = -sin(theta);
	  transform_1(1, 0) = sin(theta);
	  transform_1(1, 1) = cos(theta);
	  //   transform_1 (0,2) = 0.3;   //这样会产生缩放效果
	  //   transform_1 (1,2) = 0.6;
	  //    transform_1 (2,2) = 1;
	  transform_1(0, 3) = 25; //这里沿X轴平移
	  transform_1(1, 3) = 30;
	  transform_1(2, 3) = 380;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::transformPointCloud(*cloud, *transform_cloud1, transform_1);  //不言而喻
  //局部
	pcl::transformPointCloud(*cloud, pcl::PointIndices indices, *transform_cloud1, matrix); //第一个参数为输入，第二个参数为输入点云中部分点集索引，第三个为存储对象，第四个是变换矩阵。

//删除点云中的无效点
	vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);





//ͷ�ļ�
#include <iostream>
#include <pcl/point_types.h>
#include  <pcl/point_cloud.h>

//���庯����
int main(int argc, char** argv)

//�������
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

//��ȡ����
	#include <pcl/io/pcl_io.h>
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:\office3-after21111.pcd", *cloud);

	//������Ƴ���
	pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
	cloud->erase(index);//ɾ����һ��
	index = cloud->begin() + 5;
	cloud->erase(cloud->begin());//ɾ����5��
	pcl::PointXYZ point = { 1, 1, 1 };
	//��������Ϊ5��λ��1�ϲ���һ�㣬ԭ���ĵ����һλ
	cloud->insert(cloud->begin() + 5, point);
	cloud->push_back(point);//�ӵ�����������һ��
	std::cout << cloud->points[5].x;//���1

//���Ƶ�����任��ȫ���Լ��ֲ���
  //ȫ�ֱ仯
	  //����仯����
	  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	  float theta = M_PI / 4;   //��ת�Ķ�����������45��
	  transform_1(0, 0) = cos(theta);  //�������Ƶ�Z����ת
	  transform_1(0, 1) = -sin(theta);
	  transform_1(1, 0) = sin(theta);
	  transform_1(1, 1) = cos(theta);
	  //   transform_1 (0,2) = 0.3;   //�������������Ч��
	  //   transform_1 (1,2) = 0.6;
	  //    transform_1 (2,2) = 1;
	  transform_1(0, 3) = 25; //������X��ƽ��
	  transform_1(1, 3) = 30;
	  transform_1(2, 3) = 380;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::transformPointCloud(*cloud, *transform_cloud1, transform_1);  //���Զ���
  //�ֲ�
	pcl::transformPointCloud(*cloud, pcl::PointIndices indices, *transform_cloud1, matrix); //��һ������Ϊ���룬�ڶ�������Ϊ��������в��ֵ㼯������������Ϊ�洢���󣬵��ĸ��Ǳ任����

//ɾ�������е���Ч��
	vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);





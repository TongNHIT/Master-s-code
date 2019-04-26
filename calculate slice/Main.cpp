/*************************************************************

���հ�ĵ�����Ƭ�㷨
2019.04.17

**************************************************************/

#include "vtkAutoInit.h" 
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "Preprocessing.h"
#include "Slicing.h"
#include "Read_PtCloud.h"
#include "linkedListType.h"
#include "headings.h"
int main(int argc, char *argv[])
{
	VTK_MODULE_INIT(vtkRenderingOpenGL);
	VTK_MODULE_INIT(vtkInteractionStyle);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtransformed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr slicetransformed(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloudwithnormal(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	
	
	//���ö�ȡ��
	string str = "E:/PCLData/pointcloud/chulihoudianyun/replay112lvbo.pcd";
	char* str2 = const_cast<char*>(str.c_str());
	file1.cloudin(str2);
	file1.trans();//PCA�㷨�任����
	file1.Scale_PtCloud();//�������ģ�͵����ų߶�
	file1.Dens_PtCloud(); //���Ƶ����ܶ�
	file1.cloudtransformed();
	cloud = file1.cloudin(str2);
	cloudtransformed = file1.cloudtransformed();

	//����Ԥ������
	ppro.Final_Scale();//�����������ű������Կ���Nurbs���ߵ���ʾ
	ppro.Sections();//������Ƭ����

	//������Ƭ��
	slice1.Tkness(file1.Scale);//��Ƭ���
	slice1.Direct(file1.Scale, &file1.XYZ_Scale[0]);//��Ƭ����
    slice1.InterS(file1.num, file1.Pts, slice1.flag, slice1.tkness);//ƽ��������󽻵�
	slice1.Curves(slice1.flag, file1.dens);//�����߹���
	slicetransformed = slice1.cloudout();//��Ƭ����
	pcl::transformPointCloud(*slicetransformed, *slice, file1.trans());



	//normalsolution(file1.cloudtransformed(), slice1.cloudout(), cloudwithnormal, slices, normals);
	//showslicewithnormal(file1.cloudin(str2),file1.cloudtransformed(), slices, normals);
	 
	//showviewintwodifferentwindow(file1.cloudtransformed(), slice1.cloudout());


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0.0, 0.0, 0.0);//������ɫ,(1,1,1)��ɫ
	viewer->addCoordinateSystem(1.0f, "global");//����ϵ
	//viewer_ptr->setCameraPosition(0,0,200); //��������ԭ��

	for (std::size_t i = 0; i < slice->size() - 1; i++)
	{
		pcl::PointXYZ &p1 = slice->at(i);
		pcl::PointXYZ &p2 = slice->at(i + 1);
		std::ostringstream os;
		os << "line_" << i;
		viewer->addLine<pcl::PointXYZ>(p1, p2, 0, 0, 255, os.str());
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(slice, 255, 0, 0);//��ɫ

	viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud1_color, "cloud1");
	viewer->addPointCloud<pcl::PointXYZ>(slice, cloud2_color, "slices");//���Ʊ�ǩ
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 5, 0.02, "normal");//���߱�ǩ
  //���У�����5��ʾ����������ÿ5������ʾһ������������ȫ����ʾ��������Ϊ1��  0.02��ʾ�������ĳ��ȣ����һ��������ʱ����֪�� ���Ӱ��ģ�);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "slices");
	viewer->initCameraParameters();//��ʼ���������
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;

}

//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("slices of pointcloud"));
//	int v1 = 0;
//	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer->addCoordinateSystem(0.1);
//	viewer->setBackgroundColor(1.0, 1.0, 1.0, v1);
////	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v1_color(file1.cloudtransformed(), 0, 0, 255);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> slice_color(cloud5, 255, 0, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(file1.cloudin(str2), 0, 255, 0);
////	viewer->addPointCloud<pcl::PointXYZ>(file1.cloudtransformed(), v1_color, "cloudt", v1);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud5, slice_color, "slicein", v1);
//	viewer->addPointCloud<pcl::PointXYZ>(file1.cloudin(str2), source_color, "cloudin", v1);
//	//	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(slicein, slicenormal, 1, 0.02, "slicenormal", v1);
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudt");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "slicein");
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(cloudo, 0, 0, 255);
//	//viewer->addPointCloud<pcl::PointXYZ>(cloudo, v2_color, "slices", v1);
//	int v2 = 0;
//	//viewer->addCoordinateSystem(1.0);
//	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//	viewer->setBackgroundColor(1.0, 1.0, 1.0, v2);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> slice_color1(slice1.cloudout(), 255, 0, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(slice1.cloudout(), slice_color1, "slicein1", v2);
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "slicein");
//	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(slicein, slicenormal, 1, 0.02, "normals", v2);
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
//	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "slices");
//	//viewer->initCameraParameters();
//	while (!viewer->wasStopped())
//	{
//		viewer->spin();
//	}
//
//	system("pause");
//    return (0);
//}











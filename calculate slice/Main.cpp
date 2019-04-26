/*************************************************************

最终版的点云切片算法
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
	
	
	//调用读取类
	string str = "E:/PCLData/pointcloud/chulihoudianyun/replay112lvbo.pcd";
	char* str2 = const_cast<char*>(str.c_str());
	file1.cloudin(str2);
	file1.trans();//PCA算法变换矩阵
	file1.Scale_PtCloud();//计算点云模型的缩放尺度
	file1.Dens_PtCloud(); //估计点云密度
	file1.cloudtransformed();
	cloud = file1.cloudin(str2);
	cloudtransformed = file1.cloudtransformed();

	//调用预处理类
	ppro.Final_Scale();//计算最终缩放比例，以控制Nurbs曲线的显示
	ppro.Sections();//计算切片层数

	//调用切片类
	slice1.Tkness(file1.Scale);//切片厚度
	slice1.Direct(file1.Scale, &file1.XYZ_Scale[0]);//切片方向
    slice1.InterS(file1.num, file1.Pts, slice1.flag, slice1.tkness);//平面与点云求交点
	slice1.Curves(slice1.flag, file1.dens);//多义线构建
	slicetransformed = slice1.cloudout();//切片数据
	pcl::transformPointCloud(*slicetransformed, *slice, file1.trans());



	//normalsolution(file1.cloudtransformed(), slice1.cloudout(), cloudwithnormal, slices, normals);
	//showslicewithnormal(file1.cloudin(str2),file1.cloudtransformed(), slices, normals);
	 
	//showviewintwodifferentwindow(file1.cloudtransformed(), slice1.cloudout());


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0.0, 0.0, 0.0);//背景黑色,(1,1,1)白色
	viewer->addCoordinateSystem(1.0f, "global");//坐标系
	//viewer_ptr->setCameraPosition(0,0,200); //设置坐标原点

	for (std::size_t i = 0; i < slice->size() - 1; i++)
	{
		pcl::PointXYZ &p1 = slice->at(i);
		pcl::PointXYZ &p2 = slice->at(i + 1);
		std::ostringstream os;
		os << "line_" << i;
		viewer->addLine<pcl::PointXYZ>(p1, p2, 0, 0, 255, os.str());
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(slice, 255, 0, 0);//红色

	viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud1_color, "cloud1");
	viewer->addPointCloud<pcl::PointXYZ>(slice, cloud2_color, "slices");//点云标签
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 5, 0.02, "normal");//法线标签
  //其中，参数5表示整个点云中每5各点显示一个法向量（若全部显示，可设置为1，  0.02表示法向量的长度，最后一个参数暂时还不知道 如何影响的）);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "slices");
	viewer->initCameraParameters();//初始化相机参数
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











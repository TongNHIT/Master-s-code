#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include "vtkAutoInit.h" 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/principal_curvatures.h>
using namespace std;

int main(int argc, char** argv)
{
	VTK_MODULE_INIT(vtkRenderingOpenGL);
	VTK_MODULE_INIT(vtkInteractionStyle);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/pointcloud/RGBD/kinect4.10/bancai4101.pcd", *cloud) == -1) {

		PCL_ERROR("Could not read file \n");
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/PCLData/pointcloud/RGBD/kinect4.10/bancaiboudary1.pcd", *cloud1) == -1) {

		PCL_ERROR("Could not read file \n");
	}

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//为kdtree添加点云数据
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	//点云法向计算时，需要搜索的近邻点大小
	n.setKSearch(10);
	//n.flipNormalTowardsViewpoint(0,0,0);
	n.setViewPoint(0, 0, 100);
	n.compute(*normals);

	//将点云数据与法向信息拼接
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	/*
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;
	principal_curvatures_estimation.setInputCloud(cloud);
	principal_curvatures_estimation.setInputNormals(normals);
	principal_curvatures_estimation.setSearchMethod(tree);
	principal_curvatures_estimation.setRadiusSearch(3.0);
	principal_curvatures_estimation.compute(*principal_curvatures);
	*/


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n1;
	pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	//为kdtree添加点云数据
	tree1->setInputCloud(cloud1);
	n1.setInputCloud(cloud1);
	n1.setSearchMethod(tree);
	//点云法向计算时，需要搜索的近邻点大小
	n1.setKSearch(10);
	//开始进行法向计算
	n1.compute(*normals1);



	//显示设置

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	int v1 = 0;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->addCoordinateSystem(1.0);
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v1_color(cloud, 255, 0, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, v1_color, "cloud1", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
//	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 5, 2, "normals", v1);
	int v2 = 0;
	//viewer->addCoordinateSystem(1.0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(cloud1, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud1, v2_color, "cloud2", v2);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud1, normals1, 5, 0.02, "normals_refined", v2);
	viewer->initCameraParameters();

	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 5, 2, "normals");
	//viewer->addPointCloudPrincipalCurvatures(cloud_with_normals, principal_curvatures, 100, 1.0f, "cloud");
	*/
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	// Finish
	return (0);
}
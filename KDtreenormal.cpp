#include "vtkAutoInit.h" 
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/normal_refinement.h>
using namespace std;
int main(int argc, char *argv[])
{
	VTK_MODULE_INIT(vtkRenderingOpenGL);
	VTK_MODULE_INIT(vtkInteractionStyle);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr slicecloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr trackpoint(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr normalout(new pcl::PointCloud<pcl::Normal>);

	clock_t s1, f1, s2, s3, s4, s5, s6, s7, f2, f3, f4, f5, f6, f7;
	double t1, t2, t3, t4, t5, t6, t7;

	s1 = clock();

	pcl::io::loadPCDFile("E:/PCLData/pointcloud/test/yepent.pcd", *cloud1);
	pcl::io::loadPCDFile("E:/PCLData/pointcloud/test/yepenslices.pcd", *cloud2);
	cout << "加载 " << cloud1->width << " * " << cloud1->height << " = "
		<< cloud1->width*cloud1->height << endl;
	f1 = clock();
	t1 = (double)(f1 - s1) / CLOCKS_PER_SEC;
	cout << "t1 = " << t1 << endl;

	// Estimated and refined normals
	pcl::PointCloud<pcl::Normal> normals;
	pcl::PointCloud<pcl::Normal> normals_refined;
	// Search parameters
	const int k = 10;
	std::vector<std::vector<int> > k_indices;
	std::vector<std::vector<float> > k_sqr_distances;
	// Run search
	pcl::search::KdTree<pcl::PointXYZ> search;
	search.setInputCloud(cloud1);
	search.nearestKSearch(*cloud2, std::vector<int>(), k, k_indices, k_sqr_distances);
	cout << "KDtree搜索" << endl;
	// Use search results for normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	for (unsigned int i = 0; i < cloud2->size(); ++i)
	{
		pcl::Normal normal;
		ne.computePointNormal(*cloud1, k_indices[i], normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint(cloud2->points[i], -100, 0, 0, normal.normal_x, normal.normal_y, normal.normal_z);
		normals.push_back(normal);
	}


//	pcl::concatenateFields(*cloud3, normals_refined, *cloud_with_normals);
	//pcl::io::savePCDFileASCII("cloud_with_normals.pcd", *cloud_with_normals);
	//提取切片点法向




	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	int v1 = 0;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addCoordinateSystem(0.1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v1_color(cloud1, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> slice_color(cloud2, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud1, v1_color, "cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud2, slice_color, "slice", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "slice");
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud2, normals.makeShared(), 1, 1, "normal", v1);
	int v2 = 0;
	//viewer->addCoordinateSystem(1.0);
	//viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(slicecloud, 255, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZ>(slicecloud, v2_color, "slices", v2);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "slices");
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(slicecloud, slicenormal, 1, 1, "normals", v2);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spin();
	}
	return 0;
}


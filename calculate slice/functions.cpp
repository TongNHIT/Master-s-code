#include "headings.h"

void normalsolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr slicein,
	pcl::PointCloud<pcl::PointNormal>::Ptr slice_with_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr sliceout,
	pcl::PointCloud<pcl::Normal>::Ptr slicenormal)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>());
	*cloud3 = *cloudin;
	for (int i = 0; i < slicein->size(); i++)
	{
		pcl::PointXYZ basic_point;
		basic_point.x = slicein->points[i].x;
		basic_point.y = slicein->points[i].y;
		basic_point.z = slicein->points[i].z;

		cloud3->points.push_back(basic_point);
	}
	cloud3->width = (int)cloud3->points.size();
	cloud3->height = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	tree->setInputCloud(cloud3);
	ne.setInputCloud(cloud3);
	ne.setViewPoint(-1, -1, 0);
	ne.setSearchMethod(tree);
	ne.setKSearch(20);
	ne.compute(*normal);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud3, *normal, *cloud_with_normals);

	for (int j = 0; j < slicein->size(); j++)
	{
		for (int i = 0; i < cloud_with_normals->size(); i++)
		{
			if (slicein->points[j].x == cloud_with_normals->points[i].x && slicein->points[j].y == cloud_with_normals->points[i].y && slicein->points[j].z == cloud_with_normals->points[i].z)
			{
				pcl::PointNormal basic_point;
				pcl::Normal basic_normal;
				pcl::PointXYZ basic_cloud;
				basic_point.x = cloud_with_normals->points[i].x;
				basic_point.y = cloud_with_normals->points[i].y;
				basic_point.z = cloud_with_normals->points[i].z;
				basic_point.normal_x = cloud_with_normals->points[i].normal_x;
				basic_point.normal_y = cloud_with_normals->points[i].normal_y;
				basic_point.normal_z = cloud_with_normals->points[i].normal_z;
				basic_point.curvature = cloud_with_normals->points[i].curvature;

				basic_cloud.x = cloud_with_normals->points[i].x;
				basic_cloud.y = cloud_with_normals->points[i].y;
				basic_cloud.z = cloud_with_normals->points[i].z;
				basic_normal.normal_x = cloud_with_normals->points[i].normal_x;
				basic_normal.normal_y = cloud_with_normals->points[i].normal_y;
				basic_normal.normal_z = cloud_with_normals->points[i].normal_z;
				basic_normal.curvature = cloud_with_normals->points[i].curvature;

				sliceout->points.push_back(basic_cloud);
				slice_with_normals->points.push_back(basic_point);
				slicenormal->points.push_back(basic_normal);
			}
		}
	}
	sliceout->width = (int)sliceout->points.size();
	sliceout->height = 1;
	slicenormal->width = (int)slicenormal->points.size();
	slicenormal->height = 1;
	slice_with_normals->width = (int)slice_with_normals->points.size();
	slice_with_normals->height = 1;
}


void showslicewithnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudo, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudt, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr slicein,pcl::PointCloud<pcl::Normal>::Ptr slicenormal)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("slices of pointcloud"));
	int v1 = 0;
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addCoordinateSystem(1.0);
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v1_color(cloudt, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> slice_color(slicein, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloudt, v1_color, "cloudt", v1);
	viewer->addPointCloud<pcl::PointXYZ>(slicein, slice_color, "slicein", v1);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(slicein, slicenormal, 1, 0.02, "slicenormal", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloudin");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "slicein");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(cloudo, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(cloudo, v2_color, "slices", v1);
	//int v2 = 0;
	//viewer->addCoordinateSystem(1.0);
	//viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v2_color(cloudo, 0, 255, 0);
	//viewer->addPointCloud<pcl::PointXYZ>(cloudo, v2_color, "slices", v2);
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(slicein, slicenormal, 1, 0.02, "normals", v2);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "slices");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spin();
	}
}


void showviewintwodifferentwindow(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right)//输入两个XYZRGBA
{
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
	viewer.setBackgroundColor(1, 1, 1, v1);
	viewer.addCoordinateSystem(1.0f, "globalleft", v1);//红色是 X 轴；绿色是Y轴；蓝色是Z轴
	viewer.addText("Original point cloud", 10, 10, 0, 0, 0, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_left, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_left, single_color, "cloud_left", v1);
	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1, v2);
	viewer.setBackgroundColor(1, 1, 1, v2);
	viewer.addCoordinateSystem(1.0f, "globalright", v2);
	viewer.addText("Point cloud after processing", 10, 10, 0, 0, 0, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud_right, 0, 0, 255);

	viewer.addPointCloud<pcl::PointXYZ>(cloud_right, single_color1, "cloud_right", v2);

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_left");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_right");

	while (!viewer.wasStopped())
	{ 
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}



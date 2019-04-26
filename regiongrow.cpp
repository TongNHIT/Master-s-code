#include <pcl/ModelCoefficients.h>//ģ��ϵ��
#include <pcl/point_types.h>//���ƻ�������
#include <pcl/io/pcd_io.h>//io
#include <pcl/filters/extract_indices.h>//����������ȡ����
#include <pcl/filters/voxel_grid.h>//���ظ��²���
#include <pcl/features/normal_3d.h>//���Ʒ�������
#include <pcl/kdtree/kdtree.h>//kd�������㷨
#include <pcl/sample_consensus/method_types.h>//��������
#include <pcl/sample_consensus/model_types.h>//����ģ��
#include <pcl/segmentation/sac_segmentation.h>//������÷ָ�
#include <pcl/segmentation/extract_clusters.h>//ŷʽ����ָ�
#include <pcl/visualization/pcl_visualizer.h> // ���ӻ�

/******************************************************************************
 �򿪵������ݣ����Ե��ƽ����˲��ز���Ԥ����Ȼ�����ƽ��ָ�ģ�ͶԵ��ƽ��зָ��
 ��ȡ��������������ƽ���ϵĵ㼯�����������
******************************************************************************/
int
main(int argc, char** argv)
{
	// ��ȡ���泡������
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("F:/PCLData/pointcloud/RGBD/kinect4.10/yiziboudary1.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*
  // ֮ǰ�ɽ��С�ͳ��ѧ�˲�ȥ�����
	// //���ظ��˲��²�����1cm��1cm��1cm
	//pcl::VoxelGrid<pcl::PointXYZ> vg;//���ظ��˲��²���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//vg.setInputCloud(cloud);
	//vg.setLeafSize(0.01f, 0.01f, 0.01f);
	//vg.filter(*cloud_filtered);
	//std::cout << "PointCloud after filtering has: " <<
	//	cloud_filtered->points.size() <<
	//	" data points." << std::endl; //*
 //  //����ƽ��ģ�ͷָ�Ķ������ò���
	//pcl::SACSegmentation<pcl::PointXYZ> seg;
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//ϵ��
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::PCDWriter writer;
	//seg.setOptimizeCoefficients(true);
	//seg.setModelType(pcl::SACMODEL_PLANE);    //�ָ�ģ�͡�ƽ��ģ��
	//seg.setMethodType(pcl::SAC_RANSAC);       //�������һ���ԡ��������Ʒ���
	//seg.setMaxIterations(100);                //���ĵ����Ĵ���
	//seg.setDistanceThreshold(0.02);           //���÷���ģ�͵��ڵ㡡��ֵ

	//int i = 0, nr_points = (int)cloud_filtered->points.size();//�²���ǰ��������

	//while (cloud_filtered->points.size() > 0.3 * nr_points)
	//	// ģ�ͷָֱ����ʣ�����������30%���ϡ�ȷ��ģ�͵��ƽϺ�
	//{
	//	// Segment the largest planar component from the remaining cloud
	//	seg.setInputCloud(cloud_filtered);
	//	seg.segment(*inliers, *coefficients);//�ָ�
	//	if (inliers->indices.size() == 0)
	//	{
	//		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	//		break;
	//	}

	//	pcl::ExtractIndices<pcl::PointXYZ> extract;//��������ȡ����
	//	extract.setInputCloud(cloud_filtered);
	//	extract.setIndices(inliers);//��ȡ����ƽ��ģ�͵��ڵ�
	//	extract.setNegative(false);
	//	// ƽ��ģ���ڵ�
	//	extract.filter(*cloud_plane);
	//	std::cout << "PointCloud representing the planar component: " <<cloud_plane->points.size() <<" data points." << std::endl;
	//	// ��ȥƽ����ڵ㣬��ȡʣ�����
	//	extract.setNegative(true);
	//	extract.filter(*cloud_f);
	//	*cloud_filtered = *cloud_f;//ʣ�����
	//}

	// ����ƽ���ϡ��ĵ����š�ʹ�á�ŷʽ������㷨��kd���������Ե��ƾ���ָ�
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);//������ƽ���������ĵ���
	std::vector<pcl::PointIndices> cluster_indices;// ����������
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// ŷʽ�������
	ec.setClusterTolerance(0.01);                    // ���ý��������������뾶Ϊ2cm
	ec.setMinClusterSize(5);                       // ����һ��������Ҫ�����ٵĵ���ĿΪ100
	ec.setMaxClusterSize(25000);                     // ����һ��������Ҫ��������ĿΪ25000
	ec.setSearchMethod(tree);                        // ���õ��Ƶ���������
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);           //�ӵ�������ȡ���࣬������������������cluster_indices��

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_all(new pcl::PointCloud<pcl::PointXYZ>);
	//�������ʵ�������cluster_indices,ֱ���ָ���о���
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //��ȡÿһ�������š��ġ���

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		// writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster);
		j++;

		*cloud_cluster_all += *cloud_cluster;
	}
	//pcl::io::savePCDFileASCII("cloud_cluster_all", *cloud_cluster_all);


	// 3D������ʾ ��ɫ
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(255, 255, 255);//������ɫ����ɫ
	//viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters();
	//ƽ���ϵĵ��ơ���ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_plane_handler(cloud, 255, 0, 0);
	viewer.addPointCloud(cloud, cloud_plane_handler, "plan point");//��ӵ���
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plan point");

	//  ���ϵ��������Ȼ����һ�����ĸ��յ����š���ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cluster_handler(cloud_cluster_all, 0, 255, 0);
	viewer.addPointCloud(cloud_cluster_all, cloud_cluster_handler, "cloud_cluster point");//��ӵ���
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_cluster point");

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}
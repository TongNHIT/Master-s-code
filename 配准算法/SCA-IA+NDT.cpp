#define BOOST_TYPEOF_EMULATION
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void visualize_pcd(PointCloud::Ptr pcd_src,
	PointCloud::Ptr pcd_tgt,
	PointCloud::Ptr pcd_final)
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud");
	viewer.addPointCloud(pcd_final, final_h, "final cloud");
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle)
{
	double ax, ay, az;
	if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1)
	{
		az = 0;
		double dlta;
		dlta = atan2(result_trans(0, 1), result_trans(0, 2));
		if (result_trans(2, 0) == -1)
		{
			ay = M_PI / 2;
			ax = az + dlta;
		}
		else
		{
			ay = -M_PI / 2;
			ax = -az + dlta;
		}
	}
	else
	{
		ay = -asin(result_trans(2, 0));
		ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
		az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
	}
	result_angle << ax, ay, az;
}

int main(int argc, char** argv)
{
	//���ص����ļ�(Դ���ƣ�����׼)
	PointCloud::Ptr cloud_src_o(new PointCloud);
	pcl::io::loadPCDFile("e:/Pcl_Program/pointcloud1.pcd", *cloud_src_o);
	PointCloud::Ptr cloud_tgt_o(new PointCloud);
	pcl::io::loadPCDFile("e:/Pcl_Program/pointcloud2.pcd", *cloud_tgt_o);
	clock_t start = clock();
	//ȥ��NAN�㣨����ֵ�㣩
	std::vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	std::cout << "remove *cloud_src_o nan" << endl;
	//�²����˲� Դ����
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(cloud_src_o);
	PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_src);
	std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;
	//pcl::io::savePCDFileASCII("bunny_src_down.pcd", *cloud_src);
	//������淨�� Դ����
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_src);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);//pcl::Normal��һ�ֵ����ͣ���������
	ne_src.setRadiusSearch(0.02);//�����ڽ���ķ�Χ
	ne_src.compute(*cloud_src_normals);

	//ȥ��NAN�� Ŀ�����
	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
	std::cout << "remove *cloud_tgt_o nan" << endl;
	//�²����˲� Ŀ�����
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.01, 0.01, 0.01);
	voxel_grid_2.setInputCloud(cloud_tgt_o);
	PointCloud::Ptr cloud_tgt(new PointCloud);
	voxel_grid_2.filter(*cloud_tgt);
	std::cout << "down size *cloud_tgt_o.pcd from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;
	///pcl::io::savePCDFileASCII("bunny_tgt_down.pcd", *cloud_tgt);
	//������淨�� Ŀ�����
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.02);
	ne_tgt.compute(*cloud_tgt_normals);

	//����FPFH   
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_src);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());//ÿ�����������һ��ֱ��ͼ��FPFH��������33ά
	fpfh_src.setRadiusSearch(0.05);
	//fpfh_src.setKSearch(20);
	fpfh_src.compute(*fpfhs_src);
	std::cout << "compute *cloud_src fpfh" << endl;

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(cloud_tgt);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(0.05);
	//fpfh_tgt.setKSearch(20);
	fpfh_tgt.compute(*fpfhs_tgt);
	std::cout << "compute *cloud_tgt fpfh" << endl;

	//SAC��׼
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);   //�������������С����
	//scia.setNumberOfSamples(2);     //����ÿ�ε�����ʹ�õ���������
	//scia.setCorrespondenceRandomness(20);  //���ü���Э����ʱ�ڽ����������ڽ���Խ�࣬Э�������Խ׼ȷ�������ٶȽ���
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	//pcl::io::savePCDFileASCII("bunny_transformed_sac.pcd", *sac_result);
	clock_t sac_time = clock();

	//NDT��׼
	//��ʼ����̬�ֲ��任��NDT��
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//Ϊ��ֹ����������Сת������
	ndt.setTransformationEpsilon(0.01);
	//ΪMore-Thuente������������󲽳�
	ndt.setStepSize(0.07);
	//����NDT����ṹ�ķֱ��ʣ�VoxelGridCovariance�������ظ�Ĵ�С��
	ndt.setResolution(0.02);
	//����ƥ�������������
	ndt.setMaximumIterations(50);
	// ����Ҫ��׼�ĵ���
	ndt.setInputSource(cloud_src);
	//���õ�����׼Ŀ��
	ndt.setInputTarget(cloud_tgt_o);
	//������Ҫ�ĸ���任�Ա㽫����ĵ���ƥ�䵽Ŀ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud, sac_trans);

	clock_t end = clock();
	cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << "s" << endl;
	cout << "sac time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "ndt time: " << (double)(end - sac_time) / (double)CLOCKS_PER_SEC << " s" << endl;

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()   //ŷʽ�ʺ϶�����
		<< " score: " << ndt.getFitnessScore() << std::endl;   						//����������Ƶ����Ŀ����ƶ�Ӧ���Ƶľ���ƽ����
	Eigen::Matrix4f ndt_trans;
	ndt_trans = ndt.getFinalTransformation();
	cout << "ransformationProbability" << ndt.getTransformationProbability() << endl;
	std::cout << ndt_trans << endl;
	//ʹ�ô����ı任��δ���˵�������ƽ��б任
	pcl::transformPointCloud(*cloud_src_o, *output_cloud, ndt_trans);
	//����ת�����������
	//pcl::io::savePCDFileASCII("bunny_transformed_sac_ndt.pcd", *output_cloud);

	//�������
	Eigen::Vector3f ANGLE_origin;
	ANGLE_origin << 0, 0, M_PI / 5;
	double error_x, error_y, error_z;
	Eigen::Vector3f ANGLE_result;
	matrix2angle(ndt_trans, ANGLE_result);
	error_x = fabs(ANGLE_result(0)) - fabs(ANGLE_origin(0));
	error_y = fabs(ANGLE_result(1)) - fabs(ANGLE_origin(1));
	error_z = fabs(ANGLE_result(2)) - fabs(ANGLE_origin(2));
	cout << "original angle in x y z:\n" << ANGLE_origin << endl;
	cout << "error in aixs_x: " << error_x << "  error in aixs_y: " << error_y << "  error in aixs_z: " << error_z << endl;

	//���ӻ�
	visualize_pcd(cloud_src_o, cloud_tgt_o, output_cloud);
	return (0);
}


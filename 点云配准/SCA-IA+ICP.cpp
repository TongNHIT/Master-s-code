/*******************************************************
SCA-IA粗配准+ICP精配准  20181214
20181218  配准效果很好

第一弹：SAC-IA粗配准+ICP精配准

采样一致性初始配准算法（Sample Consensus Initial Aligment , SAC-IA)
此算法依赖于点特征直方图，所以在执行此算法之前，应该先计算点云的FPFH，算法的大致思路如下：
(1) 从待配准点云P中选取n个采样点，为了尽量保证所采样的点具有不同的FPFH特征，采样点两两之间的距离应满足大于预先给定最小距离阈值d。
(2) 在目标点云Q中查找与点云P中采样点具有相似FPFH特征的一个或多个点，从这些相似点中随机选取一个点作为点云P在目标点云Q中的一一对应点。
(3) 计算对应点之间刚体变换矩阵, 然后通过求解对应点变换后的“距离误差和”函数来判断当前配准变换的性能。此处的距离误差和函数多使用Huber罚函数表示， 记为这里写图片描述，其中：
这里写图片描述

式中：mi为一预先给定值，li为第i组对应点变换之后的距离差。上述配准的最终目的是在所有变换中找到一组最优的变换，使得误差函数的值最小，此时的变换即为最终的配准变换矩阵，进一步可得到配准结果。
SAC-IA得到的变换矩阵不精确，所以它只能用于粗配准，在PCL库中的registration模块可实现SAC-IA算法。
在点数量较多时，计算FPFH特征较慢，使得SAC-IA算法效率很低，此时，需要先对点云进行下采样处理，以减少点的数量，但这会造成部分特征点丢失，使得配准准确度降低。

迭代最近点算法(Iterative Cloest Point, ICP)
ICP算法基于SVD，其大致思路如下:
(1) 将初始配准后的两片点云P′(经过坐标变换后的源点云)和Q，作为精配准的初始点集；
(2) 对源点云P’中的每一点pi，在目标点云Q中寻找距离最近的对应点qi，作为该点在目标点云中的对应点，组成初始对应点对；
(3) 初始对应点集中的对应关系并不都是正确的，错误的对应关系会影响最终的配准结果，采用方向向量阈值剔除错误的对应点对；
(4) 计算旋转矩阵R和平移向量T，使最小，即对应点集之间的均方误差最小；
(5) 设定某一阈值ε=dk-dk-1和最大迭代次数Nmax, 将上一步得到的刚体变换作用于源点云P′，得到新点云P”，计算P”和Q的距离误差,，如果两次迭代的误差小于阈值ε或者当前迭代次数大于Nmax，则迭代结束，否则将初始配准的点集更新为P”和Q，继续重复上述步骤，直至满足收敛条件。
ICP算法对参数敏感，在使用前要设置一下几个参数：
1.setMaximumIterations， 最大迭代次数，icp是一个迭代的方法，最多迭代这些次；
2. setEuclideanFitnessEpsilon， 设置收敛条件是均方误差和小于阈值，停止迭代；
3. setTransformtionEpsilon, 设置两次变化矩阵之间的差值（一般设置为1e-10即可）；
4. setMaxCorrespondenaceDistance,设置对应点对之间的最大距离（此值对配准结果影响较大）。
在两点云相差较大的情况下，ICP算法容易陷入局部最优解，从而无法得到较好的匹配结果，故需要先给定一个初始变换矩阵。在pcl库中的registration模块可实现ICP算法。

https://blog.csdn.net/peach_blossom/article/details/78506184
*******************************************************/

#define BOOST_TYPEOF_EMULATION
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <time.h>

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//点云可视化
void visualize_pcd(PointCloud::Ptr pcd_src,
	PointCloud::Ptr pcd_tgt,
	PointCloud::Ptr pcd_final)
{
	// Create a PCLVisualizer object
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud");
	viewer.addPointCloud(pcd_final, final_h, "final cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "final cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "tgt cloud");
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spin();
	}
}

//由旋转平移矩阵计算旋转角度
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

int
main(int argc, char** argv)
{
	//加载点云文件
	PointCloud::Ptr cloud_src_o(new PointCloud);//原点云，待配准
	pcl::io::loadPCDFile("E:/PCLData/pointcloud/chulihoudianyun/hezizheng.pcd", *cloud_src_o);
	PointCloud::Ptr cloud_tgt_o(new PointCloud);//目标点云
	pcl::io::loadPCDFile("E:/PCLData/pointcloud/chulihoudianyun/hezixie.pcd", *cloud_tgt_o);

	clock_t start = clock();
	//去除NAN点
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	std::cout << "remove *cloud_src_o nan" << endl;
	//下采样滤波
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(cloud_src_o);
	PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_src);
	std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;
	pcl::io::savePCDFileASCII("bunny_src_down.pcd", *cloud_src);
	//计算表面法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_src);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(0.02);
	ne_src.compute(*cloud_src_normals);

	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
	std::cout << "remove *cloud_tgt_o nan" << endl;

	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.01, 0.01, 0.01);
	voxel_grid_2.setInputCloud(cloud_tgt_o);
	PointCloud::Ptr cloud_tgt(new PointCloud);
	voxel_grid_2.filter(*cloud_tgt);
	std::cout << "down size *cloud_tgt_o.pcd from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;
//	pcl::io::savePCDFileASCII("bunny_tgt_down.pcd", *cloud_tgt);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.02);
	ne_tgt.compute(*cloud_tgt_normals);

	//计算FPFH  针对下采样之后的点云
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_src);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(0.05);
	fpfh_src.compute(*fpfhs_src);
	std::cout << "compute *cloud_src fpfh" << endl;

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(cloud_tgt);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(0.05);
	fpfh_tgt.compute(*fpfhs_tgt);
	std::cout << "compute *cloud_tgt fpfh" << endl;

	//pcl::visualization::PCLPlotter plotter;
	//plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs_tgt, "fpfh", 100);
	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	pcl::io::savePCDFileASCII("bunny_transformed_sac.pcd", *sac_result);
	clock_t sac_time = clock();

	//icp配准
	PointCloud::Ptr icp_result(new PointCloud);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt_o);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(0.04);
	// 最大迭代次数
	icp.setMaximumIterations(50);
	// 两次变化矩阵之间的差值
	icp.setTransformationEpsilon(1e-10);
	// 均方误差   设置收敛条件是均方误差和小于阈值， 停止迭代
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.align(*icp_result, sac_trans);

	clock_t end = clock();
	cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	//我把计算法线和点特征直方图的时间也算在SAC里面了
	cout << "sac time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "icp time: " << (double)(end - sac_time) / (double)CLOCKS_PER_SEC << " s" << endl;

	std::cout << "ICP has converged:" << icp.hasConverged()
		<< " score: " << icp.getFitnessScore() << std::endl;
	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	//cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
	std::cout << icp_trans << endl;
	//使用创建的变换对未过滤的输入点云进行变换
	pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
	//保存转换的输入点云
	pcl::io::savePCDFileASCII("bunny_transformed_sac_ndt.pcd", *icp_result);

	//计算误差
	Eigen::Vector3f ANGLE_origin;
	ANGLE_origin << 0, 0, M_PI / 5;
	double error_x, error_y, error_z;
	Eigen::Vector3f ANGLE_result;
	matrix2angle(icp_trans, ANGLE_result);
	error_x = fabs(ANGLE_result(0)) - fabs(ANGLE_origin(0));
	error_y = fabs(ANGLE_result(1)) - fabs(ANGLE_origin(1));
	error_z = fabs(ANGLE_result(2)) - fabs(ANGLE_origin(2));
	cout << "original angle in x y z:\n" << ANGLE_origin << endl;
	cout << "error in aixs_x: " << error_x << "  error in aixs_y: " << error_y << "  error in aixs_z: " << error_z << endl;

	//可视化
	visualize_pcd(cloud_src_o, cloud_tgt_o, icp_result);
	//visualize_pcd(cloud_src_o, cloud_src_o, fpfhs_src);

	return (0);
}

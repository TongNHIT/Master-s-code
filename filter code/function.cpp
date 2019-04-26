#include "Mainwindows.h"
///////////////////////////////////////////点云处理/////////////////////////////////////////////////////////////

//利用统计学方法去除点云大噪声，k为k搜索之，t为保存几倍方差之内的点
void reducewrongpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout, int k, int t)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloudin);
	sor.setMeanK(k);//搜索k个临近点
	sor.setStddevMulThresh(t);//距离大于t倍标准方差
	sor.filter(*cloudout);
}

//半径滤波
void radiusfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double r, int a)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud_in);
	outrem.setRadiusSearch(r);
	outrem.setMinNeighborsInRadius(a);
	// apply filter
	outrem.filter(*cloud_out);
}
//直通滤波
void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 0.5);
	pass.setFilterLimitsNegative(true);
	pass.filter(*cloud_out);
}

//条件滤波
void tiaojian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond_cptr(new pcl::ConditionAnd<pcl::PointXYZ>());//创建条件定义对象
	range_cond_cptr->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 2.5)));//为条件定义对象添加比较算子
	range_cond_cptr->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 10)));
	
	pcl::ConditionalRemoval<pcl::PointXYZ> conditrem;//创建条件滤波器
	conditrem.setCondition(range_cond_cptr);        //并用条件定义对象初始化            
	conditrem.setInputCloud(cloud_in);             //输入点云
	conditrem.setKeepOrganized(true);                //设置保持点云的结构
	// 执行滤波
	conditrem.filter(*cloud_out);           //大于0.0小于0.8这两个条件用于建立滤波器


}

//VoxelGrid下采样
void VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud_in, *cloud_blob);
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //创建滤波对象
	sor.setInputCloud(cloud_blob);            //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.2f, 0.2f, 0.2f);  //设置滤波时创建的体素体积为1cm的立方体
	sor.filter(*cloud_filtered_blob);           //执行滤波处理，存储输出
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_out);
	
}

//点云的变化检测（尺寸、分辨率、密度、点顺序）
void cloudchange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB)
{
	srand((unsigned int)time(NULL));
	// 八叉树分辨率 即体素的大小
	float resolution = 32.0f;
	// 初始化空间变化检测对象
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>octree(resolution);
	//添加点云到八叉树，建立八叉树
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();
	// 交换八叉树缓存，但是cloudA对应的八叉树仍在内存中
	octree.switchBuffers();
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();
	std::vector<int>newPointIdxVector;
	//获取前一cloudA对应的八叉树在cloudB对应八叉树中没有的体素
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	std::cout << newPointIdxVector.size() << " points changed!\n";
}

//基于法线的滤波
void filterbynormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double scale1, double scale2, double threshold)
{//5,15,0.05
	using namespace pcl;
	using namespace std;
	//创建Kdtree来进行邻域搜索
	pcl::search::Search<PointXYZ>::Ptr tree;
	if (cloud_in->isOrganized())
	{
		tree.reset(new pcl::search::OrganizedNeighbor<PointXYZ>());
	}
	else
	{
		tree.reset(new pcl::search::KdTree<PointXYZ>(false));
	}
	tree->setInputCloud(cloud_in);// Set the input pointcloud for the search tree
	
	pcl::NormalEstimation<pcl::PointXYZ, PointNormal> ne;//生成法线估计器
	ne.setInputCloud(cloud_in);
	ne.setSearchMethod(tree);
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());//设定法线方向（要做差，同向很重要）
	cout << "计算小尺度法线...\n";
	pcl::PointCloud<PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<PointNormal>);
	//ne.setRadiusSearch(scale1);
	ne.setKSearch(scale1);//5
	ne.compute(*normals_small_scale);
	cout << "计算大尺度法线...\n";
	pcl::PointCloud<PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<PointNormal>);
	//ne.setRadiusSearch(scale2);
	ne.setKSearch(scale2);//15
	ne.compute(*normals_large_scale);
	// Create output cloud for DoN results
	PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
	copyPointCloud<PointXYZ, PointNormal>(*cloud_in, *doncloud);
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, PointNormal, PointNormal> don;//生成DoN分割器
	don.setInputCloud(cloud_in);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);
	if (!don.initCompute())
	{
		std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}
	cout << "计算法线差...\n";
	don.computeFeature(*doncloud);
	pcl::PCDWriter writer;// Save DoN features
	writer.write<pcl::PointNormal>("don.pcd", *doncloud, false);
	cout << "进行条件滤波...\n";//生成滤波条件：把法线差和阈值比
	pcl::ConditionOr<PointNormal>::Ptr range_cond(
		new pcl::ConditionOr<PointNormal>()
	);
	range_cond->addComparison(pcl::FieldComparison<PointNormal>::ConstPtr(
		new pcl::FieldComparison<PointNormal>("curvature", pcl::ComparisonOps::GT, threshold))
	);
	//生成条件滤波器,输入滤波条件和点云
    pcl::ConditionalRemoval <PointNormal> condrem; //创建条件滤波器
	condrem.setCondition(range_cond);
	condrem.setInputCloud(doncloud);
	//condrem.setKeepOrganized(true);//这个是保证是不是用空白点替代原始点云
	//导出滤波结果
	pcl::PointCloud<PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<PointNormal>);
	condrem.filter(*doncloud_filtered);
	copyPointCloud<PointNormal, pcl::PointXYZ>(*doncloud_filtered, *cloud_out);
	}
	

//利用PCA获得曲率和法线
void getnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::Normal>::Ptr normal, int k)
{
	//创建评估类
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(xyz);
	//创建Kdtree来进行邻域搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(xyz);//为kdtree添加点云数据
	ne.setInputCloud(xyz);//将点云和kdtree放入评估类
	ne.setSearchMethod(tree);
	ne.setKSearch(k);//在20个点的邻域内邻域范围内新型法线和曲率估算
	ne.compute(*normal);//进行计算法向量
}


//双边算法
void shuangbian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const pcl::PointCloud<pcl::Normal>::Ptr normal)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud_in);
	for (size_t i = 0; i < cloud_in->size(); i++)
	{
		pcl::PointXYZ searchpoint;
		searchpoint.x = cloud_in->points[i].x;
		searchpoint.y = cloud_in->points[i].y;
		searchpoint.z = cloud_in->points[i].z;

		int k =10 ;

		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquareDistance;
		tree.nearestKSearch(searchpoint, k, pointIdxNKNSearch, pointNKNSquareDistance);
		double sum1 = 0, sum2 = 0;
		for (int j = 0; j < pointIdxNKNSearch.size(); j++)
		{
			//两点间空间距离（pi-pj）
			double dp = sqrt((cloud_in->points[i].x - cloud_in->points[pointIdxNKNSearch[j]].x)* (cloud_in->points[i].x - cloud_in->points[pointIdxNKNSearch[j]].x) +
				(cloud_in->points[i].y - cloud_in->points[pointIdxNKNSearch[j]].y)*(cloud_in->points[i].y - cloud_in->points[pointIdxNKNSearch[j]].y) +
				(cloud_in->points[i].z - cloud_in->points[pointIdxNKNSearch[j]].z)*(cloud_in->points[i].z - cloud_in->points[pointIdxNKNSearch[j]].z));
			//向量相乘(n*(pi-pj))
			double np = normal->points[i].normal_x*(cloud_in->points[pointIdxNKNSearch[j]].x - cloud_in->points[i].x) +
				normal->points[i].normal_y*(cloud_in->points[pointIdxNKNSearch[j]].y - cloud_in->points[i].y) +
				normal->points[i].normal_z*(cloud_in->points[pointIdxNKNSearch[j]].z - cloud_in->points[i].z);
			//法向量的点积
			double nn = normal->points[i].normal_x*normal->points[pointIdxNKNSearch[j]].normal_x + normal->points[i].normal_y*normal->points[pointIdxNKNSearch[j]].normal_y +
			normal->points[i].normal_z*normal->points[pointIdxNKNSearch[j]].normal_z;
			//double wc = exp(-dp*dp / 2 * 0.0001);
			//double ws1 = exp(-np*np / 2 * 0.0005);
			//double ws2 = exp(-nn*nn / 2 * 0.0005);
			//sum1 = sum1 + wc*ws1*nn;
			//sum2 = sum2 + wc*ws2;
		
			
			//double nn = normal->points[i].normal_x*normal->points[pointIdxNKNSearch[j]].normal_x + normal->points[i].normal_y*normal->points[pointIdxNKNSearch[j]].normal_y +
			//normal->points[i].normal_z*normal->points[pointIdxNKNSearch[j]].normal_z;
			double wc = exp(-dp * dp / 2 * 0.25);//标准方差取0.5方，sigma_s
			double ws1 = exp(-np * np / 2 * 0.03*0.03);//sigma_r
			double ws2 = exp(-(nn-1)* (nn-1) / 2 * 0.0005);
			sum1 = sum1 + wc * ws2*np;
			sum2 = sum2 + wc * ws2;
		}
		double d = sum1 / sum2;
		//cout << "d= "<< d << endl;
		*cloud_out = *cloud_in;
		cloud_out->points[i].x = cloud_in->points[i].x + normal->points[i].normal_x*d;
		cloud_out->points[i].y = cloud_in->points[i].y + normal->points[i].normal_y*d;
		cloud_out->points[i].z = cloud_in->points[i].z + normal->points[i].normal_z*d;
	}
}


//三边算法,输入为彩色点云和Normal
void sanbian(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, const pcl::PointCloud<pcl::Normal>::Ptr PCLnormal)
{
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
	kdtree.setInputCloud(cloud_in);
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		pcl::PointXYZRGBA searchpoint;
		searchpoint.x = cloud_in->points[i].x;
		searchpoint.y = cloud_in->points[i].y;
		searchpoint.z = cloud_in->points[i].z;
		searchpoint.r = cloud_in->points[i].r;
		searchpoint.g = cloud_in->points[i].b;
		searchpoint.b = cloud_in->points[i].b;
		searchpoint.a = cloud_in->points[i].a;
		int k = 10;

		//float raduis = 0.5;
		std::vector<int>pointIdxNKNSearch;
		std::vector<float>pointNKNSquareDistance;
		kdtree.nearestKSearch(searchpoint, k, pointIdxNKNSearch, pointNKNSquareDistance);
		//kdtree.radiusSearch(searchpoint, raduis, pointIdxNKNSearch, pointNKNSquareDistance);
		double f2 = 0; double tt = 0;
		int size = pointIdxNKNSearch.size();
		float p[31];
		for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
		{
			//计算标准差
			double bzc = 0;
			for (size_t k = 0; k < pointIdxNKNSearch.size(); ++k)
			{
				bzc = bzc + sqrt(abs(pointNKNSquareDistance[k]));
			}
			double bbzzcc = 0;
			for (size_t kk = 0; kk < pointIdxNKNSearch.size(); ++kk)
			{
				bbzzcc = bbzzcc + (sqrt(abs(pointNKNSquareDistance[kk])) - (bzc / pointIdxNKNSearch.size()));
			}
			bbzzcc = sqrt(abs(bbzzcc) / pointIdxNKNSearch.size());
			double bbzzcc2 = bbzzcc;

			//找出曲率的最大最小值
			/*int size = pointIdxNKNSearch.size();
			float *p = new float[size];*/
			for (size_t ii = 0; ii < pointIdxNKNSearch.size(); ++ii)
			{
				p[ii] = PCLnormal->points[pointIdxNKNSearch[ii]].curvature;
			}
			//排序
			for (int jj = 0; jj < 10; jj++)   /* 气泡法要排序n次*/
			{
				for (int iii = 0; iii < 10 - jj; iii++)  /* 值比较大的元素沉下去后，只把剩下的元素中的最大值再沉下去就可以啦 */
				{
					if (p[iii] > p[iii + 1])  /* 把值比较大的元素沉到底 */
					{
						float k = p[iii];
						p[iii] = p[iii + 1];
						p[iii + 1] = k;
					}
				}
			}
			//std::cout << *(p) << "  " << *(p + size) << "  " << *(p + 1) << std::endl;
			double G1 = exp(-((pointNKNSquareDistance[j])) / 0.5);//ui->doubleSpinBoxThreeD1->value()
			//std::cout << G1 << std::endl;
			float bb[3] = { PCLnormal->points[i].normal_x, PCLnormal->points[i].normal_y, PCLnormal->points[i].normal_z };
			std::vector<float>normal(bb, bb + 3);
			//normal.push_back = normal_changed->points[i].normal_x;
			//normal.push_back = normal_changed->points[i].normal_y;
			//normal.push_back = normal_changed->points[i].normal_z;
			double xx = searchpoint.x;
			double yy = searchpoint.y;
			double zz = searchpoint.z;
			double xjx = cloud_in->points[pointIdxNKNSearch[j]].x;
			double yjy = cloud_in->points[pointIdxNKNSearch[j]].y;

			//这个地方的搜索点的近邻点的Z坐标一直是无效的；
			double zjz = cloud_in->points[pointIdxNKNSearch[j]].z;
			//double zjz = 0.3;
			//std::cout << zjz << std::endl;
			double cc[3] = { xx - xjx, yy - yjy, zz - zjz };//这里的第三个值不是zz-zjz了，一会要改过来看看行不行
			std::vector<double>normal_point(cc, cc + 3);
			//std::cout <<normal_point[2] << std::endl;
			float dd[3] = { PCLnormal->points[pointIdxNKNSearch[j]].normal_x, PCLnormal->points[pointIdxNKNSearch[j]].normal_y, PCLnormal->points[pointIdxNKNSearch[j]].normal_z };
			std::vector<float>normal_near(dd, dd + 3);
			//normal_near.push_back = normal_changed->points[pointIdxNKNSearch[j]].normal_x;
			//normal_near.push_back = normal_changed->points[pointIdxNKNSearch[j]].normal_y;
			//normal_near.push_back = normal_changed->points[pointIdxNKNSearch[j]].normal_z;
			//下面要计算G3
			//法线以及近邻点法线的夹角
			double d = normal[0] * normal_near[0] + normal[1] * normal_near[1] + normal[2] * normal_near[2];
			/*double length = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
			double length_near = sqrt(normal_near[0] * normal_near[0] + normal_near[1] * normal_near[1] + normal_near[2] * normal_near[2]);
			d = d / (length*length_near);*/
			//法线和法线与点之间的夹角
			double nj = normal[0] * normal_point[0] + normal[1] * normal_point[1] + normal[2] * normal_point[2];
			/*double nn_pp = sqrt(normal_point[0] * normal_point[0] + normal_point[1] * normal_point[1] + normal_point[2] * normal_point[2]);
			double angle = nj / (length*nn_pp);*/
			//std::cout << angle << std::endl;
			//angle = acos(angle);
			double G3 = 0;
			if (d > 1)
				//ui->doubleSpinBoxThreeT->value()这里的的他3选的是1
			{
				//G3 = exp(-angle*angle / 2 * bbzzcc*bbzzcc);
				G3 = exp(-(nj*nj) / 2 * bbzzcc*bbzzcc);
			}
			//std::cout << G3 << std::endl;
			double h = (PCLnormal->points[i].curvature - p[0]) /
				(p[29] - p[0]);
			//std::cout << h << std::endl;
			f2 = f2 + G1 * G3*(1 + h);
			//tt = tt + angle*G1*G3*(1 + h);
			tt = tt + nj * G1*G3*(1 + h);
			//std::cout << f2 << std::endl;
			//std::cout << tt << std::endl;
		}
		if (f2 == 0 || tt == 0)
		{
			continue;
		}
		double ff2 = f2;
		double tt2 = tt;
		cloud_in->points[i].x = cloud_in->points[i].x + (PCLnormal->points[i].normal_x / f2)*tt;
		cloud_in->points[i].y = cloud_in->points[i].y + (PCLnormal->points[i].normal_y / f2)*tt;
		//cloud_points->points[i].x = cloud_points->points[i].x;
		//cloud_points->points[i].y = cloud_points->points[i].y;
		cloud_in->points[i].z = cloud_in->points[i].z + (PCLnormal->points[i].normal_z / f2)*tt;
		cloud_in->points[i].r = cloud_in->points[i].r;
		cloud_in->points[i].g = cloud_in->points[i].g;
		cloud_in->points[i].b = cloud_in->points[i].b;
		cloud_in->points[i].a = cloud_in->points[i].a;
		//delete[] p;
	}
}


//修正那个点云normal
void normalchange(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, pcl::PointCloud<pcl::Normal>::Ptr normal_out)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_points);
	for (size_t i = 0; i < cloud_points->points.size(); ++i)
	{
		pcl::PointXYZ searchpoint;
		searchpoint.x = cloud_points->points[i].x;
		searchpoint.y = cloud_points->points[i].y;
		searchpoint.z = cloud_points->points[i].z;
		//searchpoint.r = cloud_points->points[i].r;
		//searchpoint.g = cloud_points->points[i].b;
		//searchpoint.b = cloud_points->points[i].b;
		//searchpoint.a = cloud_points->points[i].a;
		int k = 10;
		//float raduis = 0.5;
		std::vector<int>pointIdxNKNSearch;
		std::vector<float>pointNKNSquareDistance;
		kdtree.nearestKSearch(searchpoint, k, pointIdxNKNSearch, pointNKNSquareDistance);
		//kdtree.radiusSearch(searchpoint, raduis, pointIdxNKNSearch, pointNKNSquareDistance);
		double tt = 0, f1 = 0;
		//计算空间光顺高斯函数
		int size = pointIdxNKNSearch.size();
		int s = size;
		//float p[s] = { 0 };
		float p[30] = { 0 };//这样的话只能是都改成30了，不能用pointIdxNKNSearch.size()了
		for (int j = 0; j < pointIdxNKNSearch.size(); j++)
		{
			double G1 = exp(-((pointNKNSquareDistance[j])) / 0.25);
			//计算这个G2特征权函数
			//normal.push_back = cloud_normal->points[i].normal_x;
			//normal.push_back = cloud_normal->points[i].normal_y;
			//normal.push_back = cloud_normal->points[i].normal_z;
			float bb[3] = { cloud_normal->points[i].normal_x, cloud_normal->points[i].normal_y, cloud_normal->points[i].normal_z };
			std::vector<float>normal(bb, bb + 3);
			float cc[3] = { cloud_normal->points[pointIdxNKNSearch[j]].normal_x, cloud_normal->points[pointIdxNKNSearch[j]].normal_y, cloud_normal->points[pointIdxNKNSearch[j]].normal_z };
			std::vector<float>normal_near(cc, cc + 3);

			//normal_near.push_back = cloud_normal->points[pointIdxNKNSearch[j]].normal_x;
			//normal_near.push_back = cloud_normal->points[pointIdxNKNSearch[j]].normal_y;
			//normal_near.push_back = cloud_normal->points[pointIdxNKNSearch[j]].normal_z;
			double d = normal[0] * normal_near[0] + normal[1] * normal_near[1] + normal[2] * normal_near[2];
			//法向量距离的平方
			double distance = ((normal[0] - normal_near[0])*(normal[0] - normal_near[0]) + (normal[1] - normal_near[1])*(normal[1] - normal_near[1]) +
				(normal[2] - normal_near[2])*(normal[2] - normal_near[2]));
			double G2 = 0;
			if (d > 10)
				//这个地方的有一个因子这个是随便取得，为0.3；
			{
				G2 = exp(-distance / 0.3);
			}
			//能不能把curvature给向量然后找最大最小值
			//计算hi曲率影响因子
			//int size = pointIdxNKNSearch.size();
			//float p[size];//这里可能有问题，
			for (size_t ii = 0; ii < 30; ++ii)
			{
				p[ii] = cloud_normal->points[pointIdxNKNSearch[ii]].curvature;
			}
			//排序
			for (int j = 0; j < 30; j++)   /* 气泡法要排序n次*/
			{
				for (int i = 0; i < 30 - j; i++)  /* 值比较大的元素沉下去后，只把剩下的元素中的最大值再沉下去就可以啦 */
				{
					if (p[i] > p[i + 1])  /* 把值比较大的元素沉到底 */
					{
						float k = p[i];
						p[i] = p[i + 1];
						p[i + 1] = k;
					}
				}
			}
			double h = (cloud_normal->points[i].curvature - p[0]) /
				(p[29] - p[0]);
			//计算正则化因f1；
			//double f1 = 0;
			f1 = f1 + G1 * G2*(1 + h);
			//计算修正后的法线
			//double tt = 0;
			tt = tt + sqrt(distance)*G1*G2*(1 + h);

		}
		cloud_normal->points[i].normal_x = cloud_normal->points[i].normal_x + (1 / f1)*tt;
		cloud_normal->points[i].normal_y = cloud_normal->points[i].normal_y + (1 / f1)*tt;
		cloud_normal->points[i].normal_z = cloud_normal->points[i].normal_z + (1 / f1)*tt;
		//delete[] p;
	}
	*normal_out = *cloud_normal;
	//std::cout << "ll" << std::endl;
}

void normalrefine(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points,pcl::PointCloud<pcl::Normal>::Ptr cloud_normal,pcl::PointCloud<pcl::Normal>::Ptr normal_out)
{
	//kdtree search
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_points);
	int k = 10;
	std::vector<std::vector<int> > k_indices;
	std::vector<std::vector<float> > k_sqr_distances;
	// Run search
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud_points);
	tree->nearestKSearch(*cloud_points, std::vector<int>(), k, k_indices, k_sqr_distances);

	unsigned int max_iterations = 10;
	for (unsigned int it = 0; it < max_iterations; ++it)
	{
		pcl::PointCloud<pcl::Normal>temp = *cloud_normal;
		for (size_t i = 0; i < cloud_points->points.size(); ++i)
		{
			pcl::Normal tempnorm;
			tempnorm.normal_x = 0.0f;
			tempnorm.normal_y = 0.0f;
			tempnorm.normal_z = 0.0f;
			double G = 0;  //归一化系数（分母）
			//计算空间光顺高斯函数
			for (int j = 0; j < k_indices[i].size(); j++)
			{
				//计算G1
				double G1 = exp(-(k_sqr_distances[i][j]) / 2.25);  //2.25为待定系数
				//计算G2
				//法向量乘积nxn
				double nxn = cloud_normal->points[i].normal_x * cloud_normal->points[k_indices[i][j]].normal_x
					+ cloud_normal->points[i].normal_y * cloud_normal->points[k_indices[i][j]].normal_y
					+ cloud_normal->points[i].normal_z * cloud_normal->points[k_indices[i][j]].normal_z;
				double G2 = exp(-(1 - nxn)*(1 - nxn) / (1 - cos(M_PI / 12))*(1 - cos(M_PI / 12)));
				G += G1 * G2;
				tempnorm.normal_x += cloud_normal->points[k_indices[i][j]].normal_x * G1 * G2;
				tempnorm.normal_y += cloud_normal->points[k_indices[i][j]].normal_y * G1 * G2;
				tempnorm.normal_z += cloud_normal->points[k_indices[i][j]].normal_z * G1 * G2;
			}
			tempnorm.normal_x = tempnorm.normal_x / G;
			tempnorm.normal_y = tempnorm.normal_y / G;
			tempnorm.normal_z = tempnorm.normal_z / G;
			//normalize...
			const float norm = std::sqrt(tempnorm.normal_x * tempnorm.normal_x +
				tempnorm.normal_y * tempnorm.normal_y +
				tempnorm.normal_z * tempnorm.normal_z);
			temp.points[i].normal_x = tempnorm.normal_x / norm;
			temp.points[i].normal_y = tempnorm.normal_y / norm;
			temp.points[i].normal_z = tempnorm.normal_z / norm;
		}
		*cloud_normal = temp;
	}
	*normal_out = *cloud_normal;
}



//去除平面(距离、迭代、概率)
void reduceplant(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double d, int Iteration, int Probability)
{

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //存储输出的模型的系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //存储内点，使用的点
	//创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//可选设置
	seg.setOptimizeCoefficients(true);
	//必须设置
	seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
	seg.setMethodType(pcl::SAC_RANSAC);      //设置方法【聚类或随机样本一致性】
	seg.setDistanceThreshold(d);   //0.02距离阈值
	seg.setInputCloud(cloud_in);
	seg.setMaxIterations(Iteration);//1000最大迭代次数
	seg.setProbability(Probability);//0.95概率
	seg.segment(*inliers, *coefficients);    //分割操作
	if (inliers->indices.size() == 0)//根据内点数量，判断是否成功
	{

		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}
	pcl::ExtractIndices<pcl::PointXYZ> extract;//创建点云提取对象
	extract.setInputCloud(cloud_in);//设置输入的点云
	extract.setIndices(inliers);//设置分割后的内点位需要的点集
	extract.setNegative(true);//设置提取的是外点（False是内点）
	extract.filter(*cloud_out);//输出储存位置
}


/////////////////////////////////////////点云显示//////////////////////////////////////////////////////
//为一个仅执行一次的函数，作用为设置背景，放置球体。调用方法为：viewer.runOnVisualizationThreadOnce (viewerOneOff)
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

//作为回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串，调用方法是viewer.runOnVisualizationThread (viewerPsycho);每次刷新都调用
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);
	//FIXME: possible race condition here:  
	user_data++;
}

//在两个窗口中显示自定义颜色的
void showviewintwodifferentwindow(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right)//输入两个XYZRGBA
{
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
	viewer.setBackgroundColor(1, 1, 1, v1);
	viewer.addCoordinateSystem(1.0f, "globalleft", v1);//红色是 X 轴；绿色是Y轴；蓝色是Z轴
	viewer.addText("Original point cloud", 10, 10,0,0,0, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_left, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_left, single_color, "cloud_left", v1);
	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1, v2);
	viewer.setBackgroundColor(1, 1, 1, v2);
	viewer.addCoordinateSystem(1.0f, "globalright", v2);
	viewer.addText("Point cloud after processing", 10, 10,0,0,0, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud_right, 0, 0, 255);

	viewer.addPointCloud<pcl::PointXYZ>(cloud_right, single_color1, "cloud_right", v2);

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_left");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_right");

	while (!viewer.wasStopped())
	{
		//在此处可以添加其他处理  
		user_data++;
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

//在4个窗口中显示相同角度的彩色点云XYZRGBA
void showviewinfourdifferentwindow(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4)//输入两个XYZRGBA
{

	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);// 4个参数分别是X轴的最小值，Y轴的最小值,X轴最大值，Y轴最大值
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud1, "cloud1", v1);

	int v2(0);
	viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud2, "cloud2", v2);

	int v3(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
	viewer.setBackgroundColor(0, 0, 0, v3);
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud3, "cloud3", v3);

	int v4(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);
	viewer.setBackgroundColor(0, 0, 0, v4);
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud4, "cloud4", v4);

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud3");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud4");


	while (!viewer.wasStopped())
	{
		//在此处可以添加其他处理  
		user_data++;
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

//在一个窗口内显示彩色点云和曲率法线(RGBA，XYZ，Normal)
void showcloudwithnormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	//显示设置
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);     
	//添加原来的点云
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud");
	//设置点显示大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	// //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，1表示法向长度。
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloudxyz, cloud_normals, 20, 0.015, "normals");

	while (!viewer->wasStopped())
	{
		//在此处可以添加其他处理  
		user_data++;
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void showrgbcloudwithnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer_ptr->setBackgroundColor(0.0, 0.0, 0.0);//背景黑色,(1,1,1)白色
	viewer_ptr->addCoordinateSystem(1.0f, "global");//坐标系
	//viewer_ptr->setCameraPosition(0,0,200); //设置坐标原点
	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud); 
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
	//pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cloud_color_handler(cloud2);
	//该句的意思是：对输入的点云着色，Random表示的是随机上色，以上是其他两种渲染色彩的方式.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 255, 0);//红色
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr,cloud_color_handler,"sample cloud");//PointXYZRGB 类型点
	viewer_ptr->addPointCloud<pcl::PointXYZ>(cloud, cloud_color_handler, "original point cloud");//点云标签

	viewer_ptr->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 5, 0.02, "normal");//法线标签
  //其中，参数5表示整个点云中每5各点显示一个法向量（若全部显示，可设置为1，  0.02表示法向量的长度，最后一个参数暂时还不知道 如何影响的）);
	viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original point cloud");
	viewer_ptr->initCameraParameters();//初始化相机参数
	while (!viewer_ptr->wasStopped())
	{
		viewer_ptr->spinOnce();
		pcl_sleep(0.01);
	}
}

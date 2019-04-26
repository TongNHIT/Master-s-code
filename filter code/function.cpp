#include "Mainwindows.h"
///////////////////////////////////////////���ƴ���/////////////////////////////////////////////////////////////

//����ͳ��ѧ����ȥ�����ƴ�������kΪk����֮��tΪ���漸������֮�ڵĵ�
void reducewrongpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout, int k, int t)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloudin);
	sor.setMeanK(k);//����k���ٽ���
	sor.setStddevMulThresh(t);//�������t����׼����
	sor.filter(*cloudout);
}

//�뾶�˲�
void radiusfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double r, int a)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud_in);
	outrem.setRadiusSearch(r);
	outrem.setMinNeighborsInRadius(a);
	// apply filter
	outrem.filter(*cloud_out);
}
//ֱͨ�˲�
void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 0.5);
	pass.setFilterLimitsNegative(true);
	pass.filter(*cloud_out);
}

//�����˲�
void tiaojian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond_cptr(new pcl::ConditionAnd<pcl::PointXYZ>());//���������������
	range_cond_cptr->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 2.5)));//Ϊ�������������ӱȽ�����
	range_cond_cptr->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 10)));
	
	pcl::ConditionalRemoval<pcl::PointXYZ> conditrem;//���������˲���
	conditrem.setCondition(range_cond_cptr);        //����������������ʼ��            
	conditrem.setInputCloud(cloud_in);             //�������
	conditrem.setKeepOrganized(true);                //���ñ��ֵ��ƵĽṹ
	// ִ���˲�
	conditrem.filter(*cloud_out);           //����0.0С��0.8�������������ڽ����˲���


}

//VoxelGrid�²���
void VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud_in, *cloud_blob);
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //�����˲�����
	sor.setInputCloud(cloud_blob);            //������Ҫ���˵ĵ��Ƹ��˲�����
	sor.setLeafSize(0.2f, 0.2f, 0.2f);  //�����˲�ʱ�������������Ϊ1cm��������
	sor.filter(*cloud_filtered_blob);           //ִ���˲������洢���
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_out);
	
}

//���Ƶı仯��⣨�ߴ硢�ֱ��ʡ��ܶȡ���˳��
void cloudchange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB)
{
	srand((unsigned int)time(NULL));
	// �˲����ֱ��� �����صĴ�С
	float resolution = 32.0f;
	// ��ʼ���ռ�仯������
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>octree(resolution);
	//��ӵ��Ƶ��˲����������˲���
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();
	// �����˲������棬����cloudA��Ӧ�İ˲��������ڴ���
	octree.switchBuffers();
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();
	std::vector<int>newPointIdxVector;
	//��ȡǰһcloudA��Ӧ�İ˲�����cloudB��Ӧ�˲�����û�е�����
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	std::cout << newPointIdxVector.size() << " points changed!\n";
}

//���ڷ��ߵ��˲�
void filterbynormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double scale1, double scale2, double threshold)
{//5,15,0.05
	using namespace pcl;
	using namespace std;
	//����Kdtree��������������
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
	
	pcl::NormalEstimation<pcl::PointXYZ, PointNormal> ne;//���ɷ��߹�����
	ne.setInputCloud(cloud_in);
	ne.setSearchMethod(tree);
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());//�趨���߷���Ҫ���ͬ�����Ҫ��
	cout << "����С�߶ȷ���...\n";
	pcl::PointCloud<PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<PointNormal>);
	//ne.setRadiusSearch(scale1);
	ne.setKSearch(scale1);//5
	ne.compute(*normals_small_scale);
	cout << "�����߶ȷ���...\n";
	pcl::PointCloud<PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<PointNormal>);
	//ne.setRadiusSearch(scale2);
	ne.setKSearch(scale2);//15
	ne.compute(*normals_large_scale);
	// Create output cloud for DoN results
	PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
	copyPointCloud<PointXYZ, PointNormal>(*cloud_in, *doncloud);
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, PointNormal, PointNormal> don;//����DoN�ָ���
	don.setInputCloud(cloud_in);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);
	if (!don.initCompute())
	{
		std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}
	cout << "���㷨�߲�...\n";
	don.computeFeature(*doncloud);
	pcl::PCDWriter writer;// Save DoN features
	writer.write<pcl::PointNormal>("don.pcd", *doncloud, false);
	cout << "���������˲�...\n";//�����˲��������ѷ��߲����ֵ��
	pcl::ConditionOr<PointNormal>::Ptr range_cond(
		new pcl::ConditionOr<PointNormal>()
	);
	range_cond->addComparison(pcl::FieldComparison<PointNormal>::ConstPtr(
		new pcl::FieldComparison<PointNormal>("curvature", pcl::ComparisonOps::GT, threshold))
	);
	//���������˲���,�����˲������͵���
    pcl::ConditionalRemoval <PointNormal> condrem; //���������˲���
	condrem.setCondition(range_cond);
	condrem.setInputCloud(doncloud);
	//condrem.setKeepOrganized(true);//����Ǳ�֤�ǲ����ÿհ׵����ԭʼ����
	//�����˲����
	pcl::PointCloud<PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<PointNormal>);
	condrem.filter(*doncloud_filtered);
	copyPointCloud<PointNormal, pcl::PointXYZ>(*doncloud_filtered, *cloud_out);
	}
	

//����PCA������ʺͷ���
void getnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::Normal>::Ptr normal, int k)
{
	//����������
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(xyz);
	//����Kdtree��������������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(xyz);//Ϊkdtree��ӵ�������
	ne.setInputCloud(xyz);//�����ƺ�kdtree����������
	ne.setSearchMethod(tree);
	ne.setKSearch(k);//��20���������������Χ�����ͷ��ߺ����ʹ���
	ne.compute(*normal);//���м��㷨����
}


//˫���㷨
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
			//�����ռ���루pi-pj��
			double dp = sqrt((cloud_in->points[i].x - cloud_in->points[pointIdxNKNSearch[j]].x)* (cloud_in->points[i].x - cloud_in->points[pointIdxNKNSearch[j]].x) +
				(cloud_in->points[i].y - cloud_in->points[pointIdxNKNSearch[j]].y)*(cloud_in->points[i].y - cloud_in->points[pointIdxNKNSearch[j]].y) +
				(cloud_in->points[i].z - cloud_in->points[pointIdxNKNSearch[j]].z)*(cloud_in->points[i].z - cloud_in->points[pointIdxNKNSearch[j]].z));
			//�������(n*(pi-pj))
			double np = normal->points[i].normal_x*(cloud_in->points[pointIdxNKNSearch[j]].x - cloud_in->points[i].x) +
				normal->points[i].normal_y*(cloud_in->points[pointIdxNKNSearch[j]].y - cloud_in->points[i].y) +
				normal->points[i].normal_z*(cloud_in->points[pointIdxNKNSearch[j]].z - cloud_in->points[i].z);
			//�������ĵ��
			double nn = normal->points[i].normal_x*normal->points[pointIdxNKNSearch[j]].normal_x + normal->points[i].normal_y*normal->points[pointIdxNKNSearch[j]].normal_y +
			normal->points[i].normal_z*normal->points[pointIdxNKNSearch[j]].normal_z;
			//double wc = exp(-dp*dp / 2 * 0.0001);
			//double ws1 = exp(-np*np / 2 * 0.0005);
			//double ws2 = exp(-nn*nn / 2 * 0.0005);
			//sum1 = sum1 + wc*ws1*nn;
			//sum2 = sum2 + wc*ws2;
		
			
			//double nn = normal->points[i].normal_x*normal->points[pointIdxNKNSearch[j]].normal_x + normal->points[i].normal_y*normal->points[pointIdxNKNSearch[j]].normal_y +
			//normal->points[i].normal_z*normal->points[pointIdxNKNSearch[j]].normal_z;
			double wc = exp(-dp * dp / 2 * 0.25);//��׼����ȡ0.5����sigma_s
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


//�����㷨,����Ϊ��ɫ���ƺ�Normal
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
			//�����׼��
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

			//�ҳ����ʵ������Сֵ
			/*int size = pointIdxNKNSearch.size();
			float *p = new float[size];*/
			for (size_t ii = 0; ii < pointIdxNKNSearch.size(); ++ii)
			{
				p[ii] = PCLnormal->points[pointIdxNKNSearch[ii]].curvature;
			}
			//����
			for (int jj = 0; jj < 10; jj++)   /* ���ݷ�Ҫ����n��*/
			{
				for (int iii = 0; iii < 10 - jj; iii++)  /* ֵ�Ƚϴ��Ԫ�س���ȥ��ֻ��ʣ�µ�Ԫ���е����ֵ�ٳ���ȥ�Ϳ����� */
				{
					if (p[iii] > p[iii + 1])  /* ��ֵ�Ƚϴ��Ԫ�س����� */
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

			//����ط���������Ľ��ڵ��Z����һֱ����Ч�ģ�
			double zjz = cloud_in->points[pointIdxNKNSearch[j]].z;
			//double zjz = 0.3;
			//std::cout << zjz << std::endl;
			double cc[3] = { xx - xjx, yy - yjy, zz - zjz };//����ĵ�����ֵ����zz-zjz�ˣ�һ��Ҫ�Ĺ��������в���
			std::vector<double>normal_point(cc, cc + 3);
			//std::cout <<normal_point[2] << std::endl;
			float dd[3] = { PCLnormal->points[pointIdxNKNSearch[j]].normal_x, PCLnormal->points[pointIdxNKNSearch[j]].normal_y, PCLnormal->points[pointIdxNKNSearch[j]].normal_z };
			std::vector<float>normal_near(dd, dd + 3);
			//normal_near.push_back = normal_changed->points[pointIdxNKNSearch[j]].normal_x;
			//normal_near.push_back = normal_changed->points[pointIdxNKNSearch[j]].normal_y;
			//normal_near.push_back = normal_changed->points[pointIdxNKNSearch[j]].normal_z;
			//����Ҫ����G3
			//�����Լ����ڵ㷨�ߵļн�
			double d = normal[0] * normal_near[0] + normal[1] * normal_near[1] + normal[2] * normal_near[2];
			/*double length = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
			double length_near = sqrt(normal_near[0] * normal_near[0] + normal_near[1] * normal_near[1] + normal_near[2] * normal_near[2]);
			d = d / (length*length_near);*/
			//���ߺͷ������֮��ļн�
			double nj = normal[0] * normal_point[0] + normal[1] * normal_point[1] + normal[2] * normal_point[2];
			/*double nn_pp = sqrt(normal_point[0] * normal_point[0] + normal_point[1] * normal_point[1] + normal_point[2] * normal_point[2]);
			double angle = nj / (length*nn_pp);*/
			//std::cout << angle << std::endl;
			//angle = acos(angle);
			double G3 = 0;
			if (d > 1)
				//ui->doubleSpinBoxThreeT->value()����ĵ���3ѡ����1
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


//�����Ǹ�����normal
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
		//����ռ��˳��˹����
		int size = pointIdxNKNSearch.size();
		int s = size;
		//float p[s] = { 0 };
		float p[30] = { 0 };//�����Ļ�ֻ���Ƕ��ĳ�30�ˣ�������pointIdxNKNSearch.size()��
		for (int j = 0; j < pointIdxNKNSearch.size(); j++)
		{
			double G1 = exp(-((pointNKNSquareDistance[j])) / 0.25);
			//�������G2����Ȩ����
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
			//�����������ƽ��
			double distance = ((normal[0] - normal_near[0])*(normal[0] - normal_near[0]) + (normal[1] - normal_near[1])*(normal[1] - normal_near[1]) +
				(normal[2] - normal_near[2])*(normal[2] - normal_near[2]));
			double G2 = 0;
			if (d > 10)
				//����ط�����һ��������������ȡ�ã�Ϊ0.3��
			{
				G2 = exp(-distance / 0.3);
			}
			//�ܲ��ܰ�curvature������Ȼ���������Сֵ
			//����hi����Ӱ������
			//int size = pointIdxNKNSearch.size();
			//float p[size];//������������⣬
			for (size_t ii = 0; ii < 30; ++ii)
			{
				p[ii] = cloud_normal->points[pointIdxNKNSearch[ii]].curvature;
			}
			//����
			for (int j = 0; j < 30; j++)   /* ���ݷ�Ҫ����n��*/
			{
				for (int i = 0; i < 30 - j; i++)  /* ֵ�Ƚϴ��Ԫ�س���ȥ��ֻ��ʣ�µ�Ԫ���е����ֵ�ٳ���ȥ�Ϳ����� */
				{
					if (p[i] > p[i + 1])  /* ��ֵ�Ƚϴ��Ԫ�س����� */
					{
						float k = p[i];
						p[i] = p[i + 1];
						p[i + 1] = k;
					}
				}
			}
			double h = (cloud_normal->points[i].curvature - p[0]) /
				(p[29] - p[0]);
			//����������f1��
			//double f1 = 0;
			f1 = f1 + G1 * G2*(1 + h);
			//����������ķ���
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
			double G = 0;  //��һ��ϵ������ĸ��
			//����ռ��˳��˹����
			for (int j = 0; j < k_indices[i].size(); j++)
			{
				//����G1
				double G1 = exp(-(k_sqr_distances[i][j]) / 2.25);  //2.25Ϊ����ϵ��
				//����G2
				//�������˻�nxn
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



//ȥ��ƽ��(���롢����������)
void reduceplant(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double d, int Iteration, int Probability)
{

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //�洢�����ģ�͵�ϵ��
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //�洢�ڵ㣬ʹ�õĵ�
	//�����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//��ѡ����
	seg.setOptimizeCoefficients(true);
	//��������
	seg.setModelType(pcl::SACMODEL_PLANE); //����ģ�����ͣ����ƽ��
	seg.setMethodType(pcl::SAC_RANSAC);      //���÷�����������������һ���ԡ�
	seg.setDistanceThreshold(d);   //0.02������ֵ
	seg.setInputCloud(cloud_in);
	seg.setMaxIterations(Iteration);//1000����������
	seg.setProbability(Probability);//0.95����
	seg.segment(*inliers, *coefficients);    //�ָ����
	if (inliers->indices.size() == 0)//�����ڵ��������ж��Ƿ�ɹ�
	{

		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}
	pcl::ExtractIndices<pcl::PointXYZ> extract;//����������ȡ����
	extract.setInputCloud(cloud_in);//��������ĵ���
	extract.setIndices(inliers);//���÷ָ����ڵ�λ��Ҫ�ĵ㼯
	extract.setNegative(true);//������ȡ������㣨False���ڵ㣩
	extract.filter(*cloud_out);//�������λ��
}


/////////////////////////////////////////������ʾ//////////////////////////////////////////////////////
//Ϊһ����ִ��һ�εĺ���������Ϊ���ñ������������塣���÷���Ϊ��viewer.runOnVisualizationThreadOnce (viewerOneOff)
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

//��Ϊ�ص�����������������ע���ÿ֡��ʾ��ִ��һ�Σ���������ʵ���ڿ��ӻ����������һ��ˢ����ʾ�ַ��������÷�����viewer.runOnVisualizationThread (viewerPsycho);ÿ��ˢ�¶�����
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

//��������������ʾ�Զ�����ɫ��
void showviewintwodifferentwindow(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right)//��������XYZRGBA
{
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
	viewer.setBackgroundColor(1, 1, 1, v1);
	viewer.addCoordinateSystem(1.0f, "globalleft", v1);//��ɫ�� X �᣻��ɫ��Y�᣻��ɫ��Z��
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
		//�ڴ˴����������������  
		user_data++;
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

//��4����������ʾ��ͬ�ǶȵĲ�ɫ����XYZRGBA
void showviewinfourdifferentwindow(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4)//��������XYZRGBA
{

	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);// 4�������ֱ���X�����Сֵ��Y�����Сֵ,X�����ֵ��Y�����ֵ
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
		//�ڴ˴����������������  
		user_data++;
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

//��һ����������ʾ��ɫ���ƺ����ʷ���(RGBA��XYZ��Normal)
void showcloudwithnormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	//��ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);     
	//���ԭ���ĵ���
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud");
	//���õ���ʾ��С
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	// //�����Ҫ��ʾ�ĵ��Ʒ���cloudΪԭʼ����ģ�ͣ�normalΪ������Ϣ��10��ʾ��Ҫ��ʾ����ĵ��Ƽ������ÿ10������ʾһ�η���1��ʾ���򳤶ȡ�
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloudxyz, cloud_normals, 20, 0.015, "normals");

	while (!viewer->wasStopped())
	{
		//�ڴ˴����������������  
		user_data++;
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void showrgbcloudwithnormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer_ptr->setBackgroundColor(0.0, 0.0, 0.0);//������ɫ,(1,1,1)��ɫ
	viewer_ptr->addCoordinateSystem(1.0f, "global");//����ϵ
	//viewer_ptr->setCameraPosition(0,0,200); //��������ԭ��
	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud); 
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
	//pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cloud_color_handler(cloud2);
	//�þ����˼�ǣ�������ĵ�����ɫ��Random��ʾ���������ɫ������������������Ⱦɫ�ʵķ�ʽ.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 255, 0);//��ɫ
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr,cloud_color_handler,"sample cloud");//PointXYZRGB ���͵�
	viewer_ptr->addPointCloud<pcl::PointXYZ>(cloud, cloud_color_handler, "original point cloud");//���Ʊ�ǩ

	viewer_ptr->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 5, 0.02, "normal");//���߱�ǩ
  //���У�����5��ʾ����������ÿ5������ʾһ������������ȫ����ʾ��������Ϊ1��  0.02��ʾ�������ĳ��ȣ����һ��������ʱ����֪�� ���Ӱ��ģ�);
	viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original point cloud");
	viewer_ptr->initCameraParameters();//��ʼ���������
	while (!viewer_ptr->wasStopped())
	{
		viewer_ptr->spinOnce();
		pcl_sleep(0.01);
	}
}

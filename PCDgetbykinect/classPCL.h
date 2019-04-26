#pragma once
#include "headerfiles.h"
#include "kinectget.h"


class MainPCL
{
public:

	//短定义
	typedef pcl::PointCloud<pcl::PointXYZRGBA> PCLRGBA;
	typedef pcl::PointCloud<pcl::PointXYZ> PCLXYZ;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::search::KdTree<pcl::PointXYZRGBA> KdtreeRGBA;
	typedef pcl::search::KdTree<pcl::PointXYZ> KdtreeXYZ;
	typedef pcl::visualization::PCLVisualizer ShowPCL;
	typedef pcl::PointCloud<pcl::FPFHSignature33> Features;


	
	//处理给定的点云

	void
		setInputCloud(PCLRGBA::Ptr rgba)
	{
		rgba_ = PCLRGBA::Ptr(new PCLRGBA);
		xyz_ = PCLXYZ::Ptr(new PCLXYZ);
		rgba_ = rgba;
		getAllPCL();
		std::cout << "Now We Have A Point Cloud" << endl;
	}

	//从给定的位置读取PCL文件
	void
		loadPLY(const std::string &pcd_file)
	{
		rgba_ = PCLRGBA::Ptr(new PCLRGBA);
		xyz_ = PCLXYZ::Ptr(new PCLXYZ);
		pcl::io::loadPLYFile(pcd_file, *rgba_);
		getAllPCL();
	}



	void
		loadInputCloud(const std::string &pcd_file)
	{
		rgba_ = PCLRGBA::Ptr(new PCLRGBA);
		xyz_ = PCLXYZ::Ptr(new PCLXYZ);
		pcl::io::loadPCDFile(pcd_file, *rgba_);
		getAllPCL();
	}

	//获得一个pointCloud
	PCLRGBA::Ptr
		getPointCloudRGBA() const
	{
		return (rgba_);
	}

	PCLXYZ::Ptr
		getPointCloudXYZ() const
	{
		return (xyz_);
	}




	//显示点云
	void
		showViewer()
	{
		//pcl::visualization::PCLHistogramVisualizer Plot;
		ShowPCL showPCL_;
		showPCL_.initCameraParameters();
		//showPCL_.addCoordinateSystem(1.0);

		int v1(0);
		showPCL_.addPointCloud<pcl::PointXYZRGBA>(rgba_, "cloud", v1);
		showPCL_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
		pcl::visualization::PCLPlotter plotter;
		// We need to set the size of the descriptor beforehand.
		plotter.addFeatureHistogram(*descriptor, 308);
		plotter.plot();
		while (!showPCL_.wasStopped())
		{
			//在此处可以添加其他处理  
			user_data++;
			showPCL_.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}


	


protected:



	//获得XYZ类型的点云
	void
		getAllPCL()
	{
		pcl::copyPointCloud(*rgba_, *xyz_);

	}

	//改变原点
	void
		changeO()
	{
		double Sx = 0;
		double Sy = 0;
		double Sz = 0;

		for (size_t i = 0; i < rgba_->size(); i++)
		{
			Sx = Sx + rgba_->points[i].x;
			Sy = Sy + rgba_->points[i].y;
			Sz = Sz + rgba_->points[i].z;
		}
		Sx = Sx / rgba_->size();
		Sy = Sy / rgba_->size();
		Sz = Sz / rgba_->size();

		pcl::PointXYZRGBA P;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZRGBA>);;


		for (size_t j = 0; j < rgba_->size(); j++)
		{
			P.x = rgba_->points[j].x - Sx;
			P.y = rgba_->points[j].y - Sy;
			P.z = rgba_->points[j].z - Sz;
			P.r = rgba_->points[j].r;
			P.g = rgba_->points[j].g;
			P.b = rgba_->points[j].b;
			P.a = rgba_->points[j].a;
			cloudout->push_back(P);
		}
		*rgba_ = *cloudout;
	}

	//改变轴方向
	void
		changexyz()
	{
		double S11 = 0;
		double S12 = 0;
		double S13 = 0;
		double S21 = 0;
		double S22 = 0;
		double S23 = 0;
		double S31 = 0;
		double S32 = 0;
		double S33 = 0;

		for (size_t i = 0; i < rgba_->size(); i++)
		{
			S11 = S11 + (rgba_->points[i].x)*(rgba_->points[i].x);
			S12 = S12 + (rgba_->points[i].x)*(rgba_->points[i].y);
			S13 = S13 + (rgba_->points[i].x)*(rgba_->points[i].z);
			S21 = S21 + (rgba_->points[i].y)*(rgba_->points[i].x);
			S22 = S22 + (rgba_->points[i].y)*(rgba_->points[i].y);
			S23 = S23 + (rgba_->points[i].y)*(rgba_->points[i].z);
			S31 = S31 + (rgba_->points[i].z)*(rgba_->points[i].x);
			S32 = S32 + (rgba_->points[i].z)*(rgba_->points[i].y);
			S33 = S33 + (rgba_->points[i].z)*(rgba_->points[i].z);
		}

		Eigen::Matrix3d B;
		B << S11, S12, S13, S21, S22, S23, S31, S32, S33;
		Eigen::EigenSolver<Eigen::Matrix3d> es(B);

		Eigen::Matrix4d T;
		Eigen::Matrix3d A;
		Eigen::Matrix3d D = es.pseudoEigenvalueMatrix();
		A = es.pseudoEigenvectors();


		T(0, 0) = A(0, 0);
		T(0, 1) = A(0, 1);
		T(0, 2) = A(0, 2);
		T(0, 3) = 0;
		T(1, 0) = A(1, 0);
		T(1, 1) = A(1, 1);
		T(1, 2) = A(1, 2);
		T(1, 3) = 0;
		T(2, 0) = A(2, 0);
		T(2, 1) = A(2, 1);
		T(2, 2) = A(2, 2);
		T(2, 3) = 0;
		T(3, 0) = 0;
		T(3, 1) = 0;
		T(3, 2) = 0;
		T(3, 3) = 1;

		pcl::transformPointCloud(*rgba_, *rgba_, T);

	}

	void
		getPPC2()
	{
		pcl::toPCLPointCloud2(*xyz_, *PPC2);
	}


private:
	// Point cloud data
	PCLRGBA::Ptr rgba_;//点云类型
	PCLXYZ::Ptr xyz_;//点云类型
	
	//投影回二维的图片
	Mat i_rgb;


	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor;
	pcl::PCLPointCloud2::Ptr PPC2;

};



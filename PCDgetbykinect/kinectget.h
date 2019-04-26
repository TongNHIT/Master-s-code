#pragma once
#include "headerfiles.h"
#include "classPCL.h"

typedef unsigned char nuchar;


class KinectPCL
{
public:

	//����ͼ���С
	KinectPCL() :
		i_rgb(1080, 1920, CV_8UC4),
		i_depth(424, 512, CV_16UC1),
		i_ir(424, 512, CV_16UC1),
		i_depthToRgb(424, 512, CV_8UC4)
	{}

	~KinectPCL() {}



	//��ȡͼ��
	void
		getPCL()
	{
		rgba_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		gainPCL();

	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
		getKinectPCL()
	{
		return rgba_;
	}
protected:

	//��ȡPCL
	HRESULT
		gainPCL()
	{

		int j = 1; //������ط���
		int k = 1;

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
		{
			return hr;
		}
		if (m_pKinectSensor)
		{
			hr = m_pKinectSensor->Open();//��Kinect  
		}
		//��ɫ��ʼ

		//��ȳ�ʼ


		//����ͼ�����
		//UINT nColorBufferSize = 1920 * 1080 * 4;//��UINT���޷���int��UINT32���޷���32λ��*4��Ӧ���ǲ�ɫͨ����
		//UINT16 *depthData = new UINT16[424 * 512];//��UINT16���޷���16λ���ݣ�256*256�����о���
		//UINT16 *depthData1 = new UINT16[424 * 512];

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer = rgbVis(rgba_);
		//boost::thread vthread(&viewerRunner, viewer);


		while (true)
		{
			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_ColorFrameSource(&m_pColorFrameSource);
				hr = m_pKinectSensor->get_DepthFrameSource(&m_pDepthFrameSource);
				hr = m_pKinectSensor->get_InfraredFrameSource(&m_pInfraredFrameSource);
			}
			if (SUCCEEDED(hr))
			{
				hr = m_pColorFrameSource->OpenReader(&m_pColorFrameReader);
				hr = m_pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
				hr = m_pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);
			}
			hr = m_pColorFrameReader->AcquireLatestFrame(&m_pColorFrame);
			if (SUCCEEDED(hr))
				//ע��reinterpret_cast��ǿ������ת�����ţ�����ΪRGB���ݣ���˼�ǽ���ɫ�����ݷ��õ�nColorBufferSize�У�ͨ��ǿ������ת�����õ���ɫͼ���У�
				hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
			//����������
			//SafeRelease(m_pDepthFrameSource);
			hr = m_pDepthFrameReader->AcquireLatestFrame(&m_pDepthFrame);
			if (SUCCEEDED(hr))
			{
				hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
				for (int i = 0; i < 512 * 424; i++)
				{
					// 0-255���ͼ��0-65536����Ϊ����ʾ���ԣ�ֻȡ������ݵĸ�8λ��BYTE���ֽڵ���˼��unsigned char��
					UINT16 intensity = static_cast<UINT16>(depthData[i]);
					reinterpret_cast<UINT16*>(i_depth.data)[i] = intensity;
				}
			}

			hr = m_pInfraredFrameReader->AcquireLatestFrame(&m_pInfraredFrame);

			if (SUCCEEDED(hr))
			{
				hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, irData);
				for (int i = 0; i < 512 * 424; i++)
				{
					// 0-255���ͼ��0-65536����Ϊ����ʾ���ԣ�ֻȡ������ݵĸ�8λ��BYTE���ֽڵ���˼��unsigned char��
					UINT16 intensity = static_cast<UINT16>(irData[i]);
					reinterpret_cast<UINT16*>(i_ir.data)[i] = intensity;
				}
			}


			ICoordinateMapper*  m_pCoordinateMapper;



			ColorSpacePoint* m_pColorCoordinates = new ColorSpacePoint[512 * 424];//����ɫ֡ӳ�䵽512*424�Ĵ�С
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
			HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);//���ͼ��Ͳ�ɫͼ��ϲ�
			//�ϲ��ĵ����m_pColorCoordinates���У�


			if (SUCCEEDED(hr))
			{
				for (int i = 0; i < 424 * 512; i++)//�������е�
				{
					i_depthToRgb.data[i * 4] = 255;
					i_depthToRgb.data[i * 4 + 1] = 255;
					i_depthToRgb.data[i * 4 + 2] = 255;
					i_depthToRgb.data[i * 4 + 3] = 255;
					ColorSpacePoint p = m_pColorCoordinates[i];//p�ǲ�ɫ�ռ��ڵ�һ��
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
					{
						int colorX = static_cast<int>(p.X + 0.5f);
						int colorY = static_cast<int>(p.Y + 0.5f);

						if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
						{
							i_depthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
							i_depthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
							i_depthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
							i_depthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
						}
					}
				}
			}
			// ���Ʊ���
			// ʹ������ָ�룬����һ���յ��ơ�����ָ��������Զ��ͷš�

			// �������ͼ
			for (int m = 0; m < 424; m++)
				for (int n = 0; n < 512; n++)
				{
					// ��ȡ���ͼ��(m,n)����ֵ
					UINT16 d = i_depth.ptr<UINT16>(m)[n];
					// d ����û��ֵ������ˣ������˵�
					if (d == 0 || d > 1200 || d < 510)
						continue;
					// d ����ֵ�������������һ����
					pcl::PointXYZRGBA P;
					// ���������Ŀռ����꣨�������λ�ã�
					P.z = double(d) / (camera_factor);
					P.x = (n - camera_cx) * P.z / camera_fx;
					P.y = (m - camera_cy) * P.z / camera_fy;
					//�������У��
					double r2 ;
					r2 = P.x * P.x + P.y * P.y;
					P.x = P.x * (1 + camera_k1 * r2 + camera_k2 * r2*r2) + (2 * camera_p1*P.x*P.y + camera_p2 * (r2 + 2 * P.x*P.x));
					P.y = P.y*(1 + camera_k1 * r2 + camera_k2 * r2*r2) + (2 * camera_p1*(r2 + 2 * P.y*P.y) + 2 * camera_p2*P.x*P.y);
					


					// ��rgbͼ���л�ȡ������ɫ
					// rgb����ͨ����BGR��ʽͼ�����԰������˳���ȡ��ɫ
					P.b = i_depthToRgb.ptr<nuchar>(m)[n * 4];
					P.g = i_depthToRgb.ptr<nuchar>(m)[n * 4 + 1];
					P.r = i_depthToRgb.ptr<nuchar>(m)[n * 4 + 2];
					P.a = i_depthToRgb.ptr<nuchar>(m)[n * 4 + 3];

					// ��p���뵽������
					rgba_->points.push_back(P);
				}

			rgba_->width = rgba_->size();
			rgba_->height = 1;
			rgba_->is_dense = true;

			KinectPoint.setInputCloud(rgba_);


			boost::mutex::scoped_lock updateLock(updateModelMutex);

			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA>rgb(KinectPoint.getPointCloudRGBA());
			viewer->updatePointCloud<pcl::PointXYZRGBA>(KinectPoint.getPointCloudRGBA(), rgb, "sample cloud");
			updateLock.unlock();
			//boost::this_thread::sleep(boost::posix_time::microseconds(100));



			if (waitKey(1) == VK_ESCAPE)
				break;

			cv::imshow("rgb2depth", i_depthToRgb);
			cv::imshow("rgb", i_rgb);
			cv::imshow("ir", i_ir);
			cv::imshow("depth", i_depth);


			//���س����¼�
			if (waitKey(1) == VK_RETURN)
			{


				SavePCL(k);
				k++;

			}

			//���ո���¼�
			if (waitKey(1) == VK_SPACE)
			{

				SavePicture(j);
				j++;
			}



			if (waitKey(1) == VK_ESCAPE)
				break;
			//��ʾ����


			rgba_->clear();

			SafeRelease(m_pColorFrame);
			SafeRelease(m_pDepthFrame);
			SafeRelease(m_pInfraredFrame);
		}

		cv::destroyAllWindows();
		m_pKinectSensor->Close();

	}


	void
		SavePicture(int j)
	{
		std::cout << "Picture " << j << "are being saved" << std::endl;

		std::stringstream rgbname;
		std::stringstream depthname;
		std::stringstream irname;
		std::stringstream rgbdname;
		//std::stringstream depthtxt;

		rgbname << "E:/PCLData/pointcloud/RGBD/rgb_" << j << ".jpg";
		rgbdname << "E:/PCLData/pointcloud/RGBD/rgbd_" << j << ".jpg";
		depthname << "E:/PCLData/pointcloud/RGBD/depth_" << j << ".tif";
		irname << "E:/PCLData/pointcloud/RGBD/ir_" << j << ".tif";
		//depthtxt << "depthtxt_" << j << ".txt";

		//����
		imwrite(rgbname.str(), i_rgb);
		imwrite(rgbdname.str(), i_depthToRgb);
		imwrite(depthname.str(), i_depth);
		imwrite(irname.str(), i_ir);

		std::cout << "Picture " << j << "has already saved" << std::endl;
		/*
		std::ofstream fout;
		fout.open(depthtxt.str(), std::ios::out | std::ios::app);
		if (!fout.is_open())
		{
			std::cout << "File Not Opened" << std::endl;
			return;
		}
		for (int g = 0; g < 424; g++)
		{
				for (int h = 0; h<512; h++)
				{
					UINT16 d = i_depth.ptr<UINT16>(g)[h];
					fout << d<<"\t";
				}
				fout << "\n";

		}
		fout.close();

		*/

	}


	void
		SavePCL(int j)
	{

		std::stringstream pclname;
		pclname << "E:/PCLData/pointcloud/RGBD/ceshi_" << j << ".pcd";
		pcl::io::savePCDFile(pclname.str(), *rgba_);
	}


	// ���ӻ���ɫ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
	{
		// --------------------------------------------
		// -----Open 3D viewer and add point cloud-----
		// --------------------------------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
		viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, 1);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		//viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
		return (viewer);
	}


	void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
	{
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));
		}
	}



private:
	//��ɫͼƬ
	//���ͼƬ
	//����ͼƬ
	Mat i_rgb;
	Mat i_depth;
	Mat i_ir;
	Mat i_depthToRgb;//�����ɫ���ͼ��

	IKinectSensor* m_pKinectSensor;
	HRESULT hr;

	//��ɫ��ʼ
	IColorFrameSource * m_pColorFrameSource;
	IColorFrameReader* m_pColorFrameReader;
	IColorFrame* m_pColorFrame;

	//��ȳ�ʼ
	IDepthFrameSource * m_pDepthFrameSource = nullptr;
	IDepthFrameReader* m_pDepthFrameReader = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;

	//������
	IInfraredFrameSource* m_pInfraredFrameSource;
	IInfraredFrameReader* m_pInfraredFrameReader;
	IInfraredFrame* m_pInfraredFrame;

	//����ͼ�����
	UINT nColorBufferSize = 1920 * 1080 * 4;//��UINT���޷���int��UINT32���޷���32λ��*4��Ӧ���ǲ�ɫͨ����
	UINT16 *depthData = new UINT16[424 * 512];//��UINT16���޷���16λ���ݣ�256*256�����о���
	UINT16 *irData = new UINT16[424 * 512];

	ICoordinateMapper*  m_pCoordinateMapper;
	ColorSpacePoint* m_pColorCoordinates = new ColorSpacePoint[512 * 424];//����ɫ֡ӳ�䵽512*424�Ĵ�С

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgba_;

	MainPCL KinectPoint;

	boost::mutex updateModelMutex;

	//����һ������ͼƬ�ĵĿռ�

};


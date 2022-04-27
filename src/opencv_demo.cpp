#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
// #include "cv.h"
#include "fstream"
#include "pcl/point_cloud.h"
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include "pcl/visualization/pcl_visualizer.h"

using namespace std::chrono_literals;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef struct cam_param
{
	float fx;	//焦距
	float fy;	//
	int cx;	//主点
	int cy;	//
	float scale;
	/* data */
}cam_param;

typedef struct cam2pic_data
{	
	int pic_xu; //像素坐标x
	int pic_yv;	//像素坐标y
	/* data */
	float cam_x;	//相机坐标下的x
	float cam_y;	//相机坐标下的y
	float cam_z;	//相机坐标下的z
}cam2pic_data;

struct color_data
{
	int R;
	int G;
	int B;
}color_data;

static cam_param camparam;
static cam2pic_data cam2picdata;
/************************************初始化函数******************************************/
bool param_init(float read_fx ,float read_fy , int read_cx ,int read_cy ,float read_scale)
{
	 cam2picdata.cam_z = 1;//参数归一化

	 camparam.fx = read_fx;
	 camparam.fy = read_fy;
	 camparam.cx = read_cx;
	 camparam.cy = read_cy;
	 camparam.scale = read_scale;

	return true;
}


/***********************************CV::MAT转换为点云*****************************************/
PointCloud::Ptr Image2PointCloud(Mat rgb, Mat depth,  cam_param camera)
{
	PointCloud::Ptr cloud(new PointCloud);
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			ushort d = depth.ptr<ushort>(m)[n];
			if (d == 0)
				continue;
			PointT p;
			p.z = double(d) / camera.scale;
			p.x = (n - camera.cx) * p.z / camera.fx;
			p.y = (m - camera.cy) * p.z / camera.fy;
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			cloud->points.push_back(p);
		}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;
	return cloud;
}

/***********************************点云可视化*****************************************/
// void visualize(PointCloud::Ptr cloud_in_1)
// {
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("visualize"));

// 	viewer->setBackgroundColor(0, 0, 0);
// 	viewer->addPointCloud(cloud_in_1, "cloud_in_1");

// 	while (!viewer->wasStopped())
// 	{
// 		// view->spin();
// 		viewer->spinOnce(100);
// 		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
// 	}
// }



int main(int argc, char **argv)
{
/********************************参数初始化************************************/
	param_init(1.5, 1.5, 308.4, 240.6,0.05);//float read_fx ,float read_fy , int read_cx ,int read_cy ,float read_scale
/********************************读取图片**************************************/
	cv::Mat data_pic = cv::imread("/media/lan/disk2/1_workspace/demo/src/1111.jpeg");
	cv::Mat data_src = cv::imread("/media/lan/disk2/1_workspace/demo/src/1111.jpeg");
	cv::Mat data_trans = cv::imread("/media/lan/disk2/1_workspace/demo/src/1111.jpeg");//这里存放转换后的
	cv::Mat data_trans2 = cv::imread("/media/lan/disk2/1_workspace/demo/src/1111.jpeg");//这里存放转换后的
	float u_1 ,v_1 ,d_1;
	if (!data_src.data)
	{
		std::cout << "读取图片失败！" << std::endl;
		return -1;
	}
	else 
	{
		std::cout << "读取图片成功！" << std::endl;
	}

/********************************图像坐标转换到相机坐标**************************************
 * X=（u-cx)/fx;Y=(v-cy)/fy,x和y为归一化坐标
 * 方案二：u'=（u-cx)/fx;Y ;v'= (v-cy)/fy; [X Y Z] = (scale + sqrt((1-scale^2)*(u'^2+v'^2)))/(u'^2+v'^2+1)[u' v' 1] - [0 0 scale]
*****************************************************************************************/	
	cout << "enter the axis to camera" << endl;
	for ( cam2picdata.pic_xu = 0;  cam2picdata.pic_xu < data_src.rows ; cam2picdata.pic_xu++)
	{
		// cam2picdata.cam_x = (int)(( cam2picdata.pic_xu + camparam.cx)/ (float)camparam.fx);
		// cout << "X = " << cam2picdata.pic_xu << " " << endl;
		u_1 = (cam2picdata.pic_xu - camparam.cx)/ (float)camparam.fx + 300;
		for ( cam2picdata.pic_yv = 0;  cam2picdata.pic_yv < data_src.cols; cam2picdata.pic_yv++)
		{
			// cout << "Y = " << cam2picdata.pic_yv << " " << endl;
			v_1 = (float)(( (float)cam2picdata.pic_yv - (float)camparam.cy)/  (float) camparam.fy) +300;

			cam2picdata.cam_x = u_1*u_1 * ((camparam.scale + sqrt((1 - camparam.scale * camparam.scale) * (u_1*u_1 + v_1*v_1)))/(u_1*u_1 + v_1*v_1 + 1));
			cam2picdata.cam_y = v_1*v_1 * ((camparam.scale + sqrt((1 - camparam.scale * camparam.scale) * (u_1*u_1 + v_1*v_1)))/(u_1*u_1 + v_1*v_1 + 1));
  			cam2picdata.cam_z = (camparam.scale + sqrt((1 - camparam.scale * camparam.scale )* (u_1*u_1 + v_1*v_1)))/(u_1*u_1 + v_1*v_1 + 1);
			// cout << "v' = " << v_1 << " " << endl;
			// cout << "cam_x = " << cam2picdata.cam_y << " " << endl;
			// cout << "cam_y = " << cam2picdata.cam_y << " " << endl;

			int mu = data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[0] + data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[1] + data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[2];
			//  mu /= 3;
			int stdev = sqrt(double((data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[0] - mu)*(data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[0] - mu) + (data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[1] - mu)*(data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[1] - mu) + (data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[2] - mu)*(data_src.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[2] - mu)) / 3);
			for (int k = 0; k<3; k++)
				data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[k] = stdev;
		}
	}
/********************************相机坐标转换到图像坐标*************************************
 * u=X*fx/Z + cx ; v=Y*fy/Z + cy
 * 方案二：u=fx*X/(Scale*d + z) + cx ; v = fy*Y/(scale*d+z) +cy; d=sqrt(X^2+Y^2+Z^2);
 *****************************************************************************************/
		for (cam2picdata.cam_x = 0; cam2picdata.cam_x < data_trans.rows; cam2picdata.cam_x++)
		{
			for (cam2picdata.cam_y = 0; cam2picdata.cam_y < data_trans.cols; cam2picdata.cam_y++)
			{
				d_1 = sqrt(cam2picdata.cam_x * cam2picdata.cam_x + cam2picdata.cam_y * cam2picdata.cam_y + cam2picdata.cam_z * cam2picdata.cam_z);

				cam2picdata.pic_xu = cam2picdata.cam_x * camparam.fx / (camparam.scale * d_1 + cam2picdata.cam_z) + camparam.cx;
				cam2picdata.pic_yv = cam2picdata.cam_y * camparam.fy / (camparam.scale * d_1 + cam2picdata.cam_z) + camparam.cy;

				int mu = data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[0] + data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[1] + data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[2];
				// mu /= 3;
				int stdev = sqrt(double((data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[0] - mu) * (data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[0] - mu) + (data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[1] - mu) * (data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[1] - mu) + (data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[2] - mu) * (data_trans.at<Vec3b>(cam2picdata.cam_x, cam2picdata.cam_y)[2] - mu)) / 3);
				for (int k = 0; k < 3; k++)
					data_trans2.at<Vec3b>(cam2picdata.pic_xu, cam2picdata.pic_yv)[k] = stdev;
			}
		}	

/********************************生成结果显示*******************************************/
    namedWindow("图片显示");
	namedWindow("投影显示");
	namedWindow("反投影显示");

    imshow("图片显示",data_pic);
    imshow("投影显示",data_trans);
    imshow("反投影显示",data_trans2);

    waitKey(0);
	return 0;
}


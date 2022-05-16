#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include <string>
#include <math.h>
#include <iostream>   // Include OpenCV API
#include <fstream>

    using namespace cv;
	using namespace rs2;
	using namespace std;
	char fileName[200];//保存图片0的路径
	char fileName1[200];//保存图片1的路径
int main(int argc, char * argv[])
{
	rs2::config cfg;
	ofstream imu0;
	system("mkdir ./cam0 ./cam1 ./imu0");
	
	imu0.open(" ./imu0/data.csv", ios::app); //out覆盖，app追加
							
	imu0 << "#timestamp [ns]" << "\t" << "x" << "\t" << "y" << "\t" << "z" << "\t" 
	<< "q.x" << "\t" << "q.y" << "\t" << "q.z" << "\t" << "q.w" << endl;
	imu0.close();

	cout<< "file dir make " << endl;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    cfg.enable_stream(RS2_STREAM_FISHEYE,1,RS2_FORMAT_Y8);
	cfg.enable_stream(RS2_STREAM_FISHEYE,2,RS2_FORMAT_Y8);
    // Start streaming with default recommended configuration
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	rs2::pipeline pipe;
    pipe.start(cfg);

	rs2::frameset data;

    while(1)
	{	

		data = pipe.wait_for_frames(); //等待数据流

		auto f = data.first_or_default(RS2_STREAM_POSE);
		auto pose = f.as<rs2::pose_frame>().get_pose_data();
		

		cout<<"px: "<<pose.translation.x<<"   py: "<<pose.translation.y<<"   pz: "<<pose.translation.z<<
		"vx: "<<pose.velocity.x<<"   vy: "<<pose.velocity.y<<"   vz: "<<pose.velocity.z<<endl;
		cout<<"ax: "<<pose.acceleration.x<<"   ay: "<<pose.acceleration.y<<"   az: "<<pose.acceleration.z<<
		"gx: "<<pose.angular_velocity.x<<"   gy: "<<pose.angular_velocity.y<<"   gz: "<<pose.angular_velocity.z<<endl;

		rs2::frame image_left = data.get_fisheye_frame(1);
		rs2::frame image_right = data.get_fisheye_frame(2);
	
		if (!image_left || !image_right)
			break;
		// cv::waitKey();

		std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMicro
				= std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
		time_t timeStamp2 = tpMicro.time_since_epoch().count();

		sprintf(fileName, "./cam0/%ld.png", timeStamp2); //保存图片的路径
		sprintf(fileName1, "./cam1/%ld.png", timeStamp2); //保存图片的路径


		cv::Mat cv_image_left(cv::Size(848, 800), CV_8U, (void*)image_left.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat cv_image_right(cv::Size(848, 800), CV_8U, (void*)image_right.get_data(), cv::Mat::AUTO_STEP);

		cv::imshow("left", cv_image_left);
		cv::imshow("right", cv_image_right);

		cv::imwrite(fileName, cv_image_left);
		cv::imwrite(fileName1, cv_image_right);
		imu0.open("./imu0/data.csv", ios::app);
		imu0 << timeStamp2 << "\t" << pose.translation.x << "\t" << pose.translation.y << "\t" << pose.translation.z <<
			"\t" << pose.rotation.x << "\t" << pose.rotation.y << "\t" << pose.rotation.z << pose.rotation.w << endl;
		imu0.close();
	}
	
    return 0;
}

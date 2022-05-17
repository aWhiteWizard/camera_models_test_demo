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
#include <unistd.h>
    using namespace cv;
	using namespace rs2;
	using namespace std;
	char fileName[200];//保存图片0的路径
	char fileName1[200];//保存图片1的路径

int main(int argc, char * argv[])
{
	rs2::config cfg;
	ofstream imu0;
	rs2_vector acc_data;
	rs2_vector gyro_data;
	double ts;
	system("mkdir ./record");
	system("mkdir ./record/cam0 ./record/cam1 ./record/imu0");

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    cfg.enable_stream(RS2_STREAM_FISHEYE,1,RS2_FORMAT_Y8);
	cfg.enable_stream(RS2_STREAM_FISHEYE,2,RS2_FORMAT_Y8);

	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F); //前一版本没有IMU 正确数据是这里没有使能！！！！
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

	rs2::pipeline pipe;
	pipe.start(cfg);
	rs2::frameset data;

	imu0.open("./record/imu0/data.csv", ios::app);
	imu0 << "#timestamp [ns]" << "\t" << "w_RS_S_x [rad s^-1]" << "\t" << "w_RS_S_y [rad s^-1]" << "\t" << "w_RS_S_z [rad s^-1]" << "\t" << "a_RS_S_x [m s^-2]" << "\t" << "a_RS_S_y [m s^-2]" << "\t" << "a_RS_S_z [m s^-2]" << endl;
	imu0.close();
	sleep(2);
	cout<< "file dir make " << endl;
    while(1)
	{	

		data = pipe.wait_for_frames(); //等待数据流
		rs2::frame image_left = data.get_fisheye_frame(1);
		rs2::frame image_right = data.get_fisheye_frame(2);

		auto f_gyro = data.first_or_default(RS2_STREAM_GYRO);
		auto gyro = f_gyro.as<rs2::motion_frame>();
		if (gyro && gyro.get_profile().stream_type() == RS2_STREAM_GYRO && gyro.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
		{
			gyro_data = gyro.get_motion_data();
		}
		auto f_acc = data.first_or_default(RS2_STREAM_ACCEL);
		auto acc = f_acc.as<rs2::motion_frame>();
		if (acc && acc.get_profile().stream_type() == RS2_STREAM_ACCEL && acc.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
		{
			ts = acc.get_timestamp();
			acc_data = acc.get_motion_data();
			cout << endl;
		}
			cout << "TimeStamp: " << ts << "\t" << " gx:" << "\t" << gyro_data.x << " gy:" << "\t" << gyro_data.y << " gz:" << "\t" << gyro_data.z 
			<< " ax:" << "\t" << acc_data.x << " ay:" << "\t" << acc_data.y << " az:" << "\t" << acc_data.z <<  endl;
		// auto acc = acc_stream.as<rs2::pose_frame>().get_pose_data();
		// cout << "ax:" << "\t" << pose.angular_velocity.x << "ay:" << "\t" << pose.angular_velocity.y << "az:" << "\t" << pose.angular_velocity.z << endl;
		// cout << "ax:" << "\t" << pose.acceleration.x << "ay:" << "\t" << pose.acceleration.y << "az:" << "\t" << pose.acceleration.z << endl;
		// // cout << "timestamp" << ts << endl;
		if (!image_left || !image_right)
			break;
		// cv::waitKey();

		std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMicro
				= std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
		time_t timeStamp2 = tpMicro.time_since_epoch().count();

		sprintf(fileName, "./record/cam0/%ld.png", timeStamp2); //保存图片的路径
		sprintf(fileName1, "./record/cam1/%ld.png", timeStamp2); //保存图片的路径


		cv::Mat cv_image_left(cv::Size(848, 800), CV_8U, (void*)image_left.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat cv_image_right(cv::Size(848, 800), CV_8U, (void*)image_right.get_data(), cv::Mat::AUTO_STEP);

		cv::imshow("left", cv_image_left);
		cv::imshow("right", cv_image_right);

		cv::imwrite(fileName, cv_image_left);
		cv::imwrite(fileName1, cv_image_right);
		imu0.open("./record/imu0/data.csv", ios::app);
		imu0 << ts  << "\t" << gyro_data.x << "\t" << gyro_data.y <<  "\t" << gyro_data.z << "\t" << acc_data.x << "\t" << acc_data.y << "\t" << acc_data.z << endl;
		imu0.close();
		
	}
	
    return 0;
}

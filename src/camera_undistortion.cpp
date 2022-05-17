#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include <string>
#include <math.h>
#include <iostream>
using namespace std;
using namespace cv;
string image_file = "/media/lan/disk2/1_workspace/camera_models_test_demo/data/pic_2";
string distort_file = "/media/lan/disk2/1_workspace/camera_models_test_demo/data/distort";
int main(int argc, char const *argv[])
{
    double k1 = -0.22898814237070902, k2 = 0.032542929993888366, p1 = 0.01186841923858413, p2 = 0.003220581226743435;
    double fx = 271.6370684829356, fy = 271.5300517555869, cx = 420.4737734129955, cy = 388.5689069064903;
    double w = 0.9345926693865441;
    vector<KeyPoint> keypoint1;
    vector<DMatch> match;
    std::vector<cv::String> image_files;
    std::vector<cv::String> distort_files;
    cv::glob(image_file, image_files);
    cv::glob(distort_file, distort_files);

    if (image_files.size() == 0) 
		std::cout << "No image files[jpg]" << std::endl;

    for (unsigned int frame = 0; frame < image_files.size(); frame++) //image_file.size()代表文件中总共的图片个数
	{
    cv::Mat image = cv::imread(image_files[frame],cv::IMREAD_GRAYSCALE);
    cv::Mat distort = cv::imread(distort_files[frame],cv::IMREAD_GRAYSCALE);
    // cv::Mat image = cv::imread(image_file, 0);
    int rows = image.rows, cols = image.cols;
    int distort_rows = distort.rows, distort_cols = distort.cols;
    cv::Mat image_undistort = cv::Mat(rows+800,cols+800, CV_8UC1);
    cv::Mat image_undistort_fov = cv::Mat(rows+800,cols+800, CV_8UC1);
    cv::Mat image_redistort_fov = cv::Mat(rows+800,cols+800, CV_8UC1);
/*
    opencv的KB投影
*/
    for(int v = 0; v < rows+800 ; v++)
    {
        for (int u = 0; u < cols+800; u++)
        {
            double x = ( u - cx - 400) / fx, y = ( v - cy - 400) / fy;
            double r = sqrt( x * x + y * y );
            double x_distorted = x* (1 + k1 * r * r + k2 * r * r * r * r + p1 * r * r * r * r * r * r + p2 * r * r *r * r *r * r *r * r);//+ 2 * p1 * x * y + p2 * (r*r + 2*x*x);// 
            double y_distorted = y* (1 + k1 * r * r + k2 * r * r * r * r + p1 * r * r * r * r * r * r + p2 * r * r *r * r *r * r *r * r); //+ 2 * p2 * x * y + p1 * (r*r + 2*y*y);/
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows){
                image_undistort.at<u_char>(v,u) =image.at<u_char>((int) v_distorted,(int) u_distorted);
            }
            else{
                image_undistort.at<u_char>(v,u) = 0;
            }            
        }
    }
/*
    FOV的w参数投影
*/
    for (int v = 0; v < rows+800; v++)
    {
        for (int u = 0; u < cols+800; u++)
        {
            double x = ( u - cx - 400 ) / fx, y = ( v - cy - 400 ) / fy;
            double r = sqrt( x * x + y * y );

            double rd = (1/w) * atan(2 * r * tan(w/2));
            // double cita = atan(rd/r);//(v - cy) / (u - cx));
            // cout << "cita  =  " << cita << endl;

            double x_distort_fov = (rd/r)*x;
            double y_distort_fov = (rd/r)*y;

            double u_distort_fov = x_distort_fov * fx + cx;
            double v_distort_fov = y_distort_fov * fy + cy;

            if (u_distort_fov >= 0 && v_distort_fov >= 0 && u_distort_fov < cols && v_distort_fov < rows){
                // cout << "rows = " << u << endl;
                // cout << "cols = " << v << endl;
                // cout << "v_distort_fov = " << v_distort_fov << endl;
                // cout << "u_distort_fov = " << u_distort_fov << endl;
                image_undistort_fov.at<u_char>(v,u) =image.at<u_char>((int) v_distort_fov ,(int) u_distort_fov);
            }
            else{
                image_undistort_fov.at<u_char>(v,u) = 0;
            }            
        }
    }
/*
    FOV的反投影
*/
       for (int v = 0; v < distort_rows; v++)
    {
        for (int u = 0; u < distort_cols; u++)
        {
            // double cxx = cx + 400;
            // double cyy = cy + 400;

            double x_distort_fov = (u - cx - 400) / fx;
            double y_distort_fov = (v - cy - 400) / fy;
 
            double rd = sqrt(x_distort_fov * x_distort_fov + y_distort_fov * y_distort_fov);
            double r = tan(rd * w) / (2 * tan(w/2));
            
            double x = x_distort_fov * (r/rd) * fx + cx + 400;
            double y = y_distort_fov * (r/rd) * fy + cy + 400;

            // double cita = atan(rd/r);//(v - cy) / (u - cx));
            // cout << "cita  =  " << cita << endl;

            if (y >= 0 && x >= 0 && x < distort_cols && y < distort_rows){
                // cout << "rows = " << u << endl;
                // cout << "cols = " << v << endl;
                // cout << "v_distort_fov = " << v_distort_fov << endl;
                // cout << "u_distort_fov = " << u_distort_fov << endl;
                image_redistort_fov.at<u_char>(v,u) =distort.at<u_char>((int) x ,(int) y);
            }
            else{
                image_redistort_fov.at<u_char>(v,u) = 0;
            }            
        }
    }

    // cv::imwrite(std::to_string(1111)+std::to_string(frame+28) + ".jpg",image_undistort);
    // cv::imwrite(std::to_string(2222)+std::to_string(frame+28) + ".jpg",image_redistort_fov);
    
    cv::imwrite(std::to_string(frame+28) + ".jpg",image_undistort_fov);
    }	
    
    cv::waitKey();
    return 0;
}

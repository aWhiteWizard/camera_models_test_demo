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
string image_file = "/home/lan/Dataset/slam/vq910_tum/mav0/cam0/data/91573842548556.png";

int main(int argc, char const *argv[])
{
    double k1 = 0.0008932028934440016, k2 = 0.01533862024154591, p1 = -0.009194221425211433, p2 = 0.002049588674966516;
    double fx = 271.16546955760583, fy = 271.6917056906144, cx = 318.75851896359893, cy = 251.77689999945986;
    double w = 0.9142787475944693;
    cv::Mat image = cv::imread(image_file, 0);

    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows,cols, CV_8UC1);
    cv::Mat image_undistort_fov = cv::Mat(rows,cols, CV_8UC1);
/*
    opencv的KB投影
*/
    for(int v = 0; v < rows ; v++)
    {
        for (int u = 0; u < cols; u++)
        {
            double x = ( u - cx ) / fx, y = ( v - cy ) / fy;
            double r = sqrt( x * x + y * y );
            double x_distorted = x* (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r*r + 2*x*x);
            double y_distorted = y* (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p2 * x * y + p1 * (r*r + 2*y*y);
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
    for (int v = 0; v < rows; v++)
    {
        for (int u = 0; u < cols; u++)
        {
            double x = ( u - cx ) / fx, y = ( v - cy ) / fy;
            double r = sqrt( x * x + y * y );

            double rd = (1/w) * atan(2 * r * tan(w/2));
            // double cita = atan(rd/r);//(v - cy) / (u - cx));
            // cout << "cita  =  " << cita << endl;

            double x_distort_fov = (rd/r)*x;
            double y_distort_fov = (rd/r)*y;

            double u_distort_fov = x_distort_fov * fx + cx;
            double v_distort_fov = y_distort_fov * fy + cy;

            if (u_distort_fov >= 0 && v_distort_fov >= 0 && u_distort_fov < cols && v_distort_fov < rows){
                image_undistort_fov.at<u_char>(v ,u) =image.at<u_char>((int) v_distort_fov ,(int) u_distort_fov );
            }
            else{
                image_undistort_fov.at<u_char>(v,u) = 0;
            }            
        }
    }
    cv::imshow("distorted", image);
    cv::imshow("undistorted", image_undistort);
    cv::imshow("fov", image_undistort_fov);
    cv::waitKey();
    return 0;
}

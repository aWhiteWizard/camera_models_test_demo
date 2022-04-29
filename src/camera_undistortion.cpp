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
string image_file = "/media/lan/disk2/1_workspace/camera_models_test_demo/src/91573842548556.png";

int main(int argc, char const *argv[])
{
    double k1 = -0.08932028934440016, k2 = 0.01533862024154591, p1 = -0.009194221425211433, p2 = 0.002049588674966516;
    double fx = 271.16546955760583, fy = 271.6917056906144, cx = 318.75851896359893, cy = 251.77689999945986;
    double w = 0.9197157202344117;
    cv::Mat image = cv::imread(image_file, 0);

    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows+300,cols+300, CV_8UC1);
    cv::Mat image_undistort_fov = cv::Mat(rows+800,cols+800, CV_8UC1);
/*
    opencv的KB投影
*/
    for(int v = 0; v < rows+300 ; v++)
    {
        for (int u = 0; u < cols+300; u++)
        {
            double x = ( u - cx - 150) / fx, y = ( v - cy - 150) / fy;
            double r = sqrt( x * x + y * y );
            double x_distorted = x* (1 + k1 * r * r + k2 * r * r * r * r + p1 * r * r * r * r * r * r + p2 * r * r *r * r *r * r *r * r) ;//+ 2 * p1 * x * y + p2 * (r*r + 2*x*x);
            double y_distorted = y* (1 + k1 * r * r + k2 * r * r * r * r) ;//+ 2 * p2 * x * y + p1 * (r*r + 2*y*y);
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
    cv::imshow("原图", image);
    cv::imwrite("./原图.jpg",image);
    cv::imshow("多项式反畸变投影", image_undistort);
    cv::imwrite("./多项式反畸变投影.jpg",image_undistort);
    cv::imshow("fov反畸变投影", image_undistort_fov);
    cv::imwrite("./fov反畸变投影.jpg",image_undistort_fov);
    
    cv::waitKey();
    return 0;
}

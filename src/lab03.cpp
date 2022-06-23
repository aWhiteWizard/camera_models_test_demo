#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"

using namespace cv;
using namespace std;

int i = 9346;
int max1 = 20000;
double k1 = -0.22898814237070902, k2 = 0.032542929993888366, p1 = 0.01186841923858413, p2 = 0.003220581226743435;
double fx = 271.16546955760583, fy = 271.6917056906144, cx = 318.75851896359893, cy = 251.77689999945986;
double w = 0.9345926693865441;


cv::Mat image = imread("/home/lan/Dataset/slam/vq910/cam0/91573842548556.png",cv::IMREAD_GRAYSCALE);
int rows = image.rows, cols = image.cols;
cv::Mat image_undistort_fov = cv::Mat(rows+600,cols+800, CV_8UC1);

void Fov_callback(int , void *);

int main( int argc, char** argv)
{

    cv::namedWindow("Fov Undistortion", WINDOW_AUTOSIZE);
    cv::createTrackbar("w反畸变率: ", "Fov Undistortion", &i, max1, Fov_callback);
    cv::waitKey(0);
    return 0;
}

void Fov_callback(int , void *)
{
    w = (double) i/10000;
    cout << " W为 : " << (double)i/10000 << endl;  
    for (int v = 0; v < rows+600; v++)
    {
        for (int u = 0; u < cols+800; u++)
        {
            double x = ( u - cx - 400 ) / fx, y = ( v - cy - 300 ) / fy;
            double r = sqrt( x * x + y * y );
            double rd = (1/w) * atan(2 * r * tan(w/2));

            double x_distort_fov = (rd/r)*x;
            double y_distort_fov = (rd/r)*y;

            double u_distort_fov = x_distort_fov * fx + cx;
            double v_distort_fov = y_distort_fov * fy + cy;

            if (u_distort_fov >= 0 && v_distort_fov >= 0 && u_distort_fov < cols && v_distort_fov < rows){
                image_undistort_fov.at<u_char>(v,u) =image.at<u_char>((int) v_distort_fov ,(int) u_distort_fov);
            }
            else{
                image_undistort_fov.at<u_char>(v,u) = 0;
            }            
        }
    }
    imshow("Fov Undistortion", image_undistort_fov);
}

void Fov_With_Multi_callback(int , void *)
{
    w = (double) i/10000;
    cout << " W为 : " << (double)i/10000 << endl;  
    for (int v = 0; v < rows+600; v++)
    {
        for (int u = 0; u < cols+800; u++)
        {
            double x = ( u - cx - 400 ) / fx, y = ( v - cy - 300 ) / fy;
            double r = sqrt( x * x + y * y );
            double rd = (1/w) * atan(2 * r * tan(w/2));

            double x_distort_fov = (rd/r)*x;
            double y_distort_fov = (rd/r)*y;

            double u_distort_fov = x_distort_fov * fx + cx;
            double v_distort_fov = y_distort_fov * fy + cy;

            if (u_distort_fov >= 0 && v_distort_fov >= 0 && u_distort_fov < cols && v_distort_fov < rows){
                image_undistort_fov.at<u_char>(v,u) =image.at<u_char>((int) v_distort_fov ,(int) u_distort_fov);
            }
            else{
                image_undistort_fov.at<u_char>(v,u) = 0;
            }            
        }
    }
    imshow("Fov Undistortion", image_undistort_fov);
}

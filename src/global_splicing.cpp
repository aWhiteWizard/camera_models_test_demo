#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <unistd.h>

using namespace cv;
using namespace std;

cv::String img1path = "/home/lan/Dataset/new_vr1/cam0";
cv::String img2path = "/home/lan/Dataset/new_vr1/cam1";
cv::String img3path = "/home/lan/Dataset/new_vr1/cam2";
cv::String img4path = "/home/lan/Dataset/new_vr1/cam3";

cv::Mat rotate270(cv::Mat src)
{
	cv::Mat dst;//目标图像
    cv::transpose(src, dst);
    cv::flip(dst, dst, 1);

	return dst;
}

int main(int argc, char const *argv[])
{
/***********************************************************************************************************************************************************/
    Mat M1 =  (cv::Mat_<double>(3,3) << 237.58117530242203, 0, 247.5035454475108, 0, 237.72892309468634, 317.6733511666785, 0, 0, 1);

    Mat D1 = (cv::Mat_<double>(1,4) << 0.21213973424319543, -0.17259823424161017, 0.0652253701355258, -0.010763591909664529);

    Mat M2 = (Mat_<double>(3,3)     << 237.18045035085484, 0, 248.49865665975085, 0, 237.0691024446269, 321.07298916263915, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量

    Mat D2 = (cv::Mat_<double>(1,4) << 0.21147831416516444, -0.1740764990890597, 0.06477470113318139, -0.010359708410074848);//k1,k2,p1,p2,k3
    
    Mat R =  (cv::Mat_<double>(3,3) << 0.9565376585238622, -0.007862822417264607, -0.2915028024724809,//3*3
                                        -0.0018681574639612267, 0.9994506711119726, -0.03308875944344066,
                                    0.29160284260165487, 0.03219521761769283, 0.9559975157651678);

    Mat T = (cv::Mat_<double>(3,1)  <<-0.09227808662414673, -0.0033723478007794756, -0.011952131992570284);

/***********************************************************************************************************************************************************/

    Mat M0_2 = (cv::Mat_<double>(3,3) << 232.86274725711323, 0, 248.54920673888924, 0, 233.09020302028782, 317.9606809629647, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量
    
    Mat D0_2 = (cv::Mat_<double>(1,4) << 0.20617143277010225, -0.15538446737239678, 0.05679231144366941, -0.00949619709355899);//k1,k2,p1,p2,k3

    Mat M2_0 = (cv::Mat_<double>(3,3) << 235.90521065154937, 0, 324.4166737350981, 0, 237.0782399056528, 253.42060933416013, 0, 0, 1);

    Mat D2_0 = (cv::Mat_<double>(1,4) << 0.20494612169496526, -0.16426560252682942, 0.054774546156623374, -0.0071079055596136595);

    Mat R0_2 = (cv::Mat_<double>(3,3) << 0.7980273594709841, -0.584606518734723, 0.1462448350836408,//3*3
                                        0.18626820914754152, 0.47009098328793353, 0.8627390229335614,
                                    -0.5731112350990347, -0.6612485808561379, 0.48403907540508256);

    Mat T0_2 = (cv::Mat_<double>(3,1)  << 0.007618904570149553, 0.04394088227723727, -0.01905647376841649);
/***********************************************************************************************************************************************************/

/***********************************************************************************************************************************************************/

    Mat M1_3 = (cv::Mat_<double>(3,3) << 234.6462919546538, 0, 249.175058068484224, 0, 234.05833126216152, 317.6255028053466, 0, 0, 1);
    
    Mat D1_3 = (cv::Mat_<double>(1,4) << 0.30164165161823964, -0.3520470345404912, 0.2007292963444695, -0.04610152044733895);


    Mat M3_1 = (cv::Mat_<double>(3,3) << 235.97841549146239, 0, 321.56637232071205, 0, 235.18921690631257, 230.9498875099282, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量


    Mat D3_1 = (cv::Mat_<double>(1,4) << 0.17539571323426828, -0.10434037690576677, 0.023121649929471782, -0.0016578754715017945);//k1,k2,p1,p2,k3
    
    Mat R1_3 = (cv::Mat_<double>(3,3) << 0.7424536191626323, 0.6512369292337921, -0.157013010271238,//3*3
                                        -0.13884229097304374, 0.3788880085586133, 0.9149681389031156,
                                        0.6553513879054896, -0.657521360088488, 0.3717260004327371);

    Mat T1_3 = (cv::Mat_<double>(3,1)  << -0.0004561511999128029, 0.04203643041568809, -0.025652261129010492);
/***********************************************************************************************************************************************************/       

    std::vector<cv::String> img1s;
    std::vector<cv::String> img2s;
    std::vector<cv::String> img3s;
    std::vector<cv::String> img4s;

    cv::glob(img1path, img1s);
    cv::glob(img2path, img2s);
    cv::glob(img3path, img3s);
    cv::glob(img4path, img4s);

    for (int frame = 0; frame < img3s.size(); frame++)
    {
        cv::Mat img1=imread(img1s[frame]),imgLr;
        cv::Mat img2=imread(img2s[frame]),imgRr;
        cv::Mat img3=imread(img3s[frame]),imgULr;
        cv::Mat img4=imread(img4s[frame]),imgURr;
        img1 = rotate270(img1);
        img2 = rotate270(img2);
        double cx1 = M1.at<double>(0, 2);
        double cy1 = M1.at<double>(1, 2); //得到img1 img2的光心

        double cx2 = M2.at<double>(0, 2);
        double cy2 = M2.at<double>(1, 2); //得到img1 img2的光心

        cv::Mat img1_c = (cv::Mat_<double>(3, 1) << cx1, cy1, 0.0);
        cv::Mat img2_c = (cv::Mat_<double>(3, 1) << cx2, cy2, 0.0);

        //R 相机位姿* 世界坐标 + t 相机位姿 = 归一化坐标
        //像素 = 内参K * 归一化坐标
        cout << "img1_c: " << img1_c << endl;
        cout << "img2_c: " << img2_c  << endl;

        cv::Mat trans = (cv::Mat_<double>(3, 1) ); 
        cout << "trans Mat Create"<< endl;

        trans =   M1.inv() * (img1_c  - T);//R.inv() *世界坐标 = 外参旋转的逆 *（相机内参的逆 * 像素 - 相机外参的位移）
        cout << "World Coordi: " << trans << endl;
        
        // trans /= trans.at<double>(0,2);                               //世界坐标归一化
        cout << "归一化：" << trans << endl;
        
        trans = M2_0 * (R0_2 * trans + T0_2); 
        trans /= trans.at<double>(0,2);   
        cout << "trans: " << trans << endl;   

        cv::circle(img3, cv::Point(trans.at<double>(0, 0), trans.at<double>(0, 1)), 20, Scalar(0, 128, 0),2);
        cv::circle(img1, cv::Point(cx1,cy1), 20, cv::Scalar(128,0,0),2);
        
        cv::Mat img_compare(640, 640+480 , CV_8UC3);
        cv::Mat img_Part1 = img_compare( Rect(0, 0, 640, 480));
        cv::Mat img_Part2 = img_compare(Rect(640,0,480,640));
        resize(img3, img_Part1, img_Part1.size(), 0,0,cv::INTER_AREA);
        resize(img1, img_Part2, img_Part2.size(), 0,0,cv::INTER_AREA);
        imshow("result", img_compare);
        cv::waitKey();
    }
    /* code */
    return 0;
}

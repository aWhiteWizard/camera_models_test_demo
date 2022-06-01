#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <unistd.h>

#include <iostream>  
#include <vector>  
#include <iterator>  
#include <algorithm>  

#include <pangolin/pangolin.h>
using namespace cv;
using namespace std;

cv::String img1path = "/home/lan/Dataset/new_vr1/cam0";
cv::String img2path = "/home/lan/Dataset/new_vr1/cam1";
cv::String img3path = "/home/lan/Dataset/new_vr1/cam2";
cv::String img4path = "/home/lan/Dataset/new_vr1/cam3";

// cv::Mat fov_undistortion(cv::Mat img, Eigen::Matrix3d Cam_init, double w, std::vector<cv::Point2d> point_in, cv::Mat point_out)
// {
    
//     double fx = Cam_init(0,0);
//     double fy = Cam_init(1,1);
//     double cx = Cam_init(0,2);
//     double cy = Cam_init(1,2);

//     int rows = img.rows;
//     int cols = img.cols;

//     cv::Mat return_Mat = cv::Mat(rows, cols, CV_8UC1);
//     cout << "point_in.size() :" << point_in.size() << endl;
//         // cout << fx << "\t" << fy << "\t" << cx << "\t" << cy << "\t" << rows << "\t" << cols << "\t" << endl;
//     for (int v = 0; v < rows+800; v++)
//     {
//         for (int u = 0; u < cols+800; u++)
//         {

//             double x = ( u - cx - 400 ) / fx, y = ( v - cy - 400 ) / fy;
//             double r = sqrt( x * x + y * y );

//             double rd = (1/w) * atan(2 * r * tan(w/2));
//             // double cita = atan(rd/r);//(v - cy) / (u - cx));
//             // cout << "cita  =  " << cita << endl;

//             double x_distort_fov = (rd/r)*x;
//             double y_distort_fov = (rd/r)*y;

//             double u_distort_fov = x_distort_fov * fx + cx;
//             double v_distort_fov = y_distort_fov * fy + cy;

//             if (u_distort_fov >= 0 && v_distort_fov >= 0 && u_distort_fov < cols && v_distort_fov < rows){
//                 return_Mat.at<u_char>(v,u) =img.at<u_char>((int) v_distort_fov ,(int) u_distort_fov);
//             }
//             else{
//                 return_Mat.at<u_char>(v,u) = 0;
//             }  


//             if(u == (int)point_in[0].x && v == (int)point_in[0].y)
//             {
//                 point_out.at<float>(0, 1) = (float)u_distort_fov;
//                 point_out.at<float>(0, 0) = (float)v_distort_fov;
//                 cout << "point_in[0].x : " << point_in[0].x << endl;
//                 cout << "point_in[0].y : " << point_in[0].y << endl;
//             }
//             if(u == (int)point_in[1].x && v == (int)point_in[1].y)         
//             {
//                 point_out.at<float>(1, 1) = (float)u_distort_fov;
//                 point_out.at<float>(1, 0) = (float)v_distort_fov;
//             }  
//             if(u == (int)point_in[2].x && v == (int)point_in[2].y)
//             {
//                 point_out.at<float>(2, 1) = (float)u_distort_fov;
//                 point_out.at<float>(2, 0) = (float)v_distort_fov;
//             }                 
//         }
//     }
    
//     return return_Mat;
// }
cv::Mat fov_undistortion(cv::Mat img, cv::Mat Cam_init, double w, std::vector<cv::Point2d> point_in, cv::Mat point_out)
{
    
    double fx = Cam_init.at<double>(0,0);
    double fy = Cam_init.at<double>(1,1);
    double cx = Cam_init.at<double>(0,2);
    double cy = Cam_init.at<double>(1,2);

    int rows = img.rows;
    int cols = img.cols;

    cv::Mat return_Mat = cv::Mat(rows+800, cols+800, CV_8UC1);
    cout << "point_in.size() :" << point_in.size() << endl;
    cout << fx << "\t" << fy << "\t" << cx << "\t" << cy << "\t" << rows << "\t" << cols << "\t" << endl;
    for (int v = 0; v < rows+800; v++)
    {
        for (int u = 0; u < cols+800; u++)
        {
            cv::Point2d point_cal;
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
                return_Mat.at<u_char>(v,u) =img.at<u_char>((int) v_distort_fov ,(int) u_distort_fov);
            }
            else{
                return_Mat.at<u_char>(v,u) = 0;
            } 

            if((int)u_distort_fov == (int)point_in[0].x && (int)v_distort_fov == (int)point_in[0].y)
            {
                point_out.at<float>(0, 1) = (float)u;
                point_out.at<float>(0, 0) = (float)v;
                cout << "point_in[0].x : " << point_in[0].x << endl;
                cout << "point_in[0].y : " << point_in[0].y << endl;
            }
            if((int)u_distort_fov == (int)point_in[1].x && (int)v_distort_fov == (int)point_in[1].y)         
            {
                point_out.at<float>(1, 1) = (float)u;
                point_out.at<float>(1, 0) = (float)v;
            }  
            if((int)u_distort_fov == (int)point_in[2].x && (int)v_distort_fov == (int)point_in[2].y)
            {
                point_out.at<float>(2, 1) = (float)u;
                point_out.at<float>(2, 0) = (float)v;
            }                    
        }
    }
    return return_Mat;
}

// 函数声明，在pangolin中画图，已写好，无需调整
void showPointCloud(
        const vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);

cv::Mat undistortion(cv::Mat img, cv::Mat K, cv::Mat D)
{
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    double k1 = D.at<double>(0, 0);
    double k2 = D.at<double>(0, 1);
    double p1 = D.at<double>(0, 2);
    double p2 = D.at<double>(0, 3);
    cout << "K: " << K << "\n" << " D: " << D << endl;
    cout << "start undistortion " << endl;
    cout << "rows: " << img.rows << " cols :" << img.cols << endl;
    cv::Mat undistort_img = cv::Mat(img.rows, img.cols, CV_8UC1);
    for (int v = 0; v < img.rows; v++)
    {
        for (int u = 0; u < img.cols; u++)
        {
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;

            double r = sqrt(x*x + y*y);

            double x_undistorted = x * (1+ k1*r*r + k2*r*r*r*r + 2*p1*x*y + p2*(r*r + 2*x*x) );
            double y_undistorted = y * (1+ k1*r*r + k2*r*r*r*r + 2*p2*x*y + p1*(r*r + 2*y*y) );

            double u_undistorted = fx * x_undistorted + cx;
            double v_undistorted = fy * y_undistorted + cy;
            // cout << "u_undistorted: " << u_undistorted << " v_undistorted :" << v_undistorted << endl;
            if(u_undistorted >= 0 && v_undistorted >= 0 && v_undistorted < img.rows && u_undistorted < img.cols ){
                undistort_img.at<uchar>(v, u) = img.at<uchar>((int)v_undistorted, (int)u_undistorted);
            }
            else{
                undistort_img.at<uchar>(v, u) = 0;
            }
        }
    }
    cout << "undistortion done " << endl;
    
    return undistort_img;
}
cv::Mat rotate270(cv::Mat src)
{
	cv::Mat dst;//目标图像
    cv::transpose(src, dst);
    cv::flip(dst, dst, 1);

	return dst;
}

int main(int argc, char const *argv[])
{
    double w_cam0 = 0.8667213059279031;
    double w_cam1 = 0.9161572549468926;
    double w_cam2 = 0.8728631211415925;
    double w_cam3 = 0.9161572549468926;
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

    //Mat M0_2 = (cv::Mat_<double>(3,3) << 232.86274725711323, 0, 248.54920673888924, 0, 233.09020302028782, 317.9606809629647, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量
    Mat M0_2 = (cv::Mat_<double>(3,3) << 224.47825343620758, 0, 250.17780462793385, 0, 226.15081157560073, 312.52728323091605, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量
    
    Mat D0_2 = (cv::Mat_<double>(1,4) << 0.20617143277010225, -0.15538446737239678, 0.05679231144366941, -0.00949619709355899);//k1,k2,p1,p2,k3

    // Mat M2_0 = (cv::Mat_<double>(3,3) << 235.90521065154937, 0, 324.4166737350981, 0, 237.0782399056528, 253.42060933416013, 0, 0, 1);
    Mat M2_0 = (cv::Mat_<double>(3,3) << 222.8407899581976, 0, 334.8309991655111, 0, 223.8994776495839, 263.43532563889386, 0, 0, 1);

    // Mat D2_0 = (cv::Mat_<double>(1,4) << 0.20494612169496526, -0.16426560252682942, 0.054774546156623374, -0.0071079055596136595);
    Mat D2_0 = (cv::Mat_<double>(1,4) << 0.22557849482387932, -0.20933665673343918, 0.09245956588839557, -0.017794448013968348);

    Mat R0_2 = (cv::Mat_<double>(3,3) << 0.7984182399013201, -0.584025845662031, 0.14643130058708992,//3*3
                                        0.18647783774645918, 0.47109619513667433, 0.8621452261406733,
                                    -0.5724983233357828, -0.66104628168268, 0.4850396718323353);

    Mat T0_2 = (cv::Mat_<double>(3,1)  << 0.007545682217839094, 0.04403714166115202, -0.019400920740212973);
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
        cv::Mat img1=imread(img1s[frame], cv::IMREAD_GRAYSCALE),imgLr;
        cv::Mat img2=imread(img2s[frame], cv::IMREAD_GRAYSCALE),imgRr;
        cv::Mat img3=imread(img3s[frame], cv::IMREAD_GRAYSCALE),imgULr;
        cv::Mat img4=imread(img4s[frame], cv::IMREAD_GRAYSCALE),imgURr;
        img1 = rotate270(img1);
        img2 = rotate270(img2);
        // cv::undistort(img3, imgULr, M2_0, D2_0);
        // imgULr = undistortion(img3, M2_0, D2_0);
        // imshow("imgULr", imgULr);
        imgULr = img3;
        imgURr = img4;

        // cv::undistort(img4, imgURr, M3_1, D3_1);
        // imgURr = undistortion(img4, M3_1, D3_1);
        // imshow("imgURr", imgURr);
/*******************************************************************************************************************************************/
// 双目匹配和反畸变
/*******************************************************************************************************************************************/
        //双目匹配一下
        Size imageSize = img1.size();;
        Mat R1, R2, P1, P2, Q;
        stereoRectify(M1, D1, M2, D2, imageSize,
                        R, T, R1, R2, P1, P2, Q);
        cout << "Q: " << Q << endl;

        cv::Mat remapX1 = cv::Mat(imageSize, CV_32FC1);
        cv::Mat remapY1 = cv::Mat(imageSize, CV_32FC1);
        cv::Mat remapX2 = cv::Mat(imageSize, CV_32FC1);
        cv::Mat remapY2 = cv::Mat(imageSize, CV_32FC1);

        cv::fisheye::initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2,
                                            remapX1, remapY1);
        cv::fisheye::initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2,
                                            remapX2, remapY2);

        if(!remapX1.empty() && !remapY1.empty())
        {
            remap(img1, imgLr, remapX1, remapY1, INTER_LINEAR);
        }
        if(!remapX2.empty() && !remapY2.empty())
        {
            remap(img2, imgRr, remapX2, remapY2, INTER_LINEAR);
        }

        cv::Mat undistort_image(640, 480*2, CV_8UC1);
        cv::Mat undis_Part1 = undistort_image(Rect(0, 0, 480, 640));
        cv::Mat undis_Part2 = undistort_image(Rect(480, 0, 480, 640));
        resize(imgLr, undis_Part1, undis_Part1.size(), 0, 0, INTER_AREA);
        resize(imgRr, undis_Part2, undis_Part2.size(), 0, 0, INTER_AREA);
        for (int i = 1; i < 16; i++)
        {
            line(undistort_image, Point(0,i*40), Point(undistort_image.cols, i*40), cv::Scalar(0, 255, 255),2);
        }
        imshow("undistort_image", undistort_image);

        double cx1 = M1.at<double>(0, 2);
        double cy1 = M1.at<double>(1, 2); //得到img1 img2的光心

        double cx2 = M2.at<double>(0, 2);
        double cy2 = M2.at<double>(1, 2); //得到img1 img2的光心

        double fx = M1.at<double>(0,0);
        double fy = M1.at<double>(1,1);
        cout << " 双目匹配和反畸变 done!" <<endl;
/*******************************************************************************************************************************************/
// 双目拼接
/*******************************************************************************************************************************************/
        // cv::Mat stitcher_merge = cv::Mat::zeros(imgLr.size(), CV_8UC1);
        // std::vector<cv::Mat> images;
        // images.push_back(imgLr);
        // images.push_back(imgRr);

        // Stitcher::Mode mode = Stitcher::PANORAMA;
        // Ptr<Stitcher> stitcher = Stitcher::create(mode);
        // auto blender = detail::Blender::createDefault(detail::Blender::MULTI_BAND);
        // stitcher->setBlender(blender);

        // Stitcher::Status status = stitcher->stitch(images, stitcher_merge);
        // cv::imshow("result ", stitcher_merge);
        // cout << " 双目拼接 done!" <<endl;
/*******************************************************************************************************************************************/
// 双目深度
/*******************************************************************************************************************************************/
        // 内参
        // double fx = 718.856, fy = 718.856, cx1 = 607.1928, cy1 = 185.2157;
        // 基线
        double b = 0.0938;
        int check_x;
        int check_y;
        // imgLr = cv::imread("/media/lan/disk2/1_workspace/camera_models_test_demo/build/left.png");
        // imgRr = cv::imread("/media/lan/disk2/1_workspace/camera_models_test_demo/build/right.png");
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, 96, 9, 8*9*9, 32*9*9, 1, 63, 10, 100, 32
        );
        cv::Mat disparity_sgbm, disparity;
        sgbm ->compute(imgLr, imgRr, disparity_sgbm);
        disparity_sgbm.convertTo(disparity, CV_64F, 1.0/16.0f);
        Eigen::Vector4d k;

        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> point_cloud;
        std::vector<cv::Point2d> point_ori_img1;
        cout << "fx: " << fx << " fy: " << fy << endl;
        // if(disparity.at<double>(v, u) <= 10.0 || disparity.at<double>(v, u) >= 96.0)
        //     continue;
        for (int v = 0; v < imgLr.rows; v+=5)
        {
            for (int u = 0; u < imgLr.cols; u+=5)
            {
                if(disparity.at<double>(v, u) <= 10.0 || disparity.at<double>(v, u) >= 96.0)
                    continue; 
                Eigen::Vector4d point(0, 0, 0, imgLr.at<uchar>(v, u) / 255.0);
                cv::Point2d uv_point;
                double x = (u - cx1)/ fx;
                double y = (v - cy1)/ fy;
                double depth = M1.at<double>(0,0) * b  / disparity.at<double>(v, u);
                
                point[0] = x * depth;
                point[1] = y * depth;
                point[2] = depth;

                uv_point = cv::Point2d(u, v);

                // if(u == 417 && v == 447)
                // {               
                //     check_x = u;
                //     check_y = v;    
                //     k[0] = point[0];
                //     k[1] = point[1];
                //     k[2] = point[2];
                // }
                point_cloud.push_back(point);
                point_ori_img1.push_back(uv_point); 
            }
        }         


        imshow("sgbm", disparity/96.0);
        showPointCloud(point_cloud);



/*******************************************************************************************************************************************/
// cam0 到 cam2的投影
/*******************************************************************************************************************************************/
        cv::Mat img1_c = (cv::Mat_<double>(3, 1) << cx1, cy1, 0.0);
        cv::Mat img2_c = (cv::Mat_<double>(3, 1) << cx2, cy2, 0.0);
        cv::Mat trans = (cv::Mat_<double>(1, 3) ); 

        std::vector<cv::Point2d> point_match;
        std::vector<Eigen::Vector2d> check_block;
        //R 相机位姿* 世界坐标 + t 相机位姿 = 归一化坐标
        //像素 = 内参K * 归一化坐标
        for(int frame = 0 ; frame < point_cloud.size(); frame++)
        {
            cv::Point2d point_dmatch;

            k << point_cloud[frame];

            trans = (cv::Mat_<double>(3, 1) << k[0], k[1], k[2]);
            // cout << "trans_input: " << trans << endl;   

            // cout << "归一化：" << trans << endl;
            trans = R0_2 * trans + T0_2;
            // cout << "trans_to_camera_coordi: " << trans << endl; 

            trans = M2_0 * trans; 
            // cout << "trans_no_norm: " << trans << endl; 

            trans /= trans.at<double>(0,2); 
            // cout << "trans: " << trans << endl; 
            if( trans.at<double>(0, 0) >= 0 && trans.at<double>(0, 1) >= 0 &&trans.at<double>(0, 0) <= 640 && trans.at<double>(0, 1) <= 480)
                {
                    point_dmatch = cv::Point2d(trans.at<double>(0, 0), trans.at<double>(0, 1));
                    // check_block[frame].data = point_ori_img1[frame];
                }
            else
                point_dmatch = cv::Point2d(0, 0);
            point_match.push_back(point_dmatch);
        }

        // cout << "img1_c: " << img1_c << endl;
        // cout << "img2_c: " << img2_c  << endl;


        // cout << "trans Mat Create"<< endl;

        // trans =   M1.inv() * (img1_c  - T);//R.inv() *世界坐标 = 外参旋转的逆 *（相机内参的逆 * 像素 - 相机外参的位移）
        // cout << "World Coordi: " << trans << endl;
        
        // // trans /= trans.at<double>(0,2);                               //世界坐标归一化
        // cout << "归一化：" << trans << endl;ans

        cv::circle(img1, cv::Point(0,0), 10, cv::Scalar(128,0,0),2);
        
        cv::Mat img_compare(640, 640+480 , CV_8UC1);
        cv::Mat img_Part1 = img_compare( Rect(0, 0, 640, 480));
        cv::Mat img_Part2 = img_compare(Rect(640,0,480,640));
        resize(imgULr, img_Part1, img_Part1.size(), 0,0,cv::INTER_AREA);
        resize(img1, img_Part2, img_Part2.size(), 0,0,cv::INTER_AREA);
        cvtColor(img_compare, img_compare, cv::COLOR_GRAY2BGR);
        RNG rng(12345);
        std::vector<cv::Point2d> get_match;
        std::vector<cv::Point2d> get_matchori;

        for (int frame = 0; frame < point_ori_img1.size(); frame+=40)
        {
            int b = rng.uniform(0, 255);
            int g = rng.uniform(0, 255);
            int r = rng.uniform(0, 255);
            cv::circle(img_compare, cv::Point(point_ori_img1[frame].x+640, point_ori_img1[frame].y), 2, cv::Scalar(255, 255, 0), 1);
            if(frame < point_match.size() && (point_match[frame].x != 0 || point_match[frame].y != 0) ) 
            {
                cv::circle(img_compare, point_match[frame], 2, cv::Scalar(0, 255, 255), 1);
                // cv::line(img_compare,         cv::Mat combine = cv::Mat(1200, 1200, CV_8UC1);point_match[frame], cv::Point(point_ori_img1[frame].x+640, point_ori_img1[frame].y), cv::Scalar(r, g, b), 1);
                get_match.push_back(point_match[frame]);  //提取出匹配好的点
                get_matchori.push_back(point_ori_img1[frame]);//得到匹配好的点的原图像点
            }
        }

        cv::circle(img_compare, cv::Point(0,0), 10, cv::Scalar(128,0,255),2);

        std::vector<cv::Point2d> point1;
        point1.push_back(get_match[0]);
        point1.push_back(cv::Point2d(0,0));
        point1.push_back(cv::Point2d(0,0));

        std::vector<cv::Point2d> point11;
        point11.push_back(get_matchori[0]);
        point11.push_back(cv::Point2d(0,0));
        point11.push_back(cv::Point2d(0,0));
        
        for (int frame = 0; frame < get_match.size(); frame++)
        {
            if(get_match[frame].x > point1[1].x && get_match[frame].x > 0)
               { 
                   point1[1] = get_match[frame];
                   point11[1] = get_matchori[frame];
               }
            if(get_match[frame].y > point1[2].y && get_match[frame].y > 0)
               { 
                   point1[2] = get_match[frame];
                   point11[2] = get_matchori[frame];
               }
        }
        cout << "get match point done !" << endl;
        cv::line(img_compare, point1[0], point1[1], cv::Scalar(0, 0, 255), 1);
        cv::line(img_compare, point1[0], point1[2], cv::Scalar(255, 0, 0), 1);       
        cv::line(img_compare, point1[1], point1[2], cv::Scalar(0, 255, 0), 1);     
        // cout << " k1 :" << (point1[0].y-point1[1].y)/(point1[0].x - point1[1].x) << " x1: " << point1[0].x - point1[1].x  << " y1: " << point1[0].y-point1[1].y << endl;
        // cout << " k2 :" << (point1[0].y-point1[2].y)/(point1[0].x - point1[2].x) << " x2: " << point1[0].x - point1[2].x  << " y2: " << point1[0].y-point1[2].y << endl;
        // cout << " k3 :" << (point1[1].y-point1[2].y)/(point1[1].x - point1[2].x) << " x3: " << point1[1].x - point1[2].x  << " y3: " << point1[1].y-point1[2].y << endl;

        cv::line(img_compare, cv::Point(point11[0].x + 640, point11[0].y), cv::Point(point11[1].x + 640, point11[1].y), cv::Scalar(0, 0, 255), 1);
        cv::line(img_compare, cv::Point(point11[0].x + 640, point11[0].y), cv::Point(point11[2].x + 640, point11[2].y), cv::Scalar(255, 128, 0), 1);       
        cv::line(img_compare, cv::Point(point11[1].x + 640, point11[1].y), cv::Point(point11[2].x + 640, point11[2].y), cv::Scalar(0, 255, 0), 1);   
        // cout << " k11 :" << (point11[0].x-point11[1].x)/(point11[0].y - point11[1].y) << " x11: " << point11[0].y - point11[1].y  << " y11: " << point11[0].x - point11[1].x << endl;
        // cout << " k22 :" << (point11[0].x-point11[2].x)/(point11[0].y - point11[2].y) << " x22: " << point11[0].y - point11[2].y  << " y22: " << point11[0].x - point11[2].x << endl;
        // cout << " k33 :" << (point11[1].x-point11[2].x)/(point11[1].y - point11[2].y) << " x22: " << point11[1].y - point11[2].y  << " y22: " << point11[1].x - point11[2].x << endl;
        cv::line(img_compare, cv::Point(point11[0].x+640, point11[0].y), point1[0], cv::Scalar(0, 255, 128), 1);   
        cv::line(img_compare, cv::Point(point11[1].x+640, point11[1].y), point1[1], cv::Scalar(45, 215, 128), 1);   
        cv::line(img_compare, cv::Point(point11[2].x+640, point11[2].y), point1[2], cv::Scalar(168, 35, 128), 1);   

        cout << "draw the line " << endl;
        cv::Mat dst_mat(2, 3, CV_32FC1);
        cv::Mat dst_img;
        cv::Mat srcpoint= cv::Mat_<float>(3, 2);
        std::vector<cv::Point2f> check_point;
        std::vector<cv::Point2f> result_point;
        std::vector<cv::Point2f> dst_point;
        std::vector<cv::Point2f> src_point;

        dst_point.push_back(cv::Point2f(point11[0].x + 300, point11[0].y + 300));
        dst_point.push_back(cv::Point2f(point11[1].x + 300, point11[1].y + 300));
        dst_point.push_back(cv::Point2f(point11[2].x + 300, point11[2].y + 300));

        check_point.push_back(point1[0]);
        check_point.push_back(point1[1]);
        check_point.push_back(point1[2]);

        cout << " new picture" << endl;
        cv::Mat imgULr_check(480, 640, CV_8UC1);
        resize(imgULr, imgULr_check, imgULr_check.size(),0,0,cv::INTER_AREA);


        cout << " new picture111" << endl;
        cv::circle(imgULr_check, point1[0], 2, cv::Scalar(0), 2);
        cv::circle(imgULr_check, point1[1], 2, cv::Scalar(0), 2);
        cv::circle(imgULr_check, point1[2], 2, cv::Scalar(0), 2);
        cv::line(imgULr_check, point1[0], point1[1], cv::Scalar(255), 2);
        cv::line(imgULr_check, point1[1], point1[2], cv::Scalar(255), 2);
        cv::line(imgULr_check, point1[0], point1[2], cv::Scalar(255), 2);
        imshow("ori pic", imgULr_check);
        cout << " ready to undistortion" << endl;
        cv::Mat src = fov_undistortion(imgULr_check, M2_0, w_cam2, point1, srcpoint);
        cout << "srcpoint :" << srcpoint << endl;  
        src_point.push_back(cv::Point2f(srcpoint.at<float>(0,1), srcpoint.at<float>(0,0)));
        src_point.push_back(cv::Point2f(srcpoint.at<float>(1,1), srcpoint.at<float>(1,0)));
        src_point.push_back(cv::Point2f(srcpoint.at<float>(2,1), srcpoint.at<float>(2,0)));

        cvtColor(src, src, cv::COLOR_GRAY2RGB);
        cout << "undistortion done !" << endl;
        cv::Mat dst;//目标图像、
        cout << " draw line !" << endl;
        cv::circle(src, src_point[0], 2, cv::Scalar(0, 244, 255), 2);
        cv::circle(src, src_point[1], 2, cv::Scalar(0, 244, 255), 2);
        cv::circle(src, src_point[2], 2, cv::Scalar(0, 244, 255), 2);
        cv::line(src, src_point[0], src_point[1], cv::Scalar(255, 0, 200), 2);
        cv::line(src, src_point[1], src_point[2], cv::Scalar(255, 0, 200), 2);
        cv::line(src, src_point[0], src_point[2], cv::Scalar(255, 0, 200), 2);
        cout << " draw line done!" << endl;
        imshow("src", src);
        cout << " getAffineTransform" << endl;
        dst_mat = getAffineTransform(src_point, dst_point);//这个函数只能用float，不能用double

        cv::warpAffine(src, dst, dst_mat, src.size());
        namedWindow("warpaffine", cv::WINDOW_AUTOSIZE);
        imshow("warpaffine", dst );

        for (int n = 0; n<src_point.size(); n++)
        {
            Point2f p = Point2f(0, 0);
            p.x = dst_mat.ptr<double>(0)[0] *src_point[n].x + dst_mat.ptr<double>(0)[1] * src_point[n].y + dst_mat.ptr<double>(0)[2];
            p.y = dst_mat.ptr<double>(1)[0] * src_point[n].x + dst_mat.ptr<double>(1)[1] * src_point[n].y + dst_mat.ptr<double>(1)[2];
            cout << "p: " << p << endl;
            cout << "dst_point" << n << ":" << dst_point[n] << endl;
            result_point.push_back(p);
        }

        dst_mat = getAffineTransform(check_point, dst_point);//这个函数只能用float，不能用double
        cv::warpAffine(imgULr_check, dst, dst_mat, src.size());

        // warpAffine(dst_img, dst_img, translation_matrix, dst_sz);
                namedWindow("warpaffine2", cv::WINDOW_AUTOSIZE);
        imshow("warpaffine2", dst );
        imwrite("./global_splicing_data/warpaffine.png", dst);
        // cv::Mat combine = cv::Mat(1200, 1200, CV_8UC1);
                
        imshow("result", img_compare);
        cv::waitKey();
    }
    return 0;
}

//使用pangolin绘制点云图函数
void showPointCloud(const vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud) {

    //如果点云是空的，不绘制
    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    //创建并初始化显示窗口，定义窗口名称，宽度，高度
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        //如果想绘制其他图像，将这里的pointcloud改成对应的容器就可以
        //或者将pointcloud对应到其他图像
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

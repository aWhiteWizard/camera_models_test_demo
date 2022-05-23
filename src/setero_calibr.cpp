// #include "stdafx.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>

using namespace cv;

using namespace std;

void Rotate(const Mat &srcImage, Mat &destImage, double angle, int cx, int cy)
{
    Point2f center(cx, cy);//中⼼
    Mat M = getRotationMatrix2D(center, angle, 1);//计算旋转的仿射变换矩阵
    warpAffine(srcImage, destImage, M, Size(srcImage.cols, srcImage.rows));//仿射变换
    circle(destImage, center, 2, Scalar(255, 0, 0));
}

cv::Mat fov_undistortion(cv::Mat img, Eigen::Matrix3d Cam_init, double w)
{
    
    double fx = Cam_init(0,0);
    double fy = Cam_init(1,1);
    double cx = Cam_init(0,2);
    double cy = Cam_init(1,2);

    int rows = img.rows;
    int cols = img.cols;

    cv::Mat return_Mat = cv::Mat(rows, cols, CV_8UC1);
        // cout << fx << "\t" << fy << "\t" << cx << "\t" << cy << "\t" << rows << "\t" << cols << "\t" << endl;
    for (int v = 0; v < rows; v++)
    {
        for (int u = 0; u < cols; u++)
        {
            double x = ( u - cx ) / fx, y = ( v - cy ) / fy;
            double r = sqrt( x * x + y * y );

            double rd = (1/w) * atan(2 * r * tan(w/2));
            double x_distort_fov = (rd/r)*x;
            double y_distort_fov = (rd/r)*y;

            double u_distort_fov = x_distort_fov * fx + cx;
            double v_distort_fov = y_distort_fov * fy + cy;

            if (u_distort_fov >= 0 && v_distort_fov >= 0 && u_distort_fov < cols && v_distort_fov < rows){
                return_Mat.at<unsigned char>(v,u) =img.at<unsigned char>((int) v_distort_fov ,(int) u_distort_fov);
            }
            else{
                return_Mat.at<unsigned char>(v,u) = 0;
            }            
        }
    }
    return return_Mat;
}

int main()

{

    //    Mat img1=imread("/home/lan/Dataset/TUM_VI/321321/cam1/1652663646416.png"),imgLr;
    //    Mat img2=imread("/home/lan/Dataset/TUM_VI/321321/cam0/1652663646416.png"),imgRr;
       Mat img1=imread("/home/lan/Dataset/slam/vq910/cam1/91573842548556.png"),imgLr;
       Mat img2=imread("/home/lan/Dataset/slam/vq910/cam0/91573842548556.png"),imgRr;
        // cv::rotate(imgLr, imgLr, ROTATE_90_CLOCKWISE);
        // cv::rotate(imgRr, imgRr, ROTATE_90_CLOCKWISE);
        double w_cam0 = 0.9131244077353585;
        double w_cam1 = 0.9161572549468926;
       Mat M1 = (Mat_<double>(3,3)     << 271.0805948891938, 0, 318.691785507845, 0, 271.62193446429274, 252.02698501119013, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量
    
        Eigen::Matrix3d  MM1;
        MM1 << 271.0805948891938, 0, 318.691785507845, 0, 271.62193446429274, 252.02698501119013, 0, 0, 1;
       Mat D1 = (cv::Mat_<double>(1,5) << 0.008014524486454106, 0.0022721726538408884, 5.4399433288378564e-05, -0.00017398833375916037, 0);//k1,k2,p1,p2,k3

       Mat M2 = (cv::Mat_<double>(3,3) << 276.42662984407025, 0, 315.28783246459574, 0, 274.66783749873605, 241.24965814208446, 0, 0, 1);
        Eigen::Matrix3d  MM2;
        MM2 << 271.0805948891938, 0, 318.691785507845, 0, 271.62193446429274, 252.02698501119013, 0, 0, 1;

       Mat D2 = (cv::Mat_<double>(1,5) << -0.0009010715283627364, 0.004741374193654468, 0.0059017739008470505, -0.0035043741168847837, 0);
       Mat R =  (cv::Mat_<double>(3,3) << 0.9996484678637104, 0.004410083875566417, 0.026143677207742296,//3*3
                                        -0.013433859311434142, 0.9343698857627986, 0.3560511873363313,
                                       -0.02285764908586946, -0.35627723438331166, 0.9341006691671095);
       Mat T = (cv::Mat_<double>(3,1)  <<0.00447135768940104, 0.10921466117951428, -0.02414808998669323);

    //    Mat M1 = (Mat_<double>(3,3)<< 293.8498334974922, 0, 420.4823150576605, 0, 293.736400415463, 388.45441674195484, 0, 0, 1);
    

    //    Mat D1 = (cv::Mat_<double>(5,1) << -0.025087916717331488, 0.058176857155773205, 0.06887330559535476, -0.28199310832781693, 0);
    //    Mat M2 = (cv::Mat_<double>(3,3)<< 301.2068004353627, 0, 426.5000009225249, 0, 301.0534525744726, 392.71581590321114, 0, 0, 1);


    //    Mat D2 = (cv::Mat_<double>(5,1) << -0.03758919067739605, 0.016517764183157223, 0.16757672644271707, -0.22085341357376675, 0);
    //    Mat R =  (cv::Mat_<double>(3,3) << 0.9999955069280155, -0.00036417303323679843, 0.0029754834535762873,
    //             0.00034231258457879395, 0.9999729733993692, 0.007344065149377274,
    //             -0.0029780775468550426, -0.007343013606532536, 0.9999686051098298);
                           
    //    Mat T = (cv::Mat_<double>(3,1) << -0.06626814024553064, -0.002456914644580518, 0.01878789634834968);
       
       cout<<"M1: "<<M1<<"\n" <<"M2: "<<M2<<"\n" <<"D1: "<<D1<<"\n" <<"D2: "<<D2<<"\n"<<"R: "<<R<<"\n"<<"T: "<<T<<"\n"<<endl;

 

       //校正之前，把om变为3*3;

       //Mat m_Rotation;

       //Rodrigues(R,m_Rotation);//注：matlab标定的参数中R为3*1，转换成3*3形式；


       Size imageSize=img1.size();//图片尺寸
       Mat R1,R2,P1,P2,Q;
       double alpha=-1;
       if (alpha < 0 || alpha > 1)//裁剪系数，阈值
        alpha = -1;
       Rect roi1,roi2;//感兴趣区域
        stereoRectify(   M1, D1, M2,  D2,  imageSize,//图片尺寸
                        R,  T,  R1,//输出左相机的旋转矩阵
                        R2,//输出右相机的旋转矩阵
                        P1,//左投影矩阵
                        P2,//右投影矩阵
                        Q, //重投影矩阵
                        0//CALIB_ZERO_DISPARITY ,//主点坐标相同
                        //-1,//裁剪系数
                        //imageSize,
                        //&roi1,
                        //&roi2
                        );//③：执行双目校正
        // img1 = fov_undistortion(img1, MM1, w_cam0);
        // img2 = fov_undistortion(img2, MM2, w_cam1);
    //    cv::fisheye::undistortImage(img2,img2,M2,D2);
    //    imshow("undistort1", img1);
    //    imshow("undistort2", img2);


       FileStorage fs_6("Q.xml",FileStorage::WRITE);//保存Q阵
       fs_6<<"Q"<<Q;
       fs_6.release();
       cout<<"Q: "<<Q<<"\n"<<endl;
       cout<<"R1: "<<R1<<"\n" <<"R2: "<<R2<<"\n" <<"P1: "
       <<P1<<"\n" <<"P2: "<<P2<<"\n" <<endl;

       Mat remapmX1= Mat(imageSize,CV_32FC1);
       Mat remapmY1= Mat(imageSize,CV_32FC1);
       Mat remapmX2= Mat(imageSize,CV_32FC1);
       Mat remapmY2= Mat(imageSize,CV_32FC1);
       initUndistortRectifyMap(M1, //
                                D1,//
                              R1,//
                             P1,//
                              imageSize,
                             CV_16SC2,
                            remapmX1,
                             remapmY1
                               );  //④计算校正查找映射表，分别求映射矩阵

       initUndistortRectifyMap(M2, //
                             D2, //
                                 R2, //
                                P2, //
                                imageSize,
                                CV_16SC2,
                                remapmX2,
                                remapmY2
                               );


        // imgLr = fov_undistortion(imgLr, MM1, w_cam0);
        // imgRr = fov_undistortion(imgRr, MM2, w_cam1);

       if ( !remapmX1.empty() && !remapmY1.empty() )//⑤进行矫正，映射
       {
              remap( img1, imgLr, remapmX1, remapmY1, INTER_LINEAR );
       }
       if ( !remapmX2.empty() && !remapmY2.empty() )
       {
              remap( img2, imgRr, remapmX2, remapmY2, INTER_LINEAR );
       }
       
        // int cx = 314.1240768432617;
        // int cy = 180.386173248291;


        cv::Mat result1 = cv::Mat::zeros(imgLr.size(), CV_8U) ;
        std::vector<cv::Mat> images;
        images.push_back(imgLr);
        images.push_back(imgRr);
        cout << " images.size() : " << images.size() << endl;
        // cout << "P2 :" <<  P2 << endl;
        // cout << " cx : " << cx << " cy : " << cy << endl;
        // cv::Mat imgROI = combine(Rect(330, 385, img1.cols, img1.rows));
        // addWeighted(img1, 1, imgROI, 1, 0, imgROI);

        // imgROI = combine(Rect(300, 300, img2.cols, img2.rows));
        // addWeighted(img2, 1, imgROI, 1, 0, imgROI);
        Stitcher::Mode mode = Stitcher::PANORAMA;
        Ptr<Stitcher> stitcher = Stitcher::create(mode);
        auto blender = detail::Blender::createDefault(detail::Blender::MULTI_BAND);
        stitcher->setBlender(blender);

        // auto fisheye_warper = makePtr<cv::FisheyeWarper>();
        // stitcher->setWarper(fisheye_warper);
        Stitcher::Status status = stitcher->stitch(images, result1);

        // auto plane_warper = makePtr<cv::PlaneWarper>();
        // stitcher->setWarper(plane_warper);
        // status = stitcher->stitch(images, result1);
        cout << "stitcher finish "<<endl;
        // result1=np.array(result1,dtype=np.uint8);
       imshow("imgLr",imgLr);
       imshow("imgRr",imgRr);
       imshow("result", result1);
       cout << "show result  "<<endl;
       imwrite("imgLr.png",imgLr);
       imwrite("imgRr.png",imgRr);//保存图片
       imwrite("MERGE_RESULT.png",result1);//保存图片
 

       Mat img(imageSize.height*0.5, imageSize.width, CV_8UC3);//⑥创建IMG，高度一样，宽度双倍
       Mat imgPart1 = img( Rect(0, 0, imageSize.width*0.5, imageSize.height*0.5) );//浅拷贝
       Mat imgPart2 = img( Rect(imageSize.width*0.5, 0, imageSize.width*0.5, imageSize.height*0.5) );//浅拷贝
       resize(imgLr, imgPart1, imgPart1.size(), 0, 0, cv::INTER_AREA);
        resize(imgRr, imgPart2, imgPart2.size(), 0, 0, cv::INTER_AREA);//改变图像尺寸，调节0,0；

    for( int i = 0; i < img.rows; i += 16 ) //画横线

        line(img, Point(0, i), Point(img.cols, i), Scalar(0, 255, 0), 1, 8);
       imshow("rectified", img);
       imwrite("ret.png",img);
       waitKey();
       return 0;

}
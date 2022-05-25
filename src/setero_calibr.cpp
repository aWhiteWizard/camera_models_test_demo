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

// void Rotate(const Mat &srcImage, Mat &destImage, double angle, int cx, int cy)
// {
//     Point2f center(cx, cy);//中⼼
//     Mat M = getRotationMatrix2D(center, angle, 1);//计算旋转的仿射变换矩阵
//     warpAffine(srcImage, destImage, M, Size(srcImage.cols, srcImage.rows));//仿射变换
//     circle(destImage, center, 2, Scalar(255, 0, 0));
//     //    transpose();
//     //    flip();
// }

cv::Mat rotate270(cv::Mat src)
{
	cv::Mat dst;//目标图像
	// cv::Size src_sz = src.size();
	// cv::Size src_ext_sz = src.size();
    cv::transpose(src, dst);
    cv::flip(dst, dst, 1);
	// Mat src_ext = Mat::zeros(src_ext_sz, CV_8UC3);;//扩充后的原图像

	// cv::Mat tempRoi(src_ext,cv::Rect(0, (src_sz.width - src_sz.height)/2, src.cols, src.rows));
	// src.copyTo(tempRoi);

	// cv::Size dst_sz(src_ext_sz.width, src_ext_sz.height);
 
	// cv::Point center = cv::Point(src_ext.cols / 2, src_ext.rows / 2);//旋转中心
	// cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);//获得仿射变换矩阵
 
	// cv::warpAffine(src_ext, dst, rot_mat, dst_sz);
	return dst;
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
cv::String img1path = "/home/lan/Dataset/new_vr1/cam0";
cv::String img2path = "/home/lan/Dataset/new_vr1/cam1";
cv::String img3path = "/home/lan/Dataset/new_vr1/cam2";
cv::String img4path = "/home/lan/Dataset/new_vr1/cam3";

int main()

{
    std::vector<cv::String> img1s;
    std::vector<cv::String> img2s;
    std::vector<cv::String> img3s;
    std::vector<cv::String> img4s;

    //    Mat img1=imread("/home/lan/Dataset/TUM_VI/321321/cam1/1652663646416.png"),imgLr;
    //    Mat img2=imread("/home/lan/Dataset/TUM_VI/321321/cam0/1652663646416.png"),imgRr;
    cv::glob(img1path, img1s);
    cv::glob(img2path, img2s);
    cv::glob(img3path, img3s);
    cv::glob(img4path, img4s);

   //    cv::Mat img3=imread("/home/lan/Dataset/new_vr/cam2/37561454004.png"),imgURr;
    //    cv::Mat img4=imread("/home/lan/Dataset/new_vr/cam3/37561454004.png"),imgULr;
        // cv::rotate(imgLr, imgLr, ROTATE_90_CLOCKWISE);
        // cv::rotate(imgRr, imgRr, ROTATE_90_CLOCKWISE);
        // double w_cam0 = 0.9131244077353585;
        // double w_cam1 = 0.9161572549468926;

       Mat M1 =  (cv::Mat_<double>(3,3) << 237.58117530242203, 0, 247.5035454475108, 0, 237.72892309468634, 317.6733511666785, 0, 0, 1);
    
       Mat D1 = (cv::Mat_<double>(1,4) << 0.21213973424319543, -0.17259823424161017, 0.0652253701355258, -0.010763591909664529);

       Mat M2 = (Mat_<double>(3,3)     << 237.18045035085484, 0, 248.49865665975085, 0, 237.0691024446269, 321.07298916263915, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量


       Mat D2 = (cv::Mat_<double>(1,4) << 0.21147831416516444, -0.1740764990890597, 0.06477470113318139, -0.010359708410074848);//k1,k2,p1,p2,k3
       Mat R =  (cv::Mat_<double>(3,3) << 0.9565376585238622, -0.007862822417264607, -0.2915028024724809,//3*3
                                        -0.0018681574639612267, 0.9994506711119726, -0.03308875944344066,
                                       0.29160284260165487, 0.03219521761769283, 0.9559975157651678);

       Mat T = (cv::Mat_<double>(3,1)  <<-0.09227808662414673, -0.0033723478007794756, -0.011952131992570284);

    //    Mat invR =  (cv::Mat_<double>(3,3) << 0.9996631201977612, -0.012962612752502763, -0.02248592419931352,//3*3
    //                                     0.0041044559927485525, 0.9344046817196847, -0.35618961835142043,
    //                                    0.025628100933783822, 0.35597733277689975, 0.9341431041288932);

    //    Mat invT = (cv::Mat_<double>(3,1)  <<-0.0036398917391895894, -0.11083650626991857, -0.015502244558523571);

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

/***********************************************************************************************************************************************************
//  */

    //    Mat M1 = (cv::Mat_<double>(3,3) << 232.86274725711323, 0, 248.54920673888924, 0, 233.09020302028782, 317.9606809629647, 0, 0, 1);
    
    //    Mat D1 = (cv::Mat_<double>(1,4) << 0.20617143277010225, -0.15538446737239678, 0.05679231144366941, -0.00949619709355899);


    //    Mat M2 = (cv::Mat_<double>(3,3) << 235.90521065154937, 0, 324.4166737350981, 0, 237.0782399056528, 253.42060933416013, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量


    //    Mat D2 = (cv::Mat_<double>(1,4) << 0.20494612169496526, -0.16426560252682942, 0.054774546156623374, -0.0071079055596136595);//k1,k2,p1,p2,k3
       
    //    Mat R = (cv::Mat_<double>(3,3) << 0.7980273594709841,  -0.584606518734723,  0.1462448350836408,//3*3
    //                                      0.18626820914754152, 0.47009098328793353, 0.8627390229335614,
    //                                      -0.5731112350990347, -0.6612485808561379, 0.48403907540508256);

    //    Mat T = (cv::Mat_<double>(3,1)  << 0.007618904570149553, 0.04394088227723727, -0.01905647376841649);
/*/**********************************************************************************************************************************************************/       
       cout<<"M1: "<<M1<<"\n" <<"M2: "<<M2<<"\n" <<"D1: "<<D1<<"\n" <<"D2: "<<D2<<"\n"<<"R: "<<R<<"\n"<<"T: "<<T<<"\n"<<endl;

    

       //校正之前，把om变为3*3;

       //Mat m_Rotation;

       //Rodrigues(R,m_Rotation);//注：matlab标定的参数中R为3*1，转换成3*3形式；
   for (int frame = 0; frame < img1s.size(); frame++)
    {
        /* code */
       cv::Mat img1=imread(img1s[frame]),imgLr;
       cv::Mat img2=imread(img2s[frame]),imgRr;
       cv::Mat img3=imread(img3s[frame]),imgULr;
       cv::Mat img4=imread(img4s[frame]),imgURr;
        img1 = rotate270(img1);
        img2 = rotate270(img2);
/*****************************************************************************************************************************
 * IMG1 & IMG2
*****************************************************************************************************************************/
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
       cv::fisheye::initUndistortRectifyMap(M1, //
                                            D1,//
                                            R1,//
                                            P1,//
                                            imageSize,
                                            CV_16SC2,
                                            remapmX1,
                                            remapmY1
                                            );  //④计算校正查找映射表，分别求映射矩阵

       cv::fisheye::initUndistortRectifyMap(M2, //
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

/*****************************************************************************************************************************
 *Stitcher img1 & img2
*****************************************************************************************************************************/
        // int cx = 314.1240768432617;
        // int cy = 180.386173248291;
        // imshow("undistort img3", imgULr);
        // imshow("undistort img4", imgURr);

        cv::Mat result1 = cv::Mat::zeros(imgRr.size(), CV_8U) ;
        // cv::Mat result2 = cv::Mat::zeros(imgURr.size(), CV_8U) ;
        std::vector<cv::Mat> images;
        images.push_back(imgLr);
        images.push_back(imgRr);

        // image2.push_back(imgULr);
        // image2.push_back(imgURr);
        cout << " images.size() : " << images.size() << endl;

        Stitcher::Mode mode = Stitcher::PANORAMA;
        Ptr<Stitcher> stitcher = Stitcher::create(mode);
        auto blender = detail::Blender::createDefault(detail::Blender::MULTI_BAND);
        stitcher->setBlender(blender);

        // auto fisheye_warper = makePtr<cv::FisheyeWarper>();
        // stitcher->setWarper(fisheye_warper);
        Stitcher::Status status = stitcher->stitch(images, result1);
        cout << "stitcher finish "<<endl;


        // status = stitcher->stitch(image2, result2);

        // result1=np.array(result1,dtype=np.uint8);
    //    imshow("imgLr",imgLr);
    //    imshow("imgRr",imgRr);
       imshow("result", result1);

    //    cout << "show result  "<<endl;
    //    imwrite("imgLr.png",imgLr);
    //    imwrite("imgRr.png",imgRr);//保存图片
       imwrite("MERGE_RESULT.png",result1);//保存图片
 


/*****************************************************************************************************************************
 *output img1 & img2
*****************************************************************************************************************************/

       Mat img(imageSize.height*0.5, imageSize.width, CV_8UC3);//⑥创建IMG，高度一样，宽度双倍
       Mat imgPart1 = img( Rect(0, 0, imageSize.width*0.5, imageSize.height*0.5) );//浅拷贝
       Mat imgPart2 = img( Rect(imageSize.width*0.5, 0, imageSize.width*0.5, imageSize.height*0.5) );//浅拷贝
       resize(imgLr, imgPart1, imgPart1.size(), 0, 0, cv::INTER_AREA);
        resize(imgRr, imgPart2, imgPart2.size(), 0, 0, cv::INTER_AREA);//改变图像尺寸，调节0,0；

    for( int i = 0; i < img.rows; i += 16 ) //画横线

        line(img, Point(0, i), Point(img.cols, i), Scalar(0, 255, 0), 1, 8);
        // imshow("rectified", img);
        imwrite("ret.png",img);
/*****************************************************************************************************************************
 * IMG1 & IMG3
*****************************************************************************************************************************/
       img1=imread(img1s[frame]);
       img2=imread(img2s[frame]);
       img3=imread(img3s[frame]);
       img4=imread(img4s[frame]);
        img1 = rotate270(img1);
        img2 = rotate270(img2);
    Size imageSize3 = Size(640, 1280);//图片尺寸
    imageSize = Size(480,1280);
        Mat R0_21,R0_22,P0_21,P0_22,Q0_2;
        if (alpha < 0 || alpha > 1)//裁剪系数，阈值
            alpha = -1;
    //    Rect roi1,roi2;//感兴趣区域
        stereoRectify(  M0_2, D0_2, M2_0,  D2_0,  imageSize,//图片尺寸
                        R0_2, T0_2, R0_21,//输出左相机的旋转矩阵
                        R0_22,//输出右相机的旋转矩阵
                        P0_21,//左投影矩阵
                        P0_22,//右投影矩阵
                        Q0_2, //重投影矩阵
                        0//CALIB_ZERO_DISPARITY ,//主点坐标相同
                        //-1,//裁剪系数
                        //imageSize,
                        //&roi1,
                        //&roi2
                        );//③：执行双目校正
        // img1 = fov_undistortion(img1, MM1, w_cam0);
        // img2 = fov_undistortion(img2, MM2, w_cam1);

    //    FileStorage fs_6("Q0_2.xml",FileStorage::WRITE);//               保 存Q阵
    //    fs_6<<"Q0_2"<<Q0_2;
    //    fs_6.release();
            cout<<"Q0_2: "<<Q0_2<<"\n"<<endl;
        cout<<"R0_21: "<<R0_21<<"\n" <<"R0_22: "<<R0_22<<"\n" <<"P0_21: "
        <<P0_21<<"\n" <<"P0_22: "<<P0_22<<"\n" <<endl;

        Mat remapmX4= Mat(imageSize,CV_32FC1);
        Mat remapmY4= Mat(imageSize,CV_32FC1);
        Mat remapmX3= Mat(imageSize3,CV_32FC1);
        Mat remapmY3= Mat(imageSize3,CV_32FC1);
        cv::fisheye::initUndistortRectifyMap(M0_2, //
                                            D0_2,//
                                            R0_21,//
                                            P0_21,//
                                            imageSize,
                                            CV_16SC2,
                                            remapmX4,
                                            remapmY4
                                            );  //④计算校正查找映射表，分别求映射矩阵

        cv::fisheye::initUndistortRectifyMap(M2_0, //
                                            D2_0, //
                                            R0_22, //
                                            P0_22, //
                                            imageSize3,
                                            CV_16SC2,
                                            remapmX3,
                                            remapmY3
                                            ); 
        // imgLr = fov_undistortion(imgLr, MM1, w_cam0);
        // imgRr = fov_undistortion(imgRr, MM2, w_cam1);

        if ( !remapmX4.empty() && !remapmY4.empty() )//⑤进行矫正，映射
        {
                remap( img1, imgLr, remapmX4, remapmY4, INTER_LINEAR );
        }
        if ( !remapmX3.empty() && !remapmY3.empty() )
        {
                remap( img3, imgULr, remapmX3, remapmY3, INTER_LINEAR );
        }
        // imshow("imgLr_ori",imgLr);
        // imshow("imgRr_ori",imgULr);
/*****************************************************************************************************************************
 *Stitcher img1 & img3
*****************************************************************************************************************************/
        // int cx = 314.1240768432617;
        // int cy = 180.386173248291;
        // imshow("undistort img3", imgULr);
        // imshow("undistort img4", imgURr);

        cv::Mat result2 = cv::Mat::zeros(imgULr.size(), CV_8U) ;
        // cv::Mat result2 = cv::Mat::zeros(imgURr.size(), CV_8U) ;
        std::vector<cv::Mat> images2;
        images2.push_back(imgLr);
        images2.push_back(imgULr);

        // image2.push_back(imgULr);
        // image2.push_back(imgURr);
        cout << " images2.size() : " << images2.size() << endl;

        Stitcher::Mode mode2 = Stitcher::PANORAMA;
        Ptr<Stitcher> stitcher2 = Stitcher::create(mode2);
        auto blender2 = detail::Blender::createDefault(detail::Blender::MULTI_BAND);
        stitcher2->setBlender(blender2);

        // auto fisheye_warper = makePtr<cv::FisheyeWarper>();
        // stitcher->setWarper(fisheye_warper);
        Stitcher::Status status2 = stitcher2->stitch(images2, result2);
        cout << "stitcher2 finish "<<endl;


        // status = stitcher->stitch(image2, result2);

        // result2=np.array(result2,dtype=np.uint8);
        // imshow("imgLr2",imgLr);
        // imshow("imgULr",imgULr);
        imshow("result2", result2);

        // cout << "show result  "<<endl;
    //    imwrite("imgLr2.png",imgLr);
        // imwrite("imgULr.png",imgULr);//保存图片
        imwrite("MERGE_RESULT2.png",result2);//保存图片

/*****************************************************************************************************************************
 *output img1 & img3
*****************************************************************************************************************************/

        Mat img_2(imageSize.height*0.5, imageSize.width, CV_8UC3);//⑥创建IMG，高度一样，宽度双倍
        Mat imgPart3 = img_2( Rect(0, 0, imageSize.width*0.5, imageSize.height*0.5) );//浅拷贝
        Mat imgPart4 = img_2( Rect(imageSize.width*0.5, 0, imageSize.width*0.5, imageSize.height*0.5) );//浅拷贝
        resize(imgLr, imgPart3, imgPart3.size(), 0, 0, cv::INTER_AREA);
        resize(imgULr, imgPart4, imgPart4.size(), 0, 0, cv::INTER_AREA);//改变图像尺寸，调节0,0；

    for( int i = 0; i < img_2.rows; i += 16 ) //画横线

        line(img_2, Point(0, i), Point(img_2.cols, i), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", img_2);
        imwrite("ret.png",img_2);
        waitKey();
    }
        return 0;
}
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <math.h>
#include <iostream>   // Include OpenCV API
#include <fstream>
#include <unistd.h>
#define CVAUX_CONCAT_EXP(a, b) a##b
using namespace cv;
using namespace std;

cv::String path1 = "/home/lan/Dataset/new_vr1/cam0";
cv::String path2 = "/home/lan/Dataset/new_vr1/cam1";
cv::String path3 = "/home/lan/Dataset/new_vr1/cam2";
cv::String path4 = "/home/lan/Dataset/new_vr1/cam3";


cv::Mat rotate270(cv::Mat src)
{
	cv::Mat dst;//目标图像
    cv::transpose(src, dst);
    cv::flip(dst, dst, 1);

	return dst;
}

cv::Mat image_combine(cv::Mat frame1, cv::Mat frame2, cv::Mat frame3, cv::Mat frame4)
{
        cv::Mat combine = cv::Mat(1200, 1200, CV_8UC1);

        cv::Mat imgROI = combine(Rect(0, 250, frame1.cols, frame1.rows));
        addWeighted(frame1, 1, imgROI, 1, 0, imgROI);

        imgROI = combine(Rect(60, 260, frame2.cols, frame2.rows));
        addWeighted(frame2, 1, imgROI, 1, 0, imgROI);

        imgROI = combine(Rect(500, 0, frame3.cols, frame3.rows));
        addWeighted(frame3, 1, imgROI, 1, 0, imgROI);

        imgROI = combine(Rect(0, 0, frame4.cols, frame4.rows));
        addWeighted(frame4, 1, imgROI, 1, 0, imgROI);

        return combine;
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

void initUndistortRectifyMapA( InputArray K, InputArray D, InputArray R, InputArray P,
    const cv::Size& size, int m1type, OutputArray map1, OutputArray map2 )
{
    // ofstream imu0;
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    CV_Assert((K.depth() == CV_32F || K.depth() == CV_64F) && (D.depth() == CV_32F || D.depth() == CV_64F));
    CV_Assert((P.empty() || P.depth() == CV_32F || P.depth() == CV_64F) && (R.empty() || R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(K.size() == Size(3, 3) && (D.empty() || D.total() == 4));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        Matx33f camMat = K.getMat();
        f = Vec2f(camMat(0, 0), camMat(1, 1));
        c = Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        Matx33d camMat = K.getMat();
        f = Vec2d(camMat(0, 0), camMat(1, 1));
        c = Vec2d(camMat(0, 2), camMat(1, 2));
    }

    Vec4d k = Vec4d::all(0);
    if (!D.empty())
        k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();

    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);

    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.getMat().ptr<float>(i);
        float* m2f = map2.getMat().ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            double u, v;
            if( _w <= 0)
            {
                u = (_x > 0) ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
                v = (_y > 0) ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
            }
            else
            {
                double x = _x/_w, y = _y/_w;

                double r = sqrt(x*x + y*y);
                double theta = atan(r);

                double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
                double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);

                double scale = (r == 0) ? 1.0 : theta_d / r;
                u = f[0]*x*scale + c[0] ;
                v = f[1]*y*scale + c[1] ;

                // imu0.open("./record/imu0/data.csv", ios::app);
                // imu0 << "u:" << "\t" << u <<"v:" << "\t" << v << endl;
                // imu0.close();
            }

            if( m1type == CV_16SC2 )
            {
                int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }

            _x += (iR(0, 0));
            _y += (iR(1, 0));
            _w += (iR(2, 0));
        }
    }
}

void initUndistortRectifyMapB( InputArray K, InputArray D, InputArray R, InputArray P,
    const cv::Size& size, int m1type, OutputArray map1, OutputArray map2 )
{

    CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    CV_Assert((K.depth() == CV_32F || K.depth() == CV_64F) && (D.depth() == CV_32F || D.depth() == CV_64F));
    CV_Assert((P.empty() || P.depth() == CV_32F || P.depth() == CV_64F) && (R.empty() || R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(K.size() == Size(3, 3) && (D.empty() || D.total() == 4));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        Matx33f camMat = K.getMat();
        f = Vec2f(camMat(0, 0), camMat(1, 1));
        c = Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        Matx33d camMat = K.getMat();
        f = Vec2d(camMat(0, 0), camMat(1, 1));
        c = Vec2d(camMat(0, 2), camMat(1, 2));
    }

    Vec4d k = Vec4d::all(0);
    if (!D.empty())
        k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();

    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);

    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.getMat().ptr<float>(i);
        float* m2f = map2.getMat().ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            double u, v;
            if( _w <= 0)
            {
                u = (_x > 0) ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
                v = (_y > 0) ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
            }
            else
            {
                double x = _x/_w, y = _y/_w;

                double r = sqrt(x*x + y*y);
                double theta = atan(r);

                double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
                double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);

                double scale = (r == 0) ? 1.0 : theta_d / r;
                u = f[0]*x*scale + c[0] ;
                v = f[1]*y*scale + c[1] ;
            }

            if( m1type == CV_16SC2 )
            {
                int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }

            _x += (iR(0, 0));
            _y += (iR(1, 0));
            _w += (iR(2, 0));
        }
    }
}

//输入顺序：图像，目标相机内参K1，目标相机外参旋转R1，目标相机外参平移T1，本相机内参K2
cv::Mat normalized(cv::Mat img, cv::Mat K1, cv::Mat R1, cv::Mat T1, cv::Mat K2)
{

    cv::Mat after_norm = cv::Mat(img.rows, img.cols, CV_8UC1);
    cout << "rows : " << img.rows << "cols :" << img.cols << endl;

    for (double u = 0; u < img.rows; u++)
    {
        for (double v = 0; v < img.cols; v++)
        {
            cv::Mat ori = (cv::Mat_<double>(3, 1) << u, v, 1.0); //(行数，列数)
            cv::Mat trans;

            //R 相机位姿* 世界坐标 + t 相机位姿 = 归一化坐标  
            //像素 = 内参K * 归一化坐标
            trans = K2.inv() * ori ;//世界坐标 = 外参旋转的逆 *（相机内参的逆 * 像素 - 相机外参的位移）

            trans = K1 * (R1 * trans + T1);    
            trans /= trans.at<double>(0,2);                               //世界坐标归一化
                
            // trans(1) += 300; 

            // cout << "trans: "<< trans(0) << "\t" << trans(1) << "\t" << trans(2) << "\t" << endl;
            if (trans.at<double>(0, 0) >= 0 && trans.at<double>(0, 1) >= 0 && trans.at<double>(0, 0) < img.cols && trans.at<double>(0, 1) < img.rows){
                after_norm.at<u_char>(v,u) =img.at<u_char>((int) trans.at<double>(0, 1) ,(int) trans.at<double>(0, 0));
            }
            else{
                after_norm.at<u_char>(v,u) = 0;
            }    
        }
    }

    return after_norm;
}
int main(int argc, char * argv[])
{


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
    Mat R2_0 = (cv::Mat_<double>(3,3) << 0.7982476891721761, 0.1866317764801181, -0.5726859582172034,//3*3
                                        -0.5843317261179621, 0.4706315955163437, -0.6611069014567492,
                                   0.14614055078342905, 0.862365630872205, 0.48473545167142473);

    Mat T0_2 = (cv::Mat_<double>(3,1)  << 0.007545682217839094, 0.04403714166115202, -0.019400920740212973);
    Mat T2_0 = (cv::Mat_<double>(3,1)  << -0.02477449376887072, -0.029835683201294205, -0.029080335077192997);
/***********************************************************************************************************************************************************/

/***********************************************************************************************************************************************************/

    Mat M1_3 = (cv::Mat_<double>(3,3) << 234.6462919546538, 0, 249.175058068484224, 0, 234.05833126216152, 317.6255028053466, 0, 0, 1);
    
    Mat D1_3 = (cv::Mat_<double>(1,4) << 0.21312804944305436, -0.18938066909671794, 0.08096681421032965, -0.015400285664844176);


    Mat M3_1 = (cv::Mat_<double>(3,3) << 235.97841549146239, 0, 321.56637232071205, 0, 235.18921690631257, 230.9498875099282, 0, 0, 1);//相机参数，畸变系数，旋转矩阵，平移向量


    Mat D3_1 = (cv::Mat_<double>(1,4) << 0.17813568748487207, -0.10828019070960325, 0.026558424840632575, -0.002713435374528129);//k1,k2,p1,p2,k3
    
    Mat R1_3 = (cv::Mat_<double>(3,3) << 0.7424536191626323, 0.6512369292337921, -0.157013010271238,//3*3
                                        -0.13884229097304374, 0.3788880085586133, 0.9149681389031156,
                                        0.6553513879054896, -0.657521360088488, 0.3717260004327371);
    Mat R3_1 = (cv::Mat_<double>(3,3) << 0.7394067170143956, -0.1383610364180931, 0.6588884051455938,//3*3
                                        0.6558133076364507, 0.36936068484483725, -0.6583931880099386,
                                        -0.1522715086972316, 0.9189281299929083, 0.3638467803166671);

    Mat T1_3 = (cv::Mat_<double>(3,1)  << 0.02335838256311979, -0.03275110452665002, -0.029256865189976593);
    Mat T3_1 = (cv::Mat_<double>(3,1)  << 0.02335838256311979, -0.03275110452665002, -0.029256865189976593);
/***********************************************************************************************************************************************************/       

    //摄像头启动
   /* VideoCapture cap1(0);
    VideoCapture cap2(1);*/
    std::vector<cv::String> left1;
    std::vector<cv::String> right1;
    std::vector<cv::String> left2;
    std::vector<cv::String> right2;

    cv::glob(path1, left1);
    cv::glob(path2, right1);
    cv::glob(path3, left2);
    cv::glob(path4, right2);

    for(int frame = 0 ; frame < left1.size() ; frame ++ )
    {
        cv::Mat frame1 = imread(left1[frame], IMREAD_GRAYSCALE),imgLr;
        cv::Mat frame2 = imread(right1[frame], IMREAD_GRAYSCALE),imgRr;
        cv::Mat frame3 = imread(left2[frame], IMREAD_GRAYSCALE),imgULr;
        cv::Mat frame4 = imread(right2[frame], IMREAD_GRAYSCALE),imgURr;

        frame1 =  rotate270(frame1);
        frame2 =  rotate270(frame2);
        // frame4 =  rotate270(frame4);
        
/*****************************************************************************************************************************
 * IMG1 & IMG2
*****************************************************************************************************************************/
        Size imageSize=frame1.size();//图片尺寸
        Size imageSizeUp=cv::Size(700,700);//图片尺寸
        Size imageSizeLeftUp=cv::Size(700,1000);//图片尺寸
        Mat R1,R2,P1,P2,Q;
       double alpha=-1;
       if (alpha < 0 || alpha > 1)//裁剪系数，阈值
        alpha = -1;
       Rect roi1,roi2;//感兴趣区域
       cout << "stereoRectify" << endl; 
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
        cout << "stereoRectify done!" << endl; 
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

        Mat remapmX3= Mat(imageSizeUp,CV_32FC1);
        Mat remapmY3= Mat(imageSizeUp,CV_32FC1);
        // Mat remapmX4= Mat(cv::Size(2000, 2000),CV_32FC1);
        // Mat remapmY4= Mat(cv::Size(2000, 2000),CV_32FC1);     
        Mat remapmX4= Mat(imageSizeUp,CV_32FC1);
        Mat remapmY4= Mat(imageSizeUp,CV_32FC1);
        // Mat remapmX4= Mat(cv::Size(frame3.size()),CV_32FC1);
        // Mat remapmY4= Mat(cv::Size(frame3.size()),CV_32FC1);
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
        initUndistortRectifyMapA(M2_0, //
                                D2_0,//
                                R2_0*R1,//
                                P1,//
                                imageSizeUp,
                                CV_16SC2,
                                remapmX3,
                                remapmY3
                                );  //④计算校正查找映射表，分别求映射矩阵
        initUndistortRectifyMapB(M3_1, //
                                D3_1,//
                                R3_1*R2,//
                                P2,//
                                imageSizeUp,
                                CV_16SC2,
                                remapmX4,
                                remapmY4
                                );  //④计算校正查找映射表，分别求映射矩阵

        // imgLr = fov_undistortion(imgLr, MM1, w_cam0);
        // imgRr = fov_undistortion(imgRr, MM2, w_cam1);
       if ( !remapmX1.empty() && !remapmY1.empty() )//⑤进行矫正，映射
       {
              remap( frame1, imgLr, remapmX1, remapmY1, INTER_LINEAR );
       }
       if ( !remapmX2.empty() && !remapmY2.empty() )
       {
              remap( frame2, imgRr, remapmX2, remapmY2, INTER_LINEAR );
       }
        if ( !remapmX3.empty() && !remapmY3.empty() )
       {
              remap( frame3, imgULr, remapmX3, remapmY3, INTER_LINEAR );
       }
        if ( !remapmX4.empty() && !remapmY4.empty() )
       {
              remap( frame4, imgURr, remapmX4, remapmY4, INTER_LINEAR );
       }

/**********************************************************************
 * 相机基线对齐及图像融合
 * *******************************************************************/  
        // cout <<" frame2.rows :" << frame2.rows << " frame2.cols :" << frame2.cols << endl;       
        // cout <<" frame3.rows :" << frame3.rows << " frame3.cols :" << frame3.cols << endl;       
        // resize(frame3, frame3, cv::Size(1000, 1000));
        // frame1 = normalized(frame1, M1, R1, T, M1);
        // frame2 = normalized(frame2, M2, R2, T, M2);
        // frame3 = normalized(frame3, M2_0, R2_0*R1, T0_2, M2_0);
        // frame3 = normalized(frame3, M2_0, R2_0 , -T0_2, M1, R, T);
        cvtColor(imgLr, imgLr, CV_GRAY2RGB);
        cvtColor(imgRr, imgRr, CV_GRAY2RGB);
        cvtColor(imgULr, imgULr, CV_GRAY2RGB);
        cvtColor(imgURr, imgURr, CV_GRAY2RGB);
        imshow("imgLr ", imgLr);
        imshow("imgRr ", imgRr);

        imshow("imgURr ", imgURr);
        imshow("frame1 ", frame1);
        imshow("frame2 ", frame2);
        imshow("frame3 ", frame3);
        imshow("frame4 ", frame4);
        // cv::Mat combine = image_combine(destImage2, destImage3, destImage1, frame3);

/**********************************************************************
 * 相机归一化坐标
 * *******************************************************************/
        imwrite("./merge_coordinate/cam0_" + std::to_string(frame) + ".png", frame1);
        imwrite("./merge_coordinate/cam1_" + std::to_string(frame) + ".png", frame2);
        imwrite("./merge_coordinate/cam2_" + std::to_string(frame) + ".png", frame3);
        imwrite("./merge_coordinate/cam3_" + std::to_string(frame) + ".png", frame4);

        cout << frame << " frame is finish" << endl;

        cv::Mat result1 = cv::Mat::zeros(imgURr.size(), CV_8UC3) ;
        // cv::Mat result2 = cv::Mat::zeros(imgURr.size(), CV_8U) ;
        std::vector<cv::Mat> images;
        images.push_back(imgLr);
        images.push_back(imgRr);
        images.push_back(imgULr);
        images.push_back(imgURr);

        cout << " images.size() : " << images.size() << endl;

        Stitcher::Mode mode = Stitcher::PANORAMA;
        Ptr<Stitcher> stitcher = Stitcher::create(mode);
        auto blender = detail::Blender::createDefault(detail::Blender::MULTI_BAND);
        stitcher->setBlender(blender);
        Stitcher::Status status = stitcher->stitch(images, result1);
        cout << "stitcher finish "<<endl;
        if (status != Stitcher::OK)
        {
            cout << "Can't stitch images, error code = " << int(status) << endl;
            return EXIT_FAILURE;
        }
       imshow("result", result1);
        imshow("imgULr ", imgULr);
        imwrite("./merge_coordinate/result_" + std::to_string(frame) + ".png", result1);
        waitKey();
    }    

    return 0;
}

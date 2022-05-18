#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>

using namespace cv;
using namespace std;


//柱面投影
Mat cylinder(Mat imgIn, int f);
//计算两幅图像之间的平移量（模板匹配，归一化相关性匹配法）
Point2i getOffset(Mat img, Mat img1);
//线性融合（渐入渐出融合法）
Mat linearFusion(Mat img, Mat img1, Point2i a);
cv::String path1 = "/home/lan/Dataset/TUM_VI/noApril_tum/mav0/cam0/data";
cv::String path2 = "/home/lan/Dataset/TUM_VI/noApril_tum/mav0/cam1/data";
int main()
{
    // string path1 = "C:/Users/dell/Desktop/SURF/video_1_1.mp4";//左图
    // string path2 = "C:/Users/dell/Desktop/SURF/video_3_1.mp4";//右图
    // VideoCapture cap1(path1);
    // VideoCapture cap2(path2);

    //摄像头启动
   /* VideoCapture cap1(0);
    VideoCapture cap2(1);*/

    double rate = 60;
    int delay = 1000 / rate;
    bool stop(false);
    std::vector<cv::String> left1;
    std::vector<cv::String> right1;
    cv::Mat frame1;
    cv::Mat frame2;
    cv::Mat frame3;
    Point2i a;//存储偏移量
    int k = 0;

    cv::glob(path1, left1);
    cv::glob(path2, right1);

    for(int frame = 0 ; frame < left1.size() ; frame ++ )
    {
        frame1 = imread(left1[frame], IMREAD_GRAYSCALE);
        frame2 = imread(right1[frame], IMREAD_GRAYSCALE);

        cv::namedWindow("cam1", WINDOW_AUTOSIZE);  //opencv4.2.0将CV_WINDOW_AUTOSIZE改为WINDOW_AUTOSIZE
        cv::namedWindow("cam2", WINDOW_AUTOSIZE);
        cv::namedWindow("stitch",WINDOW_AUTOSIZE);
        cout << "make the windows" << endl;
        // if (cap1.isOpened() && cap2.isOpened())
        // {
        //     cout << "*** ***" << endl;
        //     cout << "摄像头已启动！" << endl;
        // }
        // else
        // {
        //     cout << "*** ***" << endl;
        //     cout << "警告：摄像头打开不成功或者未检测到有两个摄像头!" << endl;
        //     cout << "程序结束！" << endl << "*** ***" << endl;
        //     return -1;
        // }

        //cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        //cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
        //cap2.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        //cap2.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
        // cap1.set(CAP_PROP_FOCUS, 0);   //opencv4.2.0将CV_CAP_PROP_FOCUS改为CAP_PROP_FOCUS；定义摄像头参数cap.set
        // cap2.set(CAP_PROP_FOCUS, 0);

        // while (!stop)
        // {
            // if (cap1.read(frame1) && cap2.read(frame2)){}
                imshow("cam1", frame1);
                imshow("cam2", frame2);
                cout << "show the frame" << endl;
                //彩色帧转灰度
                // cvtColor(frame1, frame1, COLOR_RGB2GRAY);  //opencv4.2.0将CV_RGB2GRAY改为COLOR_RGB2GRAY
                // cvtColor(frame2, frame2, COLOR_RGB2GRAY);
                
                //柱面投影变换
                //frame1 = cylinder(frame1, 1005);
                //frame2 = cylinder(frame2, 1005);
                //匹配和拼接
                /*视频拼接通过while循环实现，下面这个判断的意思是，有两
                *种情形才计算平移参数，一是程序启动时，前3个循环内；二
                *是按下回车键时。这样在场景和摄像头相对固定时，避免了平
                *移量的重复计算，提高了拼接的实时性
                */
                if (k < 3 || waitKey(delay) == 13)//按回车键
                {
                    cout << "正在匹配..." << endl;
                    a = getOffset(frame1, frame2);
                }
                frame3 = linearFusion(frame1, frame2, a);

                imshow("stitch", frame3);
                imwrite("./merge_coordinate/" + std::to_string(frame) + ".jpg", frame3);
                k++;

            // }
            // else
            // {
            //     cout << "----------------------" << endl;
            //     cout << "waitting..." << endl;
            // }
            //按下ESC键，退出循环，程序结束
            // if (waitKey(1) == 27)
            // {
            //     stop = true;
            //     cout << "程序结束！" << endl;
            //     cout << "*** ***" << endl;
            // }
        // }
    }
    waitKey(0);
    return 0;
}

//计算平移参数
Point2i getOffset(Mat img, Mat img1)
{
    Mat templ(img1, Rect(0, 0.4 * img1.rows, 0.2 * img1.cols, 0.2 * img1.rows));
    Mat result(img.cols - templ.cols + 1, img.rows - templ.rows + 1, CV_8UC1);//result存放匹配位置信息
    matchTemplate(img, templ, result, TM_CCORR_NORMED);  //opencv4.2.0将CV_TM_CCORR_NORMED改为TM_CCORR_NORMED
    normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
    double minVal; double maxVal; Point minLoc; Point maxLoc; Point matchLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    matchLoc = maxLoc;//获得最佳匹配位置
    int dx = matchLoc.x;
    int dy = matchLoc.y - 0.4 * img1.rows;//右图像相对左图像的位移
    Point2i a(dx, dy);
    return a;
}

//线性（渐入渐出）融合
Mat linearFusion(Mat img, Mat img1, Point2i a)
{
    int d = img.cols - a.x;//过渡区宽度
    int ms = img.rows - abs(a.y);//拼接图行数
    int ns = img.cols + a.x;//拼接图列数
    Mat stitch = Mat::zeros(ms, ns, CV_8UC1);
    //拼接
    Mat_<uchar> ims(stitch);
    Mat_<uchar> im(img);
    Mat_<uchar> im1(img1);

    if (a.y >= 0)
    {
        Mat roi1(stitch, Rect(0, 0, a.x, ms));
        img(Range(a.y, img.rows), Range(0, a.x)).copyTo(roi1);
        Mat roi2(stitch, Rect(img.cols, 0, a.x, ms));
        img1(Range(0, ms), Range(d, img1.cols)).copyTo(roi2);
        for (int i = 0; i < ms; i++)
            for (int j = a.x; j < img.cols; j++)
                ims(i, j) = uchar((img.cols - j) / float(d) * im(i + a.y, j) + (j - a.x) / float(d) * im1(i, j - a.x));

    }
    else
    {
        Mat roi1(stitch, Rect(0, 0, a.x, ms));
        img(Range(0, ms), Range(0, a.x)).copyTo(roi1);
        Mat roi2(stitch, Rect(img.cols, 0, a.x, ms));
        img1(Range(-a.y, img.rows), Range(d, img1.cols)).copyTo(roi2);
        for (int i = 0; i < ms; i++)
            for (int j = a.x; j < img.cols; j++)
                ims(i, j) = uchar((img.cols - j) / float(d) * im(i, j) + (j - a.x) / float(d) * im1(i + abs(a.y), j - a.x));
    }


    return stitch;
}

//柱面投影校正
Mat cylinder(Mat imgIn, int f)
{
    int colNum, rowNum;
    colNum = 2 * f * atan(0.5 * imgIn.cols / f);//柱面图像宽
    rowNum = 0.5 * imgIn.rows * f / sqrt(pow(f, 2)) + 0.5 * imgIn.rows;//柱面图像高

    Mat imgOut = Mat::zeros(rowNum, colNum, CV_8UC1);
    Mat_<uchar> im1(imgIn);
    Mat_<uchar> im2(imgOut);

    //正向插值
    int x1(0), y1(0);
    for (int i = 0; i < imgIn.rows; i++)
        for (int j = 0; j < imgIn.cols; j++)
        {
            x1 = f * atan((j - 0.5 * imgIn.cols) / f) + f * atan(0.5 * imgIn.cols / f);
            y1 = f * (i - 0.5 * imgIn.rows) / sqrt(pow(j - 0.5 * imgIn.cols, 2) + pow(f, 2)) + 0.5 * imgIn.rows;
            if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum)
            {
                im2(y1, x1) = im1(i, j);
            }
        }
    return imgOut;
}

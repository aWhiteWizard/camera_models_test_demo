/************************************************************
 * WhiteWizard Designed in 2022.5.17
 * *********************************************************/

#include "opencv_orb_match.h"

using namespace cv;
using namespace std;

String img_left ="/home/lan/Dataset/TUM_VI/noApril_tum/mav0/cam0/data";
String img_right ="/home/lan/Dataset/TUM_VI/noApril_tum/mav0/cam1/data";

void linespace(Mat&image,float begin,float finish,int w1,Mat&mask);
void generate_mask(Mat&img,Mat&mask);
 

int main(int argc, char const *argv[])
{
    std::vector<cv::String> img_lefts;
    std::vector<cv::String> img_rights;

    cv::glob(img_left,img_lefts);
    cv::glob(img_right,img_rights);

/******************************************************
 * 读取双目图像
 * 
*******************************************************/
    if(img_lefts.size() == 0)
        cout << "left cam no data inputs,Please Check" << endl;
    
    if(img_rights.size() == 0)
        cout << "right cam no data inputs,Please Check" << endl;
    
    if(img_lefts != img_rights)
        cout << "data of two cam is NOT equi,PLEASE CHECK!" << endl;
    
    for(int frame = 0 ; frame < img_lefts.size() ; frame ++ )
    {
        cv::Mat lefts = imread(img_lefts[frame], IMREAD_GRAYSCALE);
        cv::Mat rights = imread(img_rights[frame], IMREAD_GRAYSCALE);
        assert(lefts.data != nullptr && rights.data != nullptr);
        
        std::vector<KeyPoint> Keypoint1,Keypoint2;
        cv::Mat descriptor1, descriptor2;
        Ptr<FeatureDetector> decetor = ORB::create();
        Ptr<FeatureDetector> discriptor = ORB::create();
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        decetor -> detect(lefts, Keypoint1);
        decetor -> detect(rights, Keypoint2);

        discriptor->compute(lefts, Keypoint1, descriptor1);
        discriptor->compute(rights, Keypoint2, descriptor2);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
        cout <<"extract ORB cost = " << time_used.count() << "seconds." << endl;

        Mat outimg1;
        drawKeypoints(lefts, Keypoint1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        // imwrite("ORB_features.png", outimg1);

        vector<DMatch> matches;
        t1 = chrono::steady_clock::now();
        matcher -> match(descriptor1, descriptor2, matches);
        t2 = chrono::steady_clock::now();
        time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "ORB Matches time = " << time_used.count() << endl;

        auto max_min = minmax_element(matches.begin(), matches.end(), [](const DMatch &m1, const DMatch &m2) {return m1.distance < m2.distance;} );
        double min_dist = max_min.first->distance;
        double max_dist = max_min.second->distance;

        cout << "---MAX dist: " << max_dist << endl;
        cout << "---MIN dist: " << min_dist << endl;

        std::vector<DMatch> good_matches;
        std::vector<Point2f> pic_point_1;
        std::vector<Point2f> pic_point_2;
        for (int i = 0; i < descriptor1.rows ; i++)
        {
            if(matches[i].distance <= max(2*min_dist, 30.0))
                {
                    good_matches.push_back(matches[i]);
                }
        }

        for (int t = 0; t < good_matches.size(); t++)
        {
            pic_point_1.push_back(Keypoint1[good_matches[t].queryIdx].pt);
            pic_point_2.push_back(Keypoint1[good_matches[t].queryIdx].pt); 
        }
        cout << "pic_point_push-back" << endl;
        Mat hamming = findHomography(pic_point_1, pic_point_2, RANSAC);

        int mix_high = max(lefts.rows, rights.rows);
        int mix_width = lefts.cols + rights.cols;
        Mat mix_pic_01 = Mat::zeros(Size(mix_width, mix_high),CV_8UC1);
        cv::Rect roi;
        roi.x = 0;
        roi.y = 0;
        roi.width = lefts.cols;
        roi.height = lefts.rows;

        cout << "copy to pic" << endl;
        //对齐图像
        lefts.copyTo(mix_pic_01(roi));
        // imshow("mix_pic_01", mix_pic_01);

        cv::Mat mix_pic_02;
        warpPerspective(rights, mix_pic_02, hamming, Size(mix_width, mix_high));
        // imshow("mix_pic_02", mix_pic_02);
        cout << "mask creative" << endl;
        //生成mask
        cv::Mat mask = cv::Mat::zeros(Size(mix_width, mix_high), CV_8UC1);
        generate_mask(mix_pic_02,mask);
        // imshow("mask", mask);
        cout << "mask diff" << endl;
        //初始化mask权重
        cv::Mat mask1 = cv::Mat::ones(Size(mix_width, mix_high), CV_8UC1);
        cv::Mat mask2 = cv::Mat::ones(Size(mix_width, mix_high), CV_8UC1);
        cout << "left & right mask" << endl;
        //left mask1
        linespace(mask1, 1, 0, lefts.cols, mask);
        // imshow("mask1", mask1);
        //right mask2
        linespace(mask2, 1, 0, rights.cols, mask);
        // imshow("mask2", mask2);
        cout << "left multiply" << endl;
                
        Mat m_left;
        std::vector<Mat> mv;
        mv.push_back(mask1);
        mv.push_back(mask1);
        mv.push_back(mask1);
        cout << "left merge" << endl;
        merge(mv,m_left);
        cout << "left convertTo" << endl;
        mix_pic_01.convertTo(mix_pic_01, CV_8UC1);
        cv::imwrite("./merge/" + std::to_string(frame) + "ori_mix_pic_01.jpg", mix_pic_01);
        cout << "left multiply_real" << endl;
        cout << "size of mix_pic_01 is : " << mix_pic_01.size() << endl;
        cout << "size of m_left is : " << m_left.size() << endl;
        
        cv::multiply(mix_pic_01, m_left, mix_pic_01);// 这里很奇妙的时好时不好，应该是前面keypoints的问题，可以参考例程改一下

        cv::imwrite("./merge/" + std::to_string(frame) + "mix_pic_01.jpg", mix_pic_01);
        cout << "right merge" << endl;    
        Mat m_right;
        mv.push_back(mask2);
        merge(mv,m_right);
        cout << "rignt convertTo " << endl;
        mix_pic_02.convertTo(mix_pic_02, CV_8UC1);
        cout << "right multiply_real " << endl;
        cout << "size of mix_pic_02 is : " << mix_pic_02.size() << endl;
        cout << "size of m_right is : " << m_right.size() << endl;
        multiply(mix_pic_02, m_right, mix_pic_02);
        cout << "merge the pic" << endl;
        //merge
        cv::Mat mix_pic;
        add(mix_pic_01, mix_pic_02, mix_pic);
        mix_pic.convertTo(mix_pic, CV_8U);
        imshow("mix_pic", mix_pic);
        imwrite("./merge/" + std::to_string(frame) + "miximage.jpg", mix_pic);


    // cv::Mat img_match;
    // cv::Mat img_goodmatch;

    // drawMatches(lefts, Keypoint1, rights, Keypoint2, matches, img_match);
    // drawMatches(lefts, Keypoint1, rights, Keypoint2, good_matches, img_goodmatch);

    // imshow("matches", img_match);
    // imshow("good matcher", img_goodmatch);
    
    // imwrite("./orb_match/"+std::to_string(frame)+"match.jpg", img_match);
    // imwrite("./orb_match/"+std::to_string(frame)+"good_match.jpg", img_goodmatch);
    }
    waitKey(0);
    return 0;

}
/*******************************************************************
 * 生成mask的函数
 * ***************************************************************/
void generate_mask(Mat&img, Mat&mask) {
	int w = img.cols;
	int h = img.rows;
	for (int row = 0; row < h;row++) {
		for (int col = 0; col < w; col++) {
			Vec3b p = img.at<Vec3b>(row,col);
			int b = p[0];
			int g = p[1];
			int r = p[2];
			if (b == g && g == r && r == 0) {
				mask.at<uchar>(row, col) = 255;
			}
 
		}
		
	}
}

void linespace(Mat&image, float begin, float finish, int w1, Mat&mask) {
	int offstx = 0;
	float interval = 0;
	float delta = 0;
	for (int i = 0; i < image.rows; i++) {
		offstx = 0;
		interval = 0;
		delta = 0;
		for (int j = 0; j < image.cols; j++) {
			int pv = mask.at<uchar>(i, j);
			if (pv == 0 && offstx == 0) {
				offstx = j;
				delta = w1 - offstx;
				interval = (finish-begin) / (delta - 1);
				image.at<float>(i, j) = begin + (j - offstx)*interval;
			}
			else if(pv==0&&offstx>0&&(j-offstx)<delta){
				image.at<float>(i, j) = begin + (j - offstx)*interval;
			}
			 
		}
	}
}
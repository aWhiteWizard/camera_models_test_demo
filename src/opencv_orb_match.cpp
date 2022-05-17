#include "opencv_orb_match.h"

using namespace cv;
using namespace std;

String img_left ="/home/lan/Dataset/TUM_VI/noApril_tum/mav0/cam0/data";
String img_right ="/home/lan/Dataset/TUM_VI/noApril_tum/mav0/cam1/data";

int main(int argc, char const *argv[])
{
    std::vector<cv::String> img_lefts;
    std::vector<cv::String> img_rights;

    cv::glob(img_left,img_lefts);
    cv::glob(img_right,img_rights);



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
        imwrite("ORB_features.png", outimg1);

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
        for (int i = 0; i < descriptor1.rows ; i++)
        {
            if(matches[i].distance <= max(1.33*max_dist, 30.0))
                good_matches.push_back(matches[i]);
        }
    cv::Mat img_match;
    cv::Mat img_goodmatch;

    drawMatches(lefts, Keypoint1, rights, Keypoint2, matches, img_match);
    drawMatches(lefts, Keypoint1, rights, Keypoint2, good_matches, img_goodmatch);

    imshow("matches", img_match);
    imshow("good matcher", img_goodmatch);
    
    imwrite("./orb_match/"+std::to_string(frame)+"match.jpg", img_match);
    imwrite("./orb_match/"+std::to_string(frame)+"good_match.jpg", img_goodmatch);
    }
    waitKey(0);
    return 0;

}

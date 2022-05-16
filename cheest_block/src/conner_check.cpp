#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    // 内部角点的每行个数和每列个数，即黑色方块的交点
    cv::Size pattern(5, 7);
    cv::Mat image = cv::imread(argv[1], 0);
    std::vector<cv::Point2f> corners;
    bool patternfound = findChessboardCorners(image, pattern, corners);
    std::cout << patternfound << std::endl;
    if (patternfound)
        cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
    drawChessboardCorners(image, pattern, cv::Mat(corners), patternfound);
    cv::imshow("res",image);
    cv::waitKey(0);

    return 0;
}

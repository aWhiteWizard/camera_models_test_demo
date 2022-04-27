/*
    Pinhole Camera models define
*/
#include<camera_models.hpp>
#include<Eigen/Core>
#include<opencv4/opencv2/core>

using namespace cv;

Eigen::Matrix2d Pinhole_3Dto2D(Eigen::Matrix3d Point)
{
        Eigen::Vector2d res;
    res[0] = mvParameters[0] * Point[0] / Point[2] + mvParameters[2];
    res[1] = mvParameters[1] * Point[1] / Point[2] + mvParameters[3];

    return res;

}

cv::Mat Camera_model()
{
    cv::Mat picture_res;
    Eigen::Matrix2d res;

    res = Pinhole_3Dto2D()

    return picture_res;
}
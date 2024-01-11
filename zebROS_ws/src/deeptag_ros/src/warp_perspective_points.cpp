#include "deeptag_ros/warp_perspective_points.h"

std::vector<cv::Point2d> warpPerspectivePts(const cv::Mat &H, const std::vector<cv::Point2d> &points, const double image_scale)
{
    // H is CV64FC1, so these have to match and be doubles rather than floats
    cv::Mat pts(points.size(), 2, CV_64FC1);
    cv::Mat ptsHom(points.size(), 3, CV_64FC1);
    for (size_t i = 0; i < points.size(); i++)
    {
        pts.at<double>(i, 0) = points[i].x;
        pts.at<double>(i, 1) = points[i].y;
        ptsHom.at<double>(i, 0) = points[i].x;
        ptsHom.at<double>(i, 1) = points[i].y;
        ptsHom.at<double>(i, 2) = 1.0f;
    }
    cv::Mat ptsWarp = (H * ptsHom.t()).t();
#if 0
    std::cout << "H = " << H << std::endl;
    std::cout << "pts = " << pts << std::endl;
    std::cout << "ptsHom = " << ptsHom << std::endl;
    std::cout << "ptsHom.t() = " << ptsHom.t() << std::endl;
    std::cout << "ptsWarp = " << std::endl;
    std::cout << ptsWarp << std::endl;
#endif

    ptsWarp(cv::Rect(0, 0, 2, ptsWarp.rows)).copyTo(pts);
    for (int i = 0; i < pts.rows; i++)
    {
        const auto divisor = std::max(1.e-20, ptsWarp.at<double>(i, 2));
        pts.at<double>(i, 0) /= divisor;
        pts.at<double>(i, 1) /= divisor;
    }
    std::vector<cv::Point2d> ret;
    for (int i = 0; i < pts.rows; i++)
    {
        ret.emplace_back(cv::Point2d{pts.at<double>(i, 0) / image_scale, pts.at<double>(i, 1) / image_scale});
        //std::cout << "\t" << ret.back().x << " " << ret.back().y << std::endl;
    }
    return ret;
}
#ifndef POINTS_AND_IDS_INC__
#define POINTS_AND_IDS_INC__
#include "opencv2/core.hpp"

struct PointsAndIDs
{
    PointsAndIDs()
        : m_point{0., 0.}
        , m_id{-1}
        , m_score{0.}
    {
    }
    PointsAndIDs(const double x, const double y, const int id, const double score = 0)
        : m_point{cv::Point2d{x, y}}, m_id{id}, m_score{score}
    {
    }
    cv::Point2d m_point;
    int m_id; 
    double m_score;

    friend std::ostream& operator<<(std::ostream &os, const PointsAndIDs &pid)
    {
        os << pid.m_point.x << ", " << pid.m_point.y << " id = " << pid.m_id << " score = " << pid.m_score;
        return os;
    }
};
#endif

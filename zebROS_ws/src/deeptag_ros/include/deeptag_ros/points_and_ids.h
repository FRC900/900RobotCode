#ifndef POINTS_AND_IDS_INC__
#define POINTS_AND_IDS_INC__
#include "opencv2/core.hpp"

template <size_t GRID_SIZE>
struct PointsAndIDs
{
    PointsAndIDs()
    {
        std::ranges::fill(m_id, -1);
        std::ranges::fill(m_score, -1);
    }

    std::array<cv::Point2d, GRID_SIZE * GRID_SIZE> m_point{};
    std::array<int, GRID_SIZE * GRID_SIZE> m_id{};
    std::array<double, GRID_SIZE * GRID_SIZE> m_score{};

    size_t size(void) const { return m_point.size(); } // all arrays are the same size

    friend std::ostream& operator<<(std::ostream &os, const PointsAndIDs &pid)
    {
        for (size_t ii = 0; ii < pid.m_point.size(); ii++)
        {
            os << pid.m_point[ii].x << ", " << pid.m_point[ii].y << " id = " << pid.m_id[ii] << " score = " << pid.m_score[ii] << std::endl;
        }
        return os;
    }
};
#endif

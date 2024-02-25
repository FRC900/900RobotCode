#include <iostream>
#include <numeric>
#include "opencv2/calib3d.hpp"
#include "deeptag_ros/distorted_h_transform.h"
#include "deeptag_ros/points_and_ids.h"
#include "deeptag_ros/stage2_keypoint_group.h"
#include "deeptag_ros/unit_tag_template.h"
#include "deeptag_ros/warp_perspective_points.h"
// #define DEBUG
#include "deeptag_ros/debug.h"

static constexpr size_t maxWarpTry = 3;
template <size_t FINE_GRID_SIZE>
std::array<PointsAndIDs, FINE_GRID_SIZE * FINE_GRID_SIZE> iterativeMatchAndWarp(const tcb::span<const Stage2KeypointGroup> &unorderedPoints,
                                                                                const std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> &unitPoints,
                                                                                const std::array<cv::Point2d, 4> &unitCorners,
                                                                                const std::array<cv::Point2d, 4> &cptsInCrop, // ordered corners in crop
                                                                                const std::array<double, 9> &cameraMatrix,
                                                                                const std::array<double, 6> &distCoeffs,
                                                                                const cv::Mat &H,
                                                                                const std::vector<cv::Mat> &HListForCtpsInCrop,
                                                                                const size_t maxWarpTry);

template <size_t FINE_GRID_SIZE>
static void checkMatchRatio(double &matchRatio,
                            int &count,
                            int &totalCount,
                            const std::array<PointsAndIDs, FINE_GRID_SIZE * FINE_GRID_SIZE> &orderedPoints,
                            const int unorderedPointsNum);

template <size_t GRID_SIZE, class UNIT_TAG_CLASS, bool IS_NEED_MAIN_IDX, size_t STEP_ELEM_NUM, size_t KPT_START_IDX>
UnitTagTemplate<GRID_SIZE, UNIT_TAG_CLASS, IS_NEED_MAIN_IDX, STEP_ELEM_NUM, KPT_START_IDX>::UnitTagTemplate()
    : m_unitTags{}
{
}

template <size_t GRID_SIZE, class UNIT_TAG_CLASS, bool IS_NEED_MAIN_IDX, size_t STEP_ELEM_NUM, size_t KPT_START_IDX>
void UnitTagTemplate<GRID_SIZE, UNIT_TAG_CLASS, IS_NEED_MAIN_IDX, STEP_ELEM_NUM, KPT_START_IDX>::matchFineGrid(double &maxMatchRatio,
                                                                                                               std::array<PointsAndIDs, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &bestOrderedPoints,
                                                                                                               const tcb::span<const Stage2KeypointGroup> &unorderedPoints,
                                                                                                               const cv::Mat &H,
                                                                                                               const tcb::span<const float2> &stage2PredCorners,
                                                                                                               const cv::Mat &cameraMatrix,
                                                                                                               const cv::Mat &distCoeffs) const
{
    // Make sure there are actually predictions to work with.
    if (stage2PredCorners.size() < 4)
    {
        return;
    }
    if (unorderedPoints.size() == 0)
    {
        return;
    }
    std::vector<cv::Mat> H_list;
    H_list.emplace_back(cv::Mat::eye(3, 3, H.type()));

    const std::array<cv::Point2d, 4> orderedCorners = {cv::Point2d{stage2PredCorners[0].x, stage2PredCorners[0].y},
                                                       cv::Point2d{stage2PredCorners[1].x, stage2PredCorners[1].y},
                                                       cv::Point2d{stage2PredCorners[3].x, stage2PredCorners[3].y},
                                                       cv::Point2d{stage2PredCorners[2].x, stage2PredCorners[2].y}};
    const auto unitCorners = m_unitTags.getOrderedCorners();
    const auto unitPoints = m_unitTags.getFineGridPoints(0, true, STEP_ELEM_NUM);
    printPoints("unitCorners", unitCorners);
    printPoints("orderedCorners", orderedCorners);
    maxMatchRatio = 0;
    for (size_t i = 0; i < bestOrderedPoints.size(); i++)
    {
        bestOrderedPoints[i] = PointsAndIDs{unitPoints[i].x, unitPoints[i].y, -1};
    }

    constexpr auto unitPointsSize = (GRID_SIZE + 2) * (GRID_SIZE + 2);
    const double possibleMatchRatio = static_cast<double>(std::min(unorderedPoints.size(), unitPointsSize)) /
                                      static_cast<double>(std::max(unorderedPoints.size(), unitPointsSize));
    // std::cout << "possibleMatchRatio = " << possibleMatchRatio << std::endl;
    if (maxMatchRatio > possibleMatchRatio)
    {
        return;
    }
    auto orderedPoints = iterativeMatchAndWarp<GRID_SIZE + 2>(unorderedPoints,
                                                              unitPoints,
                                                              unitCorners,
                                                              orderedCorners,
                                                              cameraMatrix,
                                                              distCoeffs,
                                                              H,
                                                              H_list,
                                                              maxWarpTry);
    double matchRatio;
    int count;
    int totalCount;
    checkMatchRatio<GRID_SIZE + 2>(matchRatio, count, totalCount, orderedPoints, static_cast<int>(unorderedPoints.size()));
    // std::cout << "matchRatio = " << matchRatio << std::endl;
    // std::cout << "count = " << count << std::endl;
    // std::cout << "totalCount = " << totalCount << std::endl;
    if (matchRatio > maxMatchRatio)
    {
        maxMatchRatio = matchRatio;
        bestOrderedPoints = orderedPoints;
    }
}

template <size_t GRID_SIZE, class UNIT_TAG_CLASS, bool IS_NEED_MAIN_IDX, size_t STEP_ELEM_NUM, size_t KPT_START_IDX>
std::array<cv::Point2d, 4> UnitTagTemplate<GRID_SIZE, UNIT_TAG_CLASS, IS_NEED_MAIN_IDX, STEP_ELEM_NUM, KPT_START_IDX>::updateCornersInImage(const std::array<PointsAndIDs, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &orderedPointsAndIds,
                                                                                                                                            const cv::Mat &HCrop,
                                                                                                                                            const cv::Mat &cameraMatrix,
                                                                                                                                            const cv::Mat &distCoeffs) const
{
    const auto unitPoints = m_unitTags.getFineGridPoints(0, true, STEP_ELEM_NUM);
    std::array<cv::Point2d, (GRID_SIZE + 2) * (GRID_SIZE + 2)> orderedPoints;
    for (size_t i = 0; i < orderedPointsAndIds.size(); i++)
    {
        orderedPoints[i] = orderedPointsAndIds[i].m_point;
    }
    const auto unitCorners = m_unitTags.getOrderedCorners();
    const auto cornersInCropUpdated = controlpointsToKeypointsInCropWithH(unitPoints, orderedPoints, unitCorners, cameraMatrix, distCoeffs, HCrop);

    return warpPerspectivePts(HCrop.inv(), cornersInCropUpdated);
}

// Try to match detected points in kptsWithIds to the candidate spots
// calculated using the given FINE_GRID_SIZE.
// In this context, the FINE_GRID_SIZE is the tag grid size + 2, adding
// a top/bottom or left/right border in each dimesion
// The status of the match is reported in matchFlagsCand and matchIds. 
// Additionally, attempt to find a H transform between the matched detected points and
// their corresponding ground truth locations that minimizes the error between the two
template <size_t FINE_GRID_SIZE>
static void matchAndWarp(cv::Mat &HNew,
                         std::array<bool, FINE_GRID_SIZE * FINE_GRID_SIZE> &matchFlagsCand,
                         std::array<int, FINE_GRID_SIZE * FINE_GRID_SIZE> &matchIds,
                         double &meanDist,
                         const std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> &kptsCand,
                         const tcb::span<const Stage2KeypointGroup> &kptsWithIds,
                         const double maxMatchDist,
                         cv::Mat H)
{
#ifdef DEBUG
    std::cout << "================================================================================" << std::endl;
    std::cout << "matchAndWarp" << std::endl;
    printPoints("kptsCand", kptsCand);

    std::cout << "kptsWitIds = ";
    for (const auto &s : kptsWithIds)
    {
        std::cout << "[ " << s.m_keypoint.x << " " << s.m_keypoint.y << " " << s.m_label << "] " << std::endl;;
    }
    std::cout << "maxMatchDist = " << maxMatchDist << std::endl;
    std::cout << "H = " << H << std::endl;
#endif

    if (H.empty())
    {
        H = cv::Mat::eye(3, 3, CV_64FC1);
    }

    std::ranges::fill(matchFlagsCand, false);
    std::vector<bool> matchFlags(kptsWithIds.size(), false);
    std::ranges::fill(matchIds, -1);

    const auto kptsCandWarp = warpPerspectivePts(H, kptsCand);
    printPoints("kptsCandWarp", kptsCandWarp);
    std::vector<double> distList;

    for (size_t ii = 0; ii < kptsCandWarp.size(); ii++)
    {
        // If this ground truth candidate location already has a detected point assigned
        // to it, don't do anything else with that candidate location
        if (matchFlagsCand[ii])
        {
            continue;
        }
        // Otherwise try to find the closest available detected point and assign it
        // as a match to the candiate ground truth
        for (size_t jj = 0; jj < kptsWithIds.size(); jj++)
        {
            if (matchFlags[jj])
            {
                continue;
            }

            const double dist = hypot(kptsWithIds[jj].m_keypoint.x - kptsCandWarp[ii].x, kptsWithIds[jj].m_keypoint.y - kptsCandWarp[ii].y);
#if 0
            std::cout << "ii = " << ii;
            std::cout << " jj = " << jj;
            std::cout << " kpt_tem = " << kptsCandWarp[ii].x << " " << kptsCandWarp[ii].y;
            std::cout << " kpt = " << kptsWithIds[ii].m_keypoint.x << " " << kptsWithIds[ii].m_keypoint.y;
            std::cout << " dist = " << dist << std::endl;
#endif
            // Have a threshold distance that's the average dist between candidate GT points
            // to prevent a match with a detected point obviously too far away
            if (dist < maxMatchDist)
            {
                matchFlagsCand[ii] = true;
                matchFlags[jj] = true;
                matchIds[ii] = jj;
                distList.push_back(dist);
                break;
            }
        }
    }

    // Create vectors of matches between candidate GT points and detected points
    // to use for computing a tranform to warp the detected points to ground truth
    std::vector<cv::Point2d> matchedKpts;
    std::vector<cv::Point2d> matchedKptsCand;
    for (size_t ii = 0; ii < matchIds.size(); ii++)
    {
        if (matchFlagsCand[ii])
        {
            matchedKpts.push_back(cv::Point2d{kptsWithIds[matchIds[ii]].m_keypoint.x, kptsWithIds[matchIds[ii]].m_keypoint.y});
            matchedKptsCand.push_back(cv::Point2d{kptsCandWarp[ii].x, kptsCandWarp[ii].y});
        }
    }
    printPoints("matchedKpts", matchedKpts);
    printPoints("matchedKptsCand", matchedKptsCand);

    if ((matchedKpts.size() < 4) || (matchedKptsCand.size() < 4))
    {
        HNew = cv::Mat();
        meanDist = maxMatchDist;
    }
    else
    {
        const auto HEst = cv::findHomography(matchedKptsCand, matchedKpts);
        HNew = HEst * H;
#ifdef DEBUG
        std::cout << "HEst =  " << HEst << std::endl;
        std::cout << "HNew =  " << HNew << std::endl;
#endif
    }
    if (distList.size())
    {
        meanDist = *std::ranges::max_element(distList);
    }
    else
    {
        meanDist = 1e20;
    }
}

template <size_t FINE_GRID_SIZE>
std::array<PointsAndIDs, FINE_GRID_SIZE * FINE_GRID_SIZE> iterativeMatchAndWarp(const tcb::span<const Stage2KeypointGroup> &stage2KeypointGroups,              // keypoints in crop
                                                                                const std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> &orderedKptsGt, // unit points
                                                                                const std::array<cv::Point2d, 4> &cptsGt,                                      // unit corners
                                                                                const std::array<cv::Point2d, 4> &cptsInCrop,                                  // ordered corners in crop
                                                                                const cv::Mat &cameraMatrix,
                                                                                const cv::Mat &distCoeffs,
                                                                                const cv::Mat &H,
                                                                                const std::vector<cv::Mat> &HListForCtpsInCrop,
                                                                                const size_t maxWarpTry)
{
#ifdef DEBUG
    std::cout << "iterativeMatchAndWarp" << std::endl;
    std::cout << "kptsInCrop = " << std::endl;
    for (const auto &s : stage2KeypointGroups)
    {
        std::cout << "[ " << s.m_keypoint.x << " " << s.m_keypoint.y << " " << s.m_label << "] " << std::endl;
    }
    printPoints("orderedKptsGt = ", orderedKptsGt);
    printPoints("cptsGt = ", cptsGt);
    printPoints("cptsInCrop = ", cptsInCrop);
    std::cout << "H = " << H << std::endl;
    std::cout << "cameraMatrix = " << cameraMatrix << std::endl;
    std::cout << "distCoeffs = " << distCoeffs << std::endl;

    std::cout << std::endl;
#endif
    size_t maxMatchedNum = 0;
    size_t matchedNum = 0;
    cv::Mat HCurr;
    cv::Mat HNew;

    // Result from this iteration of match and warp
    std::array<bool, FINE_GRID_SIZE * FINE_GRID_SIZE> matchFlagsCand;
    std::array<int, FINE_GRID_SIZE * FINE_GRID_SIZE> matchIds;
    double maxMatchDist = 0;
    double meanDist = 1e20;
    // Current best results so far
    std::array<bool, FINE_GRID_SIZE * FINE_GRID_SIZE> matchFlagsCandBest;
    std::array<int, FINE_GRID_SIZE * FINE_GRID_SIZE> matchIdsBest;

    for (const auto &HInit : HListForCtpsInCrop)
    {
        const auto cptsInCropUpdated = warpPerspectivePts(HInit, cptsInCrop);
        const auto kptsCand = controlpointsToKeypointsInCropWithH(cptsGt, cptsInCropUpdated, orderedKptsGt, cameraMatrix, distCoeffs, H);

        // Initialize max match dist to 1/2 distance between ground truth points
        maxMatchDist = hypot(kptsCand[0].x - kptsCand[1].x, kptsCand[0].y - kptsCand[1].y) / 2.;
#ifdef DEBUG
        printPoints("cptsInCropUpdated = ", cptsInCropUpdated);
        printPoints("ktpsCand", kptsCand);
        std::cout << "matchMaxDist = " << maxMatchDist << std::endl;
#endif

        // Create a translation between centroid of candidate GT and detected points
        cv::Mat HTranslate = cv::Mat::eye(3, 3, CV_64FC1);

        double2 kptsCandMean{0,0};
        for (const auto &p : kptsCand)
        {
            kptsCandMean.x += p.x;
            kptsCandMean.y += p.y;
        }
        kptsCandMean.x /= kptsCand.size();
        kptsCandMean.y /= kptsCand.size();

        double2 kptsMean{0,0};
        for (const auto &p : stage2KeypointGroups)
        {
            kptsMean.x += p.m_keypoint.x;
            kptsMean.y += p.m_keypoint.y;
        }
        kptsMean.x /= stage2KeypointGroups.size();
        kptsMean.y /= stage2KeypointGroups.size();

        HTranslate.at<double>(0,2) = kptsMean.x - kptsCandMean.x;
        HTranslate.at<double>(1,2) = kptsMean.y - kptsCandMean.y;

        matchAndWarp<FINE_GRID_SIZE>(HNew, matchFlagsCand, matchIds, meanDist, kptsCand, stage2KeypointGroups, maxMatchDist, HTranslate);

        matchedNum = std::accumulate(matchFlagsCand.cbegin(), matchFlagsCand.cend(), static_cast<size_t>(0));
#ifdef DEBUG
        std::cout << "kptsMean = " << kptsMean.x << " " << kptsMean.y << std::endl;
        std::cout << "kptsCandMean = " << kptsCandMean.x << " " << kptsCandMean.y << std::endl;
        std::cout << "HTranslate = " << HTranslate << std::endl;
        std::cout << "matchAndWarp : HNew = " << HNew << std::endl;
        std::cout << "matchFlagsCand = ";
        for (const auto m : matchFlagsCand)
        {
            std::cout << int(m) << " ";
        }
        std::cout << std::endl;
        std::cout << "matchIds = ";
        for (const auto m : matchIds)
        {
            std::cout << int(m) << " ";
        }
        std::cout << std::endl;
        std::cout << "meanDist = " << meanDist << std::endl;
        std::cout << "matchedNum = " << matchedNum << std::endl;
#endif
        if (!HNew.empty() && (matchedNum > maxMatchedNum))
        {
            maxMatchDist = std::min(meanDist * 1.01, maxMatchDist);
            HCurr = HNew * HInit;
#ifdef DEBUG
            std::cout << "  HCurr = " << HCurr << std::endl;
            std::cout << "  HNew = " << HNew << std::endl;
            std::cout << "  HInit = " << HInit << std::endl;
#endif

            matchFlagsCandBest = matchFlagsCand;
            matchIdsBest = matchIds;
            maxMatchedNum = matchedNum;
        }
        if (matchedNum > (orderedKptsGt.size() - 5))
        {
            break;
        }
    }

    if (!HCurr.empty())
    {
        for (size_t i = 0; i < maxWarpTry; i++)
        {
            const auto cptsInCropUpdated = warpPerspectivePts(HCurr, cptsInCrop);
            const auto kptsCand = controlpointsToKeypointsInCropWithH(cptsGt, cptsInCropUpdated, orderedKptsGt, cameraMatrix, distCoeffs, H);
            // TODO : this can't happen, given kptsCand is a std::array?
            if (kptsCand.size() == 0)
            {
                break;
            }
            double newMeanDist;
            matchAndWarp<FINE_GRID_SIZE>(HNew, matchFlagsCand, matchIds, newMeanDist, kptsCand, stage2KeypointGroups, maxMatchDist, cv::Mat::eye(3, 3, CV_64FC1));
            //std::cout << "  HNew = " << HNew << std::endl;
            const auto newMatchedNum = std::accumulate(matchFlagsCand.cbegin(), matchFlagsCand.cend(), static_cast<size_t>(0));
#ifdef DEBUG
            std::cout << "newMatchedNum = " << newMatchedNum << " matchedNum = " << matchedNum << std::endl;
            std::cout << "meanDist = " << meanDist << " newMeanDist = " << newMeanDist << std::endl;
            std::cout << "HNew = " << HNew << std::endl;
#endif
            if (((newMatchedNum < matchedNum) && (i > 0)) || HNew.empty())
            {
#ifdef DEBUG
                std::cout << "condition 1 exit" << std::endl;
#endif
                break;
            }
            else if ((newMatchedNum == matchedNum) && (meanDist <= newMeanDist))
            {
#ifdef DEBUG
                std::cout << "condition 2 exit" << std::endl;
#endif
                break;
            }
            HCurr = HNew * HCurr;
            matchFlagsCandBest = matchFlagsCand;
            matchIdsBest = matchIds;
            meanDist = newMeanDist;
            matchedNum = newMatchedNum;

#if 0
            // If there are nearly all matches and the number of matches hasn't changed
            // this time through, quit
            if ((newMatchedNum > (matchFlagsCand.size() - 3)) && (newMatchedNum == matchedNum))
            {
                std::cout << "condition 2 exit" << std::endl;
                break;
            }
            if (newMatchedNum == orderedKptsGt.size())
            {
                std::cout << "condition 3 exit" << std::endl;
                break;
            }
#endif
        }
    }
    std::array<PointsAndIDs, FINE_GRID_SIZE * FINE_GRID_SIZE> orderedKptsWithIds;
    //std::cout << "HCurr = " << HCurr << std::endl;
    if (HCurr.empty())
    {
        for (size_t ii = 0; ii < orderedKptsWithIds.size(); ii++)
        {
            orderedKptsWithIds[ii] = PointsAndIDs{orderedKptsGt[ii].x, orderedKptsGt[ii].y, -1};
        }
    }
    else
    {
        // warp template with latest H
        // TODO : only if any unmatched points?
        const auto cptsInCropUpdated = warpPerspectivePts(HCurr, cptsInCrop);
        const auto orderedKptCandidatesWarp = controlpointsToKeypointsInCropWithH(cptsGt, cptsInCropUpdated, orderedKptsGt, cameraMatrix, distCoeffs, H);

        for (size_t ii = 0; ii < matchFlagsCandBest.size(); ii++)
        {
#ifdef DEBUG
            std::cout << "ii = " << ii << " matchFlagsCandBest[ii] = " << matchFlagsCandBest[ii] << " matchIdsBest[ii] = " << matchIdsBest[ii] << std::endl;
#endif
            if (matchFlagsCandBest[ii])
            {
                const auto &kp = stage2KeypointGroups[matchIdsBest[ii]];
                orderedKptsWithIds[ii] = PointsAndIDs{kp.m_keypoint.x,
                                                      kp.m_keypoint.y,
                                                      kp.m_label,
                                                      kp.m_score};
            }
            else
            {
                orderedKptsWithIds[ii] = PointsAndIDs{orderedKptCandidatesWarp[ii].x,
                                                      orderedKptCandidatesWarp[ii].y,
                                                      -1};
            }
        }
    }
#ifdef DEBUG
    std::cout << "orderedKptsWithIds" << std::endl;
    for (const auto &o : orderedKptsWithIds)
    {
        std::cout << o.m_point.x << " " << o.m_point.y << " " << o.m_id << std::endl;
    }
#endif


    return orderedKptsWithIds;
}

template <size_t FINE_GRID_SIZE>
static void checkMatchRatio(double &matchRatio,
                            int &count,
                            int &totalCount,
                            const std::array<PointsAndIDs, FINE_GRID_SIZE * FINE_GRID_SIZE> &orderedPoints,
                            const int unorderedPointsNum)
{
    count = 0;
    for (const auto &op: orderedPoints)
    {
        if (op.m_id >= 0)
        {
            count += 1;
        }
    }
    totalCount = std::max(static_cast<int>(orderedPoints.size()), count);
    matchRatio = static_cast<double>(count) / totalCount;
#ifdef DEBUG
    std::cout << "checkMatchRatio : count = " << count << " unorderedPointNum = " << unorderedPointsNum << " totalCount = " << totalCount << " matchRatio = " << matchRatio << std::endl;
#endif
}

#include "deeptag_ros/unit_arucotag.h"
template <size_t GRID_SIZE>
UnitTagTemplateArucotag<GRID_SIZE>::UnitTagTemplateArucotag(void)
    : UnitTagTemplate<GRID_SIZE, UnitArucoTag<GRID_SIZE>, false, 1, 0>{}
{
}
template class UnitTagTemplate<4, UnitArucoTag<4>, false, 1, 0>;
template class UnitTagTemplate<5, UnitArucoTag<5>, false, 1, 0>;
template class UnitTagTemplate<6, UnitArucoTag<6>, false, 1, 0>;
template class UnitTagTemplateArucotag<4>;
template class UnitTagTemplateArucotag<5>;
template class UnitTagTemplateArucotag<6>;
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <opencv2/calib3d.hpp>        // for findHomography
#include <opencv2/core/mat.hpp>       // for Mat, MatExpr
#include <opencv2/core/mat.inl.hpp>   // for _InputArray::_InputArray, _Inpu...
#include <opencv2/core/traits.hpp>    // for Depth<>::value
#include "deeptag_ros/roi_generator.h"
#include "deeptag_ros/ssd_box.h"                   // for SSDBoxCornerForm
#include "deeptag_ros/stage1_grid_group.h"         // for Stage1GridGroup
#include "deeptag_ros/stage1_ssd_group.h"          // for Stage1SSDGroup
#include "deeptag_ros/tag_detect_info.h"
#include "deeptag_ros/warp_perspective_points.h"

#undef DEBUG
#include "deeptag_ros/debug.h"

static double2 operator+(const double2 &op1, const double2 &op2)
{
    return double2{op1.x + op2.x, op1.y + op2.y};
}
static double2 operator-(const float2 &op1, const float2 &op2)
{
    return double2{op1.x - op2.x, op1.y - op2.y};
}
void TagDetectInfo::print(void) const
{
    std::cout << "tagId = " << m_tagId << std::endl;
    std::cout << "\tcenterPos = " << m_centerPos.x << " " << m_centerPos.y << std::endl;
    std::cout << "\tcenterScore = " << m_centerScore << std::endl;
    std::cout << "\tcenterLinkScore = " << m_centerLinkScore << std::endl;
    for (const auto &c : m_orderedCorners)
    {
        std::cout << "\torderedCorner = " << c.x << " " << c.y << std::endl;
    }
    for (const auto &c : m_cornerScores)
    {
        std::cout << "\tcornerScore = " << c << std::endl;
    }
    for (const auto &a : m_anchors)
    {
        std::cout << "\tanchor = " << a.x << " " << a.y << std::endl;
    }
    std::cout << "\tmainIdx = " << m_mainIndex << std::endl;
}

template <uint32_t BATCH_SIZE>
static void assignCornersToOneCenter(std::vector<cv::Point2d> &cornersSelected,
                                     std::vector<int> &cornerIdsSelected,
                                     std::vector<double> &cosDistSelected,
                                     std::vector<double> &relDistSelected,
                                     const float2 &centerPos,
                                     const SSDBoxCornerForm &bbox,
                                     const tcb::span<const Stage1GridGroup<BATCH_SIZE>> &stage1GridGroup, // holds corners, corner ids, and corner directions
                                     const double cosThresh = 0.6,
                                     const double relDistThresh = 1.5)
{
    cornersSelected.clear();
    cornerIdsSelected.clear();
    cosDistSelected.clear();
    relDistSelected.clear();

    if (stage1GridGroup.empty())
    {
#ifdef DEBUG
        std::cout << "Stage1GridGroup is empty" << std::endl;
#endif
        return;
    }
    std::vector<double> cosDist;
    double maxCosDist = 1e-20;
    auto calcCosDist = [&](const Stage1GridGroup<BATCH_SIZE> &s1gg)
    {
        const double2 vecsRel = s1gg.m_corner - centerPos;
        const double vecRelLen = hypot(vecsRel.x, vecsRel.y);
        const double vecLen = hypot(s1gg.m_cornerDirection.x, s1gg.m_cornerDirection.y);
        cosDist.push_back((vecsRel.x * s1gg.m_cornerDirection.x + vecsRel.y * s1gg.m_cornerDirection.y) / std::max(vecRelLen * vecLen, 1.e-20));
        maxCosDist = std::max(maxCosDist, cosDist.back());
    };

    std::ranges::for_each(stage1GridGroup, calcCosDist);
    const std::array<double2, 4> fourAxis = {double2{bbox.m_tl.x - centerPos.x, 0.f},
                                             double2{0, bbox.m_tl.y - centerPos.y},
                                             double2{bbox.m_br.x - centerPos.x, 0.f},
                                             double2{0, bbox.m_br.y - centerPos.y}};

    const std::array<double, 4> fourAxisLen = {fabs(bbox.m_tl.x - centerPos.x),
                                               fabs(bbox.m_tl.y - centerPos.y),
                                               fabs(bbox.m_br.x - centerPos.x),
                                               fabs(bbox.m_br.y - centerPos.y)};

    std::vector<double> relDist(stage1GridGroup.size(), 0.);

    for (size_t i = 0; i < fourAxis.size(); i++) // for each axis
    {
        for (size_t c = 0; c < stage1GridGroup.size(); c++) // for each corner
        {
            double relDistAlongAxis = fourAxis[i].x * (stage1GridGroup[c].m_corner.x - centerPos.x) + fourAxis[i].y * (stage1GridGroup[c].m_corner.y - centerPos.y);
            relDistAlongAxis /= (fourAxisLen[i] * fourAxisLen[i]);
            relDist[c] = std::max(relDist[c], relDistAlongAxis);
        }
    }

    // TODO - should be best 4, possibly fewer if less make the thresholding cut?
    for (size_t i = 0; i < stage1GridGroup.size(); i++) // for each corner
    {
        std::cout << "i = " << i << " coord = " << stage1GridGroup[i].m_corner.x << " " << stage1GridGroup[i].m_corner.y << " cosDist[] = " << cosDist[i] << " relDist[] = " << relDist[i] << std::endl;
        if ((fabs(cosDist[i]) > std::max(cosThresh, maxCosDist * 0.85)) && (relDist[i] < relDistThresh))
        {
            cornersSelected.emplace_back(cv::Point2d{stage1GridGroup[i].m_corner.x, stage1GridGroup[i].m_corner.y});
            cornerIdsSelected.push_back(i);
            cosDistSelected.push_back(cosDist[i]);
            relDistSelected.push_back(relDist[i]);
        }
    }
}

template <class T, class W>
void xyToPolar(T &points, const W &center)
{
    for (auto &p : points)
    {
        const cv::Point2d pointCentered{p.x - center.x, p.y - center.y};
        p.x = hypot(pointCentered.x, pointCentered.y);
        p.y = atan2(pointCentered.y, pointCentered.x) * 180.f / M_PI;
    }
}

void tiltPolarCorners(std::vector<cv::Point2d> &polarCorners)
{
    std::array<double, 3> rotTries = {0., -10., 10.f};
    std::vector<double> h;

    for (const auto &p : polarCorners)
    {
        h.push_back(p.y + 180.f + 45.f); 
    }

#ifdef DEBUG
    printPoints("polarCorners", polarCorners);
    std::cout << "h = ";
    std::ranges::copy(h, std::ostream_iterator<double>(std::cout, " "));
    std::cout << std::endl;
#endif
    std::vector<double> tiltDegreesCandidate;
    std::vector<double> hRot;
    for (const auto r : rotTries)
    {
        hRot.clear();
        for (const auto &thisH : h)
        {
            hRot.push_back(round((thisH + r) / 90.) * 90. - thisH);
            // need argsort
        }
        std::vector<int> idx(hRot.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::ranges::sort(idx,
                          [&hRot](const size_t &a, const size_t &b)
                          { return fabs(hRot[a]) < fabs(hRot[b]); });
#ifdef DEBUG
        std::cout << "hRot = ";
        std::copy(hRot.cbegin(), hRot.cend(), std::ostream_iterator<double>(std::cout, " "));
        std::cout << std::endl;
        std::cout << "idx = ";
        std::copy(idx.cbegin(), idx.cend(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
#endif
        if (idx.size() >= 4)
        {
            // Push mean
            tiltDegreesCandidate.push_back(std::reduce(hRot.begin(), hRot.end()) / idx.size());
        }
        else
        {
            tiltDegreesCandidate.push_back(hRot[0]);
        }
    }
    size_t maxIdx = 0;
    double maxTiltDegrees = 0;
    for (size_t i = 0; i < tiltDegreesCandidate.size(); i++)
    {
        const auto tiltDegrees = fabs(tiltDegreesCandidate[i]);
#ifdef DEBUG
        std::cout << "tiltDegrees = " << tiltDegrees << std::endl;
#endif
        if ((tiltDegrees < 45.f) && (tiltDegrees > maxTiltDegrees))
        {
            maxIdx = i;
            maxTiltDegrees = tiltDegrees;
        }
    }
    for (auto &p : polarCorners)
    {
        p.y += tiltDegreesCandidate[maxIdx];
    }
}

template <class T, class W>
void polarToXY(T &points, const W &center)
{
    for (auto &p : points)
    {
        p.y *= M_PI / 180.;
        const auto cx = cos(p.y) * p.x;
        const auto sx = sin(p.y) * p.x;
        p.x  = cx + center.x;
        p.y  = sx + center.y;
    }
}

void sortCornersInATag(std::array<int, 4> &bestIds,
                       std::array<cv::Point2d, 4> &guessPoints,
                       const float2 &centerPos,
                       const tcb::span<const float2> &anchors, // Check this
                       const std::vector<cv::Point2d> &cornersSelected,
                       double minUnitDistInTag = 0.8)
{
#ifdef DEBUG
    std::cout << __FUNCTION__ << std::endl;
#endif
    // convert to unit space
    std::array<cv::Point2d, 5> unitRefPoints;
    const cv::Point2d unitRefCenter{0.5, 0.5};
    unitRefPoints[0] = cv::Point2d{0., 0.};
    unitRefPoints[1] = cv::Point2d{1., 0.};
    unitRefPoints[2] = cv::Point2d{1., 1.};
    unitRefPoints[3] = cv::Point2d{0., 1.};
    unitRefPoints[4] = unitRefCenter;

    std::array<cv::Point2d, 5> inputPoints;
    inputPoints[0] = cv::Point2d{anchors[0].x, anchors[0].y};
    inputPoints[1] = cv::Point2d{anchors[1].x, anchors[1].y};
    inputPoints[2] = cv::Point2d{anchors[2].x, anchors[2].y};
    inputPoints[3] = cv::Point2d{anchors[3].x, anchors[3].y};
    inputPoints[4] = cv::Point2d{centerPos.x, centerPos.y};
    printPoints("unitRefPoints", unitRefPoints);
    printPoints("inputPoints", inputPoints);

    cv::Mat H = cv::findHomography(inputPoints, unitRefPoints);
#ifdef DEBUG
    std::cout << "H = " << H << std::endl;
#endif
    std::vector<cv::Point2d> unitCornersCand = warpPerspectivePts(H, cornersSelected);
    printPoints("inputPoints", inputPoints);
    printPoints("unitRefPoints", unitRefPoints);
    printPoints("unitCornersCand", unitCornersCand);

    // convert to log-polar, tilt, then back to unit space
    xyToPolar(unitCornersCand, unitRefCenter);
    printPoints("polarCornersCand", unitCornersCand);
    const auto polarCornersCand = unitCornersCand;
    tiltPolarCorners(unitCornersCand);
    printPoints("polarCornersCandTilted", unitCornersCand);
    polarToXY(unitCornersCand, unitRefCenter);
    printPoints("unitCornersCandTilted", unitCornersCand);

    // get dist from candidate points to anchors in unit space
    std::vector<bool> matchFlags(cornersSelected.size());
    for (size_t i = 0; i < bestIds.size(); i++)
    {
        double minDist = minUnitDistInTag;
        int bestId = -1;
        const auto &refPt = unitRefPoints[i];
#ifdef DEBUG
        std::cout << " i = " << i << " refPt = " << refPt.x << " " << refPt.y << std::endl;
#endif
        for (size_t j = 0; j < cornersSelected.size(); j++)
        {
#ifdef DEBUG
            std::cout << " matchFlags[" << j << "] = " << matchFlags[j] << std::endl;
#endif
            if (matchFlags[j])
            {
                continue;
            }
            const double dist = hypot(refPt.x - unitCornersCand[j].x, refPt.y - unitCornersCand[j].y);
#ifdef DEBUG
            std::cout << "pt = " << unitCornersCand[j].x << " " << unitCornersCand[j].y << std::endl;
            std::cout << "dist = " << dist << std::endl;
#endif
            if (dist < minDist)
            {
#ifdef DEBUG
                std::cout << " New best" << std::endl;
#endif
                bestId = j;
                minDist = dist;
            }
        }
        if (bestId >= 0)
        {
            matchFlags[bestId] = true;
        }
        bestIds[i] = bestId;
#ifdef DEBUG
        std::cout << "bestIds = ";
        for (const auto i : bestIds)
        {
            std::cout << i << " ";
        }
        std::cout << std::endl;
#endif
    }

    // guess points
    printPoints("unitRefPoints", unitRefPoints);
    xyToPolar(unitRefPoints, unitRefCenter); // unit ref points is now polar_ref_points
    printPoints("polarRefPoints", unitRefPoints);
    double2 correctionsMean{0, 0};
    size_t correctionsCount{0};
    for (size_t i = 0; i < bestIds.size(); i++)
    {
        const auto bestId = bestIds[i];
        if (bestId >= 0)
        {
            correctionsMean = correctionsMean + double2{polarCornersCand[bestId].x / unitRefPoints[i].x, polarCornersCand[bestId].y - unitRefPoints[i].y};
            correctionsCount += 1;
        }
    }
    if (correctionsCount == 0)
    {
        for (size_t i = 0; i < guessPoints.size(); i++)
        {
            guessPoints[i] = cv::Point2d{anchors[i].x, anchors[i].y};
        }
    }
    else
    {
        correctionsMean.x /= correctionsCount;
        correctionsMean.y /= correctionsCount;
#ifdef DEBUG
        std::cout << "correctionsMean " << correctionsMean.x << " " << correctionsMean.y << std::endl;
#endif
        for (auto &p : unitRefPoints) // do in-place mean correction
        {
            p.x *= correctionsMean.x;
            p.y += correctionsMean.y;
        }
        printPoints("polarUnitGuessPoints", unitRefPoints);
        polarToXY(unitRefPoints, unitRefCenter);
        // create guessPoints from unitRefPoints
        for (size_t i = 0; i < guessPoints.size(); i++)
        {
            guessPoints[i] = unitRefPoints[i];
        }
        printPoints("unitGuessPoints", unitRefPoints);
        warpPerspectivePts(H.inv(), guessPoints); // then unwarp
        printPoints("guessPoints", guessPoints);
    }
}

void guessOrderedCornersAndScores(std::array<cv::Point2d, 4> &orderedCorners,
                                  double &centerScore,
                                  std::vector<double> &cornerScores,
                                  const std::vector<cv::Point2d> &cornersSelected,
                                  const std::vector<double> &cosDistList,
                                  const std::vector<double> &relPosList,
                                  const std::array<int, 4> &bestIds,
                                  const std::array<cv::Point2d, 4> &guessPoints)
{
    cornerScores.clear();
    for (size_t i = 0; i < bestIds.size(); i++)
    {
        auto bestId = bestIds[i];
        if (bestId >= 0)
        {
            orderedCorners[i] = cv::Point2d{cornersSelected[bestId].x, cornersSelected[bestId].y};
            cornerScores.push_back(fabs(cosDistList[bestId] * (1. / std::max(1., relPosList[bestId]))));
        }
        else
        {
            orderedCorners[i] = guessPoints[i];
            cornerScores.push_back(0);
        }

    }
    centerScore = std::reduce(cornerScores.cbegin(), cornerScores.cend()) / static_cast<double>(cornerScores.size());
}

template <uint32_t BATCH_SIZE>
void centerAndCornersToTags(std::vector<TagDetectInfo> &tagDetectInfo,
                            std::vector<std::vector<cv::Point2d>> &centerToCornerLinks,
                            const tcb::span<const Stage1GridGroup<BATCH_SIZE>> &stage1GridGroup,
                            const tcb::span<const Stage1SSDGroup> &stage1SSDGroup,
                            bool isAllowNoCornerRefine,
                            bool isWithCornerRefine)
{
    tagDetectInfo.clear();
    centerToCornerLinks.clear();

    std::vector<cv::Point2d> cornersSelected; // these are just copies of input corners, no math done on them yet, 
    std::vector<int> cornerIdsSelected;
    std::vector<double> cosDistSelected;
    std::vector<double> relDistSelected;
    std::array<int, 4> bestIds;
    std::array<cv::Point2d, 4> guessPoints;
    std::array<cv::Point2d, 4> orderedCorners;
    std::vector<double> cornerScores;
    int mainIdx = 0;
    double centerScore = 0;
    for (size_t i = 0; i < stage1SSDGroup.size(); i++) // loop over centers, which are in the SSD results
    {
        std::cout << "TagAssignLoop" << std::endl;
        int tagId = stage1SSDGroup[i].m_id;
        assignCornersToOneCenter(cornersSelected,
                                 cornerIdsSelected,
                                 cosDistSelected,
                                 relDistSelected,
                                 stage1SSDGroup[i].m_center,
                                 stage1SSDGroup[i].m_bbox,
                                 stage1GridGroup);
        for (auto &cid : cornerIdsSelected)
        {
            cid = stage1GridGroup[cid].m_id;
        }


        std::cout << "centerPos = " << stage1SSDGroup[i].m_center.x << " " << stage1SSDGroup[i].m_center.y << std::endl;
        printPoints("cornersSelected", cornersSelected);
        std::cout << "cornerIdsSelected = ";
        std::ranges::copy(cornerIdsSelected, std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
        printPoints("anchors", tcb::span<const float2>(&stage1SSDGroup[i].m_anchorsInBox[0], 4));
#ifdef DEBUG
        std::cout << "cosDistSelected : " << std::endl;
        for (const auto &c : cosDistSelected)
        {
            std::cout << "\t" << c << std::endl;
        }
        std::cout << "relDistSelected : " << std::endl;
        for (const auto &r : relDistSelected)
        {
            std::cout << "\t" << r << std::endl;
        }
#endif
        centerToCornerLinks.push_back(cornersSelected);
        bool validFlag = false;
        // TODO - in python code, there's a check which translates to (stage1SSDGroup[i].m_tanchorsInBox.size() > 2)
        // Need to go back and see if that's possible

        if (!cornerIdsSelected.empty() && isWithCornerRefine)
        {
            sortCornersInATag(bestIds,
                              guessPoints,
                              stage1SSDGroup[i].m_center,
                              tcb::span<const float2>(stage1SSDGroup[i].m_anchorsInBox, 4),
                              cornersSelected);
#if 1
            std::cout << "bestIds = ";
            for (const auto id : bestIds)
            {
                std::cout << id << " ";
            }
            std::cout << std::endl;
            std::cout << "guessPoints = " << std::endl;
            for (const auto &p : guessPoints)
            {
                std::cout << "\t" << p.x << " " << p.y << std::endl;
            }
#endif
            guessOrderedCornersAndScores(orderedCorners,
                                         centerScore,
                                         cornerScores,
                                         cornersSelected,
                                         cosDistSelected,
                                         relDistSelected,
                                         bestIds,
                                         guessPoints);
            validFlag = true;

            mainIdx = 0;
            int maxLabel = 0;
            int validBestIDCount = 0;
            for (size_t jj = 0; jj < bestIds.size(); jj++)
            {
                const auto bestId = bestIds[jj];
                if (bestId >= 0)
                {
                    validBestIDCount += 1;
                    if (cornerIdsSelected[bestId] > maxLabel)
                    {
                        maxLabel = cornerIdsSelected[bestId];
                        mainIdx = (jj + 2) % bestIds.size();
                    }
                }
            }
            if (validBestIDCount < 4)
            {
               tagId = -1; 
            }
        }
        else if (!isWithCornerRefine || (isAllowNoCornerRefine && !cornersSelected.size()))
        {
            cornerScores.clear();
            for (size_t j = 0; j < 4; j++)
            {
                const auto &a = stage1SSDGroup[i].m_anchorsInBox[j];
                orderedCorners[j] = cv::Point2d{a.x, a.y};
                cornerScores.push_back(0);
            }
            centerScore = 0;
            mainIdx = 0;
            validFlag = true;
        }
        if (validFlag)
        {
            TagDetectInfo t;

            t.m_centerPos = stage1SSDGroup[i].m_center;
            t.m_tagId = tagId;
            t.m_centerScore = stage1SSDGroup[i].m_score;
            t.m_centerLinkScore = static_cast<float>(centerScore);
            t.m_orderedCorners = orderedCorners;
            t.m_cornerScores = cornerScores;
            for (const auto &a : stage1SSDGroup[i].m_anchorsInBox)
            {
                t.m_anchors.push_back(a);
            }
            t.m_mainIndex = mainIdx;

            tagDetectInfo.emplace_back(t);
        }
    }
}
template void centerAndCornersToTags<1>(std::vector<TagDetectInfo> &tagDetectInfo,
                                        std::vector<std::vector<cv::Point2d>> &centerToCornerLinks,
                                        const tcb::span<const Stage1GridGroup<1>> &stage1GridGroup,
                                        const tcb::span<const Stage1SSDGroup> &stage1SSDGroup,
                                        bool isAllowNoCornerRefine,
                                        bool isWithCornerRefine);
template void centerAndCornersToTags<4>(std::vector<TagDetectInfo> &tagDetectInfo,
                                        std::vector<std::vector<cv::Point2d>> &centerToCornerLinks,
                                        const tcb::span<const Stage1GridGroup<4>> &stage1GridGroup,
                                        const tcb::span<const Stage1SSDGroup> &stage1SSDGroup,
                                        bool isAllowNoCornerRefine,
                                        bool isWithCornerRefine);
template void centerAndCornersToTags<5>(std::vector<TagDetectInfo> &tagDetectInfo,
                                        std::vector<std::vector<cv::Point2d>> &centerToCornerLinks,
                                        const tcb::span<const Stage1GridGroup<5>> &stage1GridGroup,
                                        const tcb::span<const Stage1SSDGroup> &stage1SSDGroup,
                                        bool isAllowNoCornerRefine,
                                        bool isWithCornerRefine);
template void centerAndCornersToTags<9>(std::vector<TagDetectInfo> &tagDetectInfo,
                                         std::vector<std::vector<cv::Point2d>> &centerToCornerLinks,
                                         const tcb::span<const Stage1GridGroup<9>> &stage1GridGroup,
                                         const tcb::span<const Stage1SSDGroup> &stage1SSDGroup,
                                         bool isAllowNoCornerRefine,
                                         bool isWithCornerRefine);
template void centerAndCornersToTags<10>(std::vector<TagDetectInfo> &tagDetectInfo,
                                         std::vector<std::vector<cv::Point2d>> &centerToCornerLinks,
                                         const tcb::span<const Stage1GridGroup<10>> &stage1GridGroup,
                                         const tcb::span<const Stage1SSDGroup> &stage1SSDGroup,
                                         bool isAllowNoCornerRefine,
                                         bool isWithCornerRefine);
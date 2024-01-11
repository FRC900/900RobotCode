#include <iostream>
#include "deeptag_ros/marker_dict.h"
#include "deeptag_ros/points_and_ids.h"
#include "deeptag_ros/stage2_keypoint_group.h"

tcb::span<Stage2KeypointGroup> createKeypointGroup();
int main(void)
{
    ArucoMarkerDict<4> arucoMarkerDict(cv::aruco::DICT_APRILTAG_16h5);

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 526.635, 0, 654.365, 0, 526.475, 347.125, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 8) << -0.0419871, 0.0111333, -0.000225698, 0.000263764, -0.00526231, 0, 0, 0);
    cv::Mat stage1H = (cv::Mat_<double>(3, 3) << 2.01241655e+00, -5.13027002e-01, -8.42403727e+02, 3.58385142e-01, 1.99708503e+00, -7.00689103e+02, -4.56462926e-04, -4.60325260e-04, 1.00000000e+00);

    const auto hStage2KeypointGroup = createKeypointGroup();
    cv::Mat H = (cv::Mat_<double>(3, 3) << 2.01241655e+00, -5.13027002e-01, -8.42403727e+02, 3.58385142e-01, 1.99708503e+00, -7.00689103e+02, -4.56462926e-04, -4.60325260e-04, 1.00000000e+00);

    std::vector<float2> hStage2Corners = {float2{41.26249694824219, 40.57499694824219},
                                          float2{216.03750610351562, 41.98749923706055},
                                          float2{217.6750030517578, 218.5500030517578},
                                          float2{37.875, 216.97500610351562}};
    double matchRatio;
    std::array<PointsAndIDs, 36> pointsAndIds;
    arucoMarkerDict.getUnitTagTemplate().matchFineGrid(matchRatio,
                                                       pointsAndIds,
                                                       hStage2KeypointGroup,
                                                       H,
                                                       hStage2Corners,
                                                       cameraMatrix, // cameraMatrix
                                                       distCoeffs);  // distCoeffs

    const auto roiUpdated = arucoMarkerDict.getUnitTagTemplate().updateCornersInImage(pointsAndIds, stage1H, cameraMatrix, distCoeffs);
    std::array<int, 36> tagBits;
    for (size_t i = 0; i < pointsAndIds.size(); i++)
    {
        tagBits[i] = pointsAndIds[i].m_id;
    }
    int mainIdx;
    int decimalId;
    uint64_t binaryId;
    int hammingDist = 4;

    arucoMarkerDict.getMainIdx(mainIdx, decimalId, binaryId, tagBits, hammingDist);
    std::cout << "mainIdx = " << mainIdx << " decimalId = " << decimalId << " binaryId = " << binaryId << std::endl;
}

tcb::span<Stage2KeypointGroup> createKeypointGroup()
{
    const double3 kpts[] = {
        double3{55.23993682861328, 54.205501556396484, 0},
        double3{84.2773208618164, 54.12934875488281, 0},
        double3{114.0247802734375, 54.272335052490234, 0},
        double3{143.5234375, 54.66436004638672, 0},
        double3{173.03598022460938, 54.914146423339844, 0},
        double3{202.38662719726562, 55.26726531982422, 0},
        double3{54.49873352050781, 82.77178955078125, 0},
        double3{84.00898742675781, 83.03047180175781, 0},
        double3{113.39939880371094, 83.23968505859375, 1},
        double3{143.18203735351562, 83.66826629638672, 0},
        double3{172.85000610351562, 83.92138671875, 0},
        double3{202.16134643554688, 84.1330337524414, 0},
        double3{54.353126525878906, 112.11251068115234, 0},
        double3{83.44180297851562, 112.6031265258789, 0},
        double3{113.13848876953125, 112.62303161621094, 1},
        double3{142.7757110595703, 113.08316040039062, 0},
        double3{172.90237426757812, 113.60249328613281, 1},
        double3{202.474609375, 113.80005645751953, 0},
        double3{53.85526657104492, 142.18653869628906, 0},
        double3{83.07667541503906, 142.6084747314453, 1},
        double3{113.04017639160156, 142.674560546875, 0},
        double3{142.76376342773438, 143.0010986328125, 1},
        double3{173.0109405517578, 143.6281280517578, 1},
        double3{202.48741149902344, 143.6513214111328, 0},
        double3{53.06562423706055, 172.4953155517578, 0},
        double3{82.54219055175781, 172.51718139648438, 1},
        double3{112.74296569824219, 173.21719360351562, 0},
        double3{142.6812744140625, 173.5148468017578, 0},
        double3{173.09521484375, 173.9022216796875, 1},
        double3{202.89405822753906, 174.39337158203125, 0},
        double3{52.32421875, 203.3562469482422, 0},
        double3{82.13800048828125, 203.1912841796875, 0},
        double3{112.32500457763672, 203.48223876953125, 0},
        double3{142.48745727539062, 203.76181030273438, 0},
        double3{172.88516235351562, 203.9582061767578, 0},
        double3{203.13906860351562, 204.14453125, 0}};
    static Stage2KeypointGroup *hStage2KeypointGroup = new Stage2KeypointGroup[sizeof(kpts) / sizeof(kpts[0])];
    for (size_t i = 0; i < (sizeof(kpts) / sizeof(kpts[0])); i++)
    {
        hStage2KeypointGroup[i].m_keypoint.x = kpts[i].x;
        hStage2KeypointGroup[i].m_keypoint.y = kpts[i].y;
        hStage2KeypointGroup[i].m_label = kpts[i].z;
    }
    return tcb::span<Stage2KeypointGroup>(hStage2KeypointGroup, sizeof(kpts) / sizeof(kpts[0]));
}
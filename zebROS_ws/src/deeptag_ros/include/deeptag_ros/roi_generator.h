#ifndef ROI_GENERATOR_INC__
#define ROI_GENERATOR_INC__

#include <vector>
#include <opencv2/core/types.hpp>  // for Point2d
#include "deeptag_ros/span.hpp"

template <uint32_t BATCH_SIZE> class Stage1GridGroup;
class Stage1SSDGroup;
class TagDetectInfo;

template <uint32_t BATCH_SIZE>
void centerAndCornersToTags(std::vector<TagDetectInfo> &tagDetectInfo,
                            std::vector<std::vector<cv::Point2d>> &centerToCornerLinks,
                            const tcb::span<const Stage1GridGroup<BATCH_SIZE>> &stage1GridGroup,
                            const tcb::span<const Stage1SSDGroup> &stage1SSDGroup,
                            bool isAllowNoCornerRefine = true,
                            bool isWithCornerRefine = true);

#endif

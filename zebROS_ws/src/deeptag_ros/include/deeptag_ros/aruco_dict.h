#ifndef ARUCO_DICT_INC__
#define ARUCO_DICT_INC__

#include <map>
#include <optional>
#include <vector>

#include "opencv2/aruco/dictionary.hpp"
// Class which stores binary tag values -> decimal tag ID mappings for a given
// dictionary type (e.g. 16h5 or 36h11)
class ArucoDict
{
public:
    explicit ArucoDict(const cv::aruco::PREDEFINED_DICTIONARY_NAME dictType);
    ArucoDict(const ArucoDict &other) = delete;
    ArucoDict(ArucoDict &&other) noexcept = delete;

    ArucoDict &operator=(const ArucoDict &other) = delete;
    ArucoDict &operator=(ArucoDict &&other) noexcept = delete;

    virtual ~ArucoDict() = default;
    int getGridSize(void) const;
    std::optional<int> getDecimalId(const uint64_t tagBinaryId) const;
    const std::vector<uint64_t> &getKeys(void) const;

private:
    // Map from tag bitfield to decimal tag ID
    std::map<uint64_t, unsigned int> m_tagLookupBitmap;
    // List of tag keys
    std::vector<uint64_t> m_tagKeys;
    int m_gridSize;
};

#endif
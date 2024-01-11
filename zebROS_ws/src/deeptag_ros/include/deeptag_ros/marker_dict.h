#ifndef MARKER_DICT_INC__
#define MARKER_DICT_INC__

#include "opencv2/aruco/dictionary.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

// stores codebook
// stores hamming dist - maybe move to getMainIdx param?, same with max_check_count
// used UnitTagTemplate. 
//   in get_rotation_set

// GRID_SIZE comes from codebook

template<size_t GRID_SIZE, class UNIT_TAG_TEMPLATE, class CODEBOOK>
class MarkerDict
{
public:
    explicit MarkerDict(const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryName);
    MarkerDict(const MarkerDict &other) = delete;
    MarkerDict(MarkerDict &&other) noexcept = delete;

    MarkerDict &operator=(const MarkerDict &other) = delete;
    MarkerDict &operator=(MarkerDict &&other) noexcept = delete;
    virtual ~MarkerDict() = default;

    const UNIT_TAG_TEMPLATE &getUnitTagTemplate(void) const { return m_unitTagTemplate; }
    bool getMainIdx(int &mainIdx, int &decimalId, uint64_t &binaryId, const std::array<int, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &bits, const int hammingDist) const;

    static constexpr size_t getGridSize(void) { return GRID_SIZE; }

private:
    const CODEBOOK m_codebook;
    const UNIT_TAG_TEMPLATE m_unitTagTemplate{};

    std::array<std::array<size_t, GRID_SIZE * GRID_SIZE>, 4> m_rotationSets{};
    size_t m_maxIdx{0};
};

#include "unit_tag_template.h"
#include "aruco_dict.h"
template <size_t GRID_SIZE>
using ArucoMarkerDict = MarkerDict<GRID_SIZE, UnitTagTemplateArucotag<GRID_SIZE>, ArucoDict>;
#endif
#ifndef UNIT_ARUCO_TAG_INC__
#define UNIT_ARUCO_TAG_INC__

#include <array>
#include <cstddef>

#include "unit_chessboard_tag.h"

template <std::size_t GRID_SIZE>
std::array<uint8_t, GRID_SIZE * GRID_SIZE> constexpr makeBinaryIDArray()
{
    std::array<uint8_t, GRID_SIZE * GRID_SIZE> tempArray{};
    for (size_t i = 0; i < tempArray.size(); i++)
    {
        tempArray[i] = i % 2;
    }
    return tempArray;
}

template <size_t GRID_SIZE, uint8_t MAX_LABEL = 1, uint8_t MAX_ELEM_LABEL = 1>
class UnitArucoTag : public UnitChessboardTag<GRID_SIZE + 2, MAX_LABEL, MAX_ELEM_LABEL>
{
public:
    UnitArucoTag();
    UnitArucoTag(const UnitArucoTag &other) = delete;
    UnitArucoTag(UnitArucoTag &&other) noexcept = delete;

    UnitArucoTag &operator=(const UnitArucoTag &other) = delete;
    UnitArucoTag &operator=(UnitArucoTag &&other) noexcept = delete;

    virtual ~UnitArucoTag() = default;

    static const std::array<uint8_t, GRID_SIZE * GRID_SIZE> &getBinaryIds(void) { return m_binaryIds; }
    static size_t getGridSize(void) { return GRID_SIZE; }

private:
    static constexpr std::array<uint8_t, GRID_SIZE * GRID_SIZE> m_binaryIds{makeBinaryIDArray<GRID_SIZE>()};
};
#endif

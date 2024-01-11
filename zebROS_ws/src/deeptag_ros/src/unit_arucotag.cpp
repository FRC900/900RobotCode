#include "deeptag_ros/unit_arucotag.h"
#include <cstdint>


template <size_t GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
std::array<uint8_t, (GRID_SIZE + 2) * (GRID_SIZE + 2)> constexpr makeArucoElemLabels()
{
    constexpr size_t n = GRID_SIZE + 2;
    std::array<uint8_t, n * n> elemLabels{};
    constexpr uint8_t elemCornerLabel = ((MAX_ELEM_LABEL - MAX_LABEL) > 0) ? (MAX_LABEL + 1) : 0;
    constexpr uint8_t elemBorderLabel = ((MAX_ELEM_LABEL - MAX_LABEL) > 1) ? (MAX_LABEL + 2) : 0;
    std::ranges::fill(elemLabels, elemBorderLabel);
    elemLabels[0] = elemCornerLabel;
    elemLabels[n - 1] = elemCornerLabel;
    elemLabels[elemLabels.size() - n] = elemCornerLabel;
    elemLabels.back() = elemCornerLabel;

    for (size_t y = 0; y < GRID_SIZE; y++)
    {
        for (size_t x = 0; x < GRID_SIZE; x++)
        {
            elemLabels[(y + 1) * n + x + 1] = (y * GRID_SIZE + x) % 2;
        }
    }
    return elemLabels;
}

template <size_t GRID_SIZE, uint8_t MAX_LABEL>
std::array<uint8_t, (GRID_SIZE + 2) * (GRID_SIZE + 2)> constexpr makeArucoLabels()
{
    constexpr size_t n = GRID_SIZE + 2;
    std::array<uint8_t, n * n> labels{};
    std::ranges::fill(labels, MAX_LABEL + 1);

    for (size_t y = 0; y < GRID_SIZE; y++)
    {
        for (size_t x = 0; x < GRID_SIZE; x++)
        {
            labels[(y + 1) * n + x + 1] = (y * GRID_SIZE + x) % 2;
        }
    }
    return labels;
}

// Use GRID_SIZE + 2, since chessboard fine grid size adds a left/right/top/bottom
// border for the grid that isn't reflected in the Arucotag NxN grid size
template <size_t GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
UnitArucoTag<GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::UnitArucoTag()
    : UnitChessboardTag<GRID_SIZE + 2, MAX_LABEL, MAX_ELEM_LABEL>(makeArucoElemLabels<GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>(),
                                                                  makeArucoLabels<GRID_SIZE, MAX_LABEL>(),
                                                                  makeArucoLabels<GRID_SIZE, MAX_LABEL>())
{
}

template class UnitArucoTag<4, 1, 1>; // 16h5
template class UnitArucoTag<5, 1, 1>; // 25h9 - we'll never use this, probably
template class UnitArucoTag<6, 1, 1>; // 36h10/36h11
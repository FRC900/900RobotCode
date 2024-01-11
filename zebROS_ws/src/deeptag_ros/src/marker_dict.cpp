#include <algorithm>
#include "deeptag_ros/marker_dict.h"

#ifdef DEBUG
#include <iostream>
#include <iterator>
#endif

template <size_t GRID_SIZE>
static std::array<size_t, (GRID_SIZE + 2) * (GRID_SIZE + 2)> constexpr makeIdxListAll()
{
    constexpr size_t n = GRID_SIZE + 2;
    std::array<size_t, n * n> idxListAll{};
    for (size_t i = 0; i < idxListAll.size(); i++)
    {
        idxListAll[i] = i;
    }
    return idxListAll;
}

template <size_t GRID_SIZE, class UNIT_TAG_TEMPLATE, class CODEBOOK>
MarkerDict<GRID_SIZE, UNIT_TAG_TEMPLATE, CODEBOOK>::MarkerDict(const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryName)
    : m_codebook{dictionaryName}
{
    constexpr auto idxListAll = makeIdxListAll<GRID_SIZE>();
    std::array<size_t, (GRID_SIZE + 2) * (GRID_SIZE + 2)> fineGridPointsRotated;
    for (size_t i = 0; i < m_rotationSets.size(); i++)
    {
        m_unitTagTemplate.reorderPointsWithMainIdx(fineGridPointsRotated, m_rotationSets[i], i, idxListAll);
        m_maxIdx = std::max(m_maxIdx, *std::max_element(m_rotationSets[i].cbegin(), m_rotationSets[i].cend()));
    }
}

// Return number of bits which are different between key and tag
static int checkHammingDist1d(const uint64_t key, const uint64_t tag)
{
    uint64_t diffBits = key ^ tag;
    int count = 0;
    while(diffBits != 0)
    {
        count += diffBits & 1;
        diffBits >>= 1;
    }
    return count;
}

template <size_t GRID_SIZE>
struct MarkerBits
{
    explicit MarkerBits(const int mainIdx)
        : m_mainIdx{mainIdx}
    {
    }

    std::array<int, GRID_SIZE * GRID_SIZE> m_bits;
    uint64_t m_binaryValue{0};
    int m_mainIdx;
};

// Given a list of tag ID bit, expand the unknown ones into 
// both 0 and 1 values
template <size_t GRID_SIZE>
void expandUnknowns(std::vector<MarkerBits<GRID_SIZE>> &bits)
{
    for (size_t entry = 0; entry < bits.size(); entry++)
    {
        for (size_t bit = 0; bit < bits[entry].m_bits.size(); bit++)
        {
            // For each unknown bit seen, make a copy of the
            // MarkerBits entry.  Set the bit in the original to 0,
            // and the bit in the copy to 1.
            if (bits[entry].m_bits[bit] == -1)
            {
                bits.push_back(bits[entry]);
                bits[entry].m_bits[bit] = 0;
                bits.back().m_bits[bit] = 1;
            }
        }
    }
}

// Take a list of tag bits (0/1 plus -1 for not identified)
// Try to determine the decimal tag ID for that set of bits
//
// If # of unidentified bits > some threshold, return
// Otherwise construct all sets of possible bits, sub-ing in values with
// both 0 and 1 for each -1.
// If there's an exact match, return it.
// If not, find the ID with the lowest hamming dist to a valid ID. If that
// is < hammingDist arg, return a match, otherwise return no match
template <size_t GRID_SIZE, class UNIT_TAG_TEMPLATE, class CODEBOOK>
bool MarkerDict<GRID_SIZE, UNIT_TAG_TEMPLATE, CODEBOOK>::getMainIdx(int &mainIdx,
                                                                    int &decimalId,
                                                                    uint64_t &binaryId,
                                                                    const std::array<int, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &bits,
                                                                    const int hammingDist) const
{
    mainIdx = 0;
    decimalId = -1;

    size_t unknownCount = 0;
    for (const auto &b : bits)
    {
        if (b == -1)
        {
            unknownCount += 1;
        }
    }
    // TODO : config item
    if (unknownCount > 6)
    {
        return false;
    }
    // For each possible rotation of the tag, extract
    // bits from the detected tag in the correct order
    // to make a set of marker bits. 
    std::vector<MarkerBits<GRID_SIZE>> markerBits;
    for (int ii = 0; ii < static_cast<int>(m_rotationSets.size()); ii++)
    {
        markerBits.push_back(MarkerBits<GRID_SIZE>{ii});
        const auto &idxList = m_rotationSets[ii];
        for (size_t bit = 0; bit < idxList.size(); bit++)
        {
            markerBits.back().m_bits[bit] = bits[idxList[bit]];
        }
#ifdef DEBUG
        std::cout << "markerBits[" << ii << "] = ";
        std::copy(markerBits.back().m_bits.cbegin(), markerBits.back().m_bits.cend(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
#endif
    }

    // Expand any unknown bits into a pair of 0/1 values for that bit location
    expandUnknowns(markerBits);

    // Convert the array of bits into a single uint64_t value
    // Use this to try and find an exact match in the codebook
    for (auto &m : markerBits)
    {
        uint64_t tagBinaryId = 0;
        for (size_t bit = 0; bit < m.m_bits.size(); bit++)
        {
            if (m.m_bits[bit])
            {
                tagBinaryId |= 1ULL << ((GRID_SIZE * GRID_SIZE) - 1 - bit);
            }
        }
        m.m_binaryValue = tagBinaryId;
#ifdef DEBUG
        printf("tagBinaryId = %lx\n", tagBinaryId);
#endif
        const auto tagDecimalId = m_codebook.getDecimalId(tagBinaryId);
        if (tagDecimalId)
        {
#ifdef DEBUG
            std::cout << " Match, mainIdx = " << m.m_mainIdx << std::endl;
#endif
            mainIdx = m.m_mainIdx;
            decimalId = *tagDecimalId;
            binaryId = tagBinaryId;
            return true;
        }
    }

    // If there's not an exact match, try to find the closest
    // 1-d hamming distance match among the codebook keys
    if (hammingDist > 0)
    {
        int checkCount = 0;
        int minHammingDist = hammingDist;
        for (const auto &k : m_codebook.getKeys())
        {
            for (const auto &m : markerBits)
            {
                checkCount += 1;
                const auto dist = checkHammingDist1d(k, m.m_binaryValue);
                if (dist < minHammingDist)
                {
                    mainIdx = m.m_mainIdx;
                    decimalId = *(m_codebook.getDecimalId(k));
                    binaryId = m.m_binaryValue;
                    minHammingDist = dist;
                    if (minHammingDist <= 1)
                    {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

template class MarkerDict<4, UnitTagTemplateArucotag<4>, ArucoDict>;
template class MarkerDict<5, UnitTagTemplateArucotag<5>, ArucoDict>;
template class MarkerDict<6, UnitTagTemplateArucotag<6>, ArucoDict>;
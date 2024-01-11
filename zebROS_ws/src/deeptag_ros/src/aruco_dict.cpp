//#include <iostream>

#include "deeptag_ros/aruco_dict.h"

ArucoDict::ArucoDict(const cv::aruco::PREDEFINED_DICTIONARY_NAME dictType)
{
    const auto dict = cv::aruco::getPredefinedDictionary(dictType);
    m_gridSize = dict->markerSize;
    /**
     * bytesList contains the marker codewords where
     * - bytesList.rows is the dictionary size
     * - each marker is encoded using `nbytes = ceil(markerSize*markerSize/8.)`
     * - each row contains all 4 rotations of the marker, so its length is `4*nbytes`
     *
     * `bytesList.ptr(i)[k*nbytes + j]` is then the j-th byte of i-th marker, in its k-th rotation.
     */
    //std::cout << dict->bytesList << std::endl;
    const int tagBytes = ceil(m_gridSize * m_gridSize / 8.);
    for (int tagIdx = 0; tagIdx < dict->bytesList.rows; tagIdx++)
    {
        uint64_t tagBits{0};
        int bitCount = 0;
        for (int byte = 0; byte < tagBytes; byte++)
        {
            auto thisByte = dict->bytesList.at<uint8_t>(tagIdx, byte);
            //printf ("tagIdx = %d, byte = %d, tagBytes = %d, thisByte = %x\n", tagIdx, byte, tagBytes, thisByte);
            // Last byte is right justified, so in cases where the grid size squared
            // isn't a multiple of 8, need to start in the middle of the last byte
            // so the leading 0s are ignored
            int bitOffset = std::max(0, bitCount + 8 - m_gridSize * m_gridSize);
            for (int bit = bitOffset; bit < 8; bit++)
            {
                if (thisByte & (1ULL << (7 - bit)))
                {
                    tagBits |= 1ULL << ((8 * byte) + (bit - bitOffset));
                }
                bitCount += 1;
            }
        }
        //printf("id = %d, tagBits = %lx\n", tagIdx, tagBits);
        tagBits &= (1ULL << (m_gridSize * m_gridSize)) - 1;

        m_tagLookupBitmap[tagBits] = tagIdx;
        m_tagKeys.push_back(tagBits);
    }
}

int ArucoDict::getGridSize(void) const
{
    return m_gridSize;
}

std::optional<int> ArucoDict::getDecimalId(const uint64_t tagBinaryId) const
{
    const auto it = m_tagLookupBitmap.find(tagBinaryId);
    if (it == m_tagLookupBitmap.cend())
    {
        return std::nullopt;
    }
    return it->second;
}

const std::vector<uint64_t> &ArucoDict::getKeys(void) const
{
    return m_tagKeys;
}

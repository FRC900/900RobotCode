#ifndef DECODER_SOFTMAX_RESULTS__
#define DECODER_SOFTMAX_RESULTS__

class DecoderSoftmaxResults
{
public:
    float m_confidence1; // Skip bg, just keep two useful classes
    float m_confidence2;
};

#endif

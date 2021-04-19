#ifndef CUDA_UTILS_INC_
#define CUDA_UTILS_INC_
#include <cstddef>
#include <cuda_runtime.h>

#ifndef cudaSafeCall
#define cudaSafeCall(call) cudaSafeCallWrapper((call),__FILE__,__LINE__)
#endif

void cudaSafeCallWrapper(cudaError err, const char* file, const int line);
size_t numBlocks(size_t threads_per_block, size_t N);
bool hasCudaGPU(void);
#endif

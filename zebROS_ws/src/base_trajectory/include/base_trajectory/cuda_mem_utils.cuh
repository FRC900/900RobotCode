#ifndef CUDA_MEM_UTILS_INC_
#define CUDA_MEM_UTILS_INC_

#include <cstddef>
#include "base_trajectory/cuda_utils.h"

// Functions to allocate device and host pinned memory. The
// code expects the caller to provide both a pointer to the memory
// and the previous capacity of it.  Reallocation only happens if
// the requested size is greater than the current capactiy - this
// speeds up the common case where ptr points to a buffer large
// enough to hold the requested size
// Returns true if memory has been reallocated, false otherwise.

template <class T>
bool reallocHostPinnedMemory(T *&ptr, size_t &capacity, size_t size)
{
	if (capacity >= size)
		return false;

	if (ptr)
		cudaSafeCall(cudaFreeHost(ptr));

	capacity = size;
	cudaSafeCall(cudaMallocHost(&ptr, capacity));
	return true;
}

template <class T>
bool reallocDeviceMemory(T *&ptr, size_t &capacity, size_t size)
{
	if (capacity >= size)
		return false;

	if (ptr)
		cudaSafeCall(cudaFree(ptr));

	capacity = size;
	cudaSafeCall(cudaMalloc(&ptr, capacity));
	return true;
}


#endif

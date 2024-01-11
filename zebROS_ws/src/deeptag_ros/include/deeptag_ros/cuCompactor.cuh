// https://raw.githubusercontent.com/knotman90/cuStreamComp/master/src/cuCompactor.cuh
// See repo above for LICENSE
/*
 * cuCompactor.h
 *
 *  Created on: 21/mag/2015
 *      Author: knotman
 */
// Used to quickly compact just the results with a high enough confidence into
// the start of a separate results array.

#ifndef CUCOMPACTOR_H_
#define CUCOMPACTOR_H_

#include "deeptag_ros/cuda_utils.h"
//#include "cub/version.cuh"
//#include "cub/device/device_scan.cuh"

namespace cuCompactor
{

constexpr uint32_t warpSize = 32;
constexpr uint32_t FULL_MASK = 0xffffffff;

__device__ __inline__ uint32_t pow2i(const uint32_t e)
{
	return 1U << e;
}

template <typename INPUT, typename Predicate>
__global__ void computeBlockCounts(INPUT *d_input, const uint32_t length, uint32_t *d_BlockCounts, const Predicate predicate)
{
	const uint32_t idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx < length)
	{
		const uint32_t pred = predicate(d_input, idx, length);
#ifdef DEBUG
		if (pred)
			printf("idx = %u, length = %u, pred = %u\n", idx, length, pred);
#endif
		const uint32_t BC = __syncthreads_count(pred);

		if (threadIdx.x == 0)
		{
			d_BlockCounts[blockIdx.x] = BC; // BC will contain the number of valid elements in all threads of this thread block
		}
	}
}

template <typename INPUT, typename OUTPUT, typename GRID_DIM, typename Predicate>
__global__ void compactK(const INPUT *d_input1,
						 const INPUT *d_input2,
						 const INPUT *d_input3,
						 const GRID_DIM *gridDim,
						 const float centerVariance,
						 const float sizeVariance,
						 const uint32_t length,
						 OUTPUT *d_output,
						 const uint32_t *d_BlocksOffset,
						 const Predicate predicate)
{
	const uint32_t idx = threadIdx.x + blockIdx.x * blockDim.x;
	extern __shared__ uint32_t warpTotals[];
	if (idx < length)
	{
		const uint32_t pred = predicate(d_input1, idx, length);
		const uint32_t w_i = threadIdx.x / warpSize;	// warp index
		const uint32_t w_l = idx % warpSize;			// thread index within a warp

		// compute exclusive prefix sum based on predicate validity to get output offset for thread in warp
		const uint32_t t_m = FULL_MASK >> (warpSize - w_l); // thread mask
#if (CUDART_VERSION < 9000)
		const int b = __ballot(pred) & t_m; // ballot result = number whose ith bit is one if the ith's thread pred is true masked up to the current index in warp
#else
		const uint32_t b = __ballot_sync(FULL_MASK, pred) & t_m;
#endif
		const uint32_t t_u = __popc(b); // popc count the number of bit one. simply count the number predicated true BEFORE MY INDEX

		// last thread in warp computes total valid counts for the warp
		if (w_l == warpSize - 1)
		{
			warpTotals[w_i] = t_u + pred;
			//printf("warpTotals[%u] = %u\n", w_i, warpTotals[w_i]);
		}

		// need all warps in thread block to fill in warpTotals before proceeding
		__syncthreads();

		// first numWarps threads in first warp compute exclusive prefix sum to get output offset for each warp in thread block
		const uint32_t numWarps = blockDim.x / warpSize;
		const uint32_t numWarpsMask = FULL_MASK >> (warpSize - numWarps);
		if (w_i == 0 && w_l < numWarps)
		{
			uint32_t w_i_u = 0;
			for (int j = 0; j <= 5; j++)
			{ // must include j=5 in loop in case any elements of warpTotals are identically equal to 32
#if (CUDART_VERSION < 9000)
				int b_j = __ballot(warpTotals[w_l] & pow2i(j)); // # of the ones in the j'th digit of the warp offsets
#else
				const uint32_t b_j = __ballot_sync(numWarpsMask, warpTotals[w_l] & pow2i(j));
#endif
				w_i_u += (__popc(b_j & t_m)) << j;
				// printf("indice %i t_m=%i,j=%i,b_j=%i,w_i_u=%i\n",w_l,t_m,j,b_j,w_i_u);
			}
			warpTotals[w_l] = w_i_u;
		}

		// need all warps in thread block to wait until prefix sum is calculated in warpTotals
		__syncthreads();

		// if valid element, place the element in proper destination address based on thread offset in warp, warp offset in block, and block offset in grid
		if (pred)
		{
			//printf("Calling %s copyInto on index %u from source %u : t_u = %u, warpTotals[%u] = %u, d_BlocksOffset[%u] = %u\n", predicate.getName(), t_u + warpTotals[w_i] + d_BlocksOffset[blockIdx.x], idx, t_u, w_i, warpTotals[w_i], blockIdx.x, d_BlocksOffset[blockIdx.x]);
			d_output[t_u + warpTotals[w_i] + d_BlocksOffset[blockIdx.x]].copyInto(d_input1,
																				  d_input2,
																				  d_input3,
																				  gridDim,
																				  centerVariance,
																				  sizeVariance,
																				  idx,
																				  length);
		}
	}
}

// These are small arrays, so a simple approach should work?
__global__ void exclusiveScanKernel(uint32_t *dOut,
									const uint32_t *dIn,
									const uint32_t count)
{
	const uint32_t idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx == 0)
	{
		*dOut = 0;
		for (uint32_t i = 0; i < count; i++)
		{
			dOut[i + 1] = dOut[i] + dIn[i];
		}
	}
}

template <class T>
__global__ void printArray_GPU(T *hd_data, int size, int newline)
{
	int w = 0;
	for (int i = 0; i < size; i++)
	{
		if (i % newline == 0)
		{
			printf("\n%i -> ", w);
			w++;
		}
		printf("%i ", hd_data[i]);
	}
	printf("\n");
}

template <typename OUTPUT, typename GRID_DIM, typename Predicate>
void compact(const std::array<const float *, 3> &d_inputs, // {confidences, keypoint, values}
			 OUTPUT *d_output,							   // structure holding filtered results processed into user-friendly-ish struct
			 uint32_t *h_OutputCountPtr,				   // 2 entry host array holding the values summed to get the total output count
			 const GRID_DIM *gridDims,					   // map of tensor output coords to screen coord
			 const float centerVariance,				   // variance of location inference results
			 const float sizeVariance,					   // variance of location inference results
			 const uint32_t length,						   // output tensor lengths
			 const Predicate predicate,					   // functor for filtering outputs on confidence vals
			 const uint32_t blockSize,					   // 1d block size used to calc below two arrays
			 uint32_t *d_BlocksCount,					   // holds count of results from previous warps
			 uint32_t *d_BlocksOffset,
			 cudaStream_t cudaStream)
{
	const uint32_t numBlocks = iDivUp(length, blockSize);

	// phase 1: count number of valid elements in each thread block
	computeBlockCounts<<<numBlocks, blockSize, 0, cudaStream>>>(d_inputs[0], length, d_BlocksCount, predicate);
	cudaSafeCall(cudaGetLastError());

	// phase 2: compute exclusive prefix sum of valid block counts to get output offset for each thread block in grid
	// Offsets for each block is the cumulative count of valid entries in previous block
	// Counts for each block is the count of valid entries in that block
	exclusiveScanKernel<<<1, 1, 0, cudaStream>>>(d_BlocksOffset, d_BlocksCount, numBlocks);
	cudaSafeCall(cudaGetLastError());

	// Save last value of both block and count to get the total number of valid items in d_output
    cudaSafeCall(cudaMemcpyAsync(h_OutputCountPtr, d_BlocksOffset + numBlocks, sizeof(*d_BlocksOffset), cudaMemcpyDeviceToHost, cudaStream));

	// phase 3: compute output offset for each thread in warp and each warp in thread block, then output valid elements
	compactK<<<numBlocks, blockSize, sizeof(uint32_t) * (blockSize / warpSize), cudaStream>>>(d_inputs[0],
																							  d_inputs[1],
																							  d_inputs[2],
																							  gridDims,
																							  centerVariance,
																							  sizeVariance,
																							  length,
																							  d_output,
																							  d_BlocksOffset,
																							  predicate);
	cudaSafeCall(cudaGetLastError());
}

} /* namespace cuCompactor */
#endif /* CUCOMPACTOR_H_ */
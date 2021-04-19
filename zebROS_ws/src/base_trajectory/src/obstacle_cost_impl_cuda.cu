#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "base_trajectory/cudat_def.cuh"
#include "base_trajectory/cuda_mem_utils.cuh"
#include "base_trajectory/cuda_utils.h"
#include "base_trajectory/obstacle_cost_impl_cuda.h"

// From https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/base_local_planner/include/base_local_planner/line_iterator.h
// Added __device__ to everything for GPU work
/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** An iterator implementing Bresenham Ray-Tracing. */

class LineIterator
{
public:
  __device__
  LineIterator( int x0, int y0, int x1, int y1 )
    : x0_( x0 )
    , y0_( y0 )
    , x1_( x1 )
    , y1_( y1 )
    , x_( x0 ) // X and Y start of at first endpoint.
    , y_( y0 )
    , deltax_( abs( x1 - x0 ))
    , deltay_( abs( y1 - y0 ))
    , curpixel_( 0 )
  {
    if( x1_ >= x0_ )                 // The x-values are increasing
    {
      xinc1_ = 1;
      xinc2_ = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1_ = -1;
      xinc2_ = -1;
    }

    if( y1_ >= y0_)                 // The y-values are increasing
    {
      yinc1_ = 1;
      yinc2_ = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1_ = -1;
      yinc2_ = -1;
    }

    if( deltax_ >= deltay_ )         // There is at least one x-value for every y-value
    {
      xinc1_ = 0;                  // Don't change the x when numerator >= denominator
      yinc2_ = 0;                  // Don't change the y for every iteration
      den_ = deltax_;
      num_ = deltax_ / 2;
      numadd_ = deltay_;
      numpixels_ = deltax_;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2_ = 0;                  // Don't change the x for every iteration
      yinc1_ = 0;                  // Don't change the y when numerator >= denominator
      den_ = deltay_;
      num_ = deltay_ / 2;
      numadd_ = deltax_;
      numpixels_ = deltay_;         // There are more y-values than x-values
    }
  }

  __device__ bool isValid() const
  {
    return curpixel_ <= numpixels_;
  }

  __device__ void advance()
  {
    num_ += numadd_;              // Increase the numerator by the top of the fraction
    if( num_ >= den_ )            // Check if numerator >= denominator
    {
      num_ -= den_;               // Calculate the new numerator value
      x_ += xinc1_;               // Change the x as appropriate
      y_ += yinc1_;               // Change the y as appropriate
    }
    x_ += xinc2_;                 // Change the x as appropriate
    y_ += yinc2_;                 // Change the y as appropriate

    curpixel_++;
  }

  __device__ int getX() const { return x_; }
  __device__ int getY() const { return y_; }

  __device__ int getX0() const { return x0_; }
  __device__ int getY0() const { return y0_; }

  __device__ int getX1() const { return x1_; }
  __device__ int getY1() const { return y1_; }

private:
  int x0_; ///< X coordinate of first end point.
  int y0_; ///< Y coordinate of first end point.
  int x1_; ///< X coordinate of second end point.
  int y1_; ///< Y coordinate of second end point.

  int x_; ///< X coordinate of current point.
  int y_; ///< Y coordinate of current point.

  int deltax_; ///< Difference between Xs of endpoints.
  int deltay_; ///< Difference between Ys of endpoints.

  int curpixel_; ///< index of current point in line loop.

  int xinc1_, xinc2_, yinc1_, yinc2_;
  int den_, num_, numadd_, numpixels_;
};

// inspired From tf2/LinearMath/Vector3.h, Transform.h, Matrix3x3.h :
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
class Vector2
{
	public:
		__host__ __device__ Vector2() { }
		__host__ __device__ Vector2(const float x, const float y)
		{
			m_floats[0] = x;
			m_floats[1] = y;
		}
		Vector2(const tf2::Vector3 &v_in)
		{
			m_floats[0] = v_in.getX();
			m_floats[1] = v_in.getY();
		}
		__host__ __device__ float getX() const { return m_floats[0]; }
		__host__ __device__ float getY() const { return m_floats[1]; }
		__host__ __device__ void  setX(const float x) { m_floats[0] = x;};
		__host__ __device__ void  setY(const float y) { m_floats[1] = y;};
		/**@brief Return the dot product
		 * @param v The other vector in the dot product */
		__device__ float dot(const Vector2& v) const
		{
			return m_floats[0] * v.m_floats[0] + m_floats[1] * v.m_floats[1];
		}
		void setValue(const float x, const float y)
		{
			m_floats[0]=x;
			m_floats[1]=y;
		}
	private:
		float m_floats[2];
};
// Original comments from tf2/LinearMath/Matrix3x3.h:
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
class Matrix2x2 {
public:
	__host__ __device__ Matrix2x2() { }
	Matrix2x2(const tf2::Matrix3x3& other)
	{
		m_el[0] = other.getRow(0);
		m_el[1] = other.getRow(1);
	}
	__host__ __device__ Matrix2x2(const Matrix2x2& other)
	{
		m_el[0] = other.m_el[0];
		m_el[1] = other.m_el[1];
	}

	/** @brief Get a mutable reference to a row of the matrix as a vector
	*  @param i Row number 0 indexed */
	__device__ Vector2&  operator[](int i)
	{
		return m_el[i];
	}

	/** @brief Get a const reference to a row of the matrix as a vector
	*  @param i Row number 0 indexed */
	__device__ const Vector2& operator[](int i) const
	{
		return m_el[i];
	}
	void setRotation(const tf2::Quaternion& q)
	{
		const float d = q.length2();
		tf2FullAssert(d != float(0.0));
		const float s = float(2.0) / d;
		const float xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
		const float                                     wz = q.w() * zs;
		const float xx = q.x() * xs,  xy = q.x() * ys;
		const float yy = q.y() * ys,                    zz = q.z() * zs;
		setValue(float(1.0) - (yy + zz), xy - wz,
			xy + wz, float(1.0) - (xx + zz));
	}

	void setValue(const float xx, const float xy,
				  const float yx, const float yy)
	{
		m_el[0].setValue(xx, xy);
		m_el[1].setValue(yx, yy);
	}
private:
	///Data storage for the matrix, each vector is a row of the matrix
		Vector2 m_el[2];
};

// Original comments from tf2/LinearMath/Transform.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
class Transform2D {
	public:
		__host__ __device__ Transform2D() { }
		Transform2D(const tf2::Transform &t_in)
		  : m_basis(t_in.getBasis())
		  , m_origin(t_in.getOrigin())
		{
		}
		/**@brief Return the transform of the vector */
		__device__ Vector2 operator()(const Vector2& x) const
		{
			return Vector2(m_basis[0].dot(x) + m_origin.getX(),
						   m_basis[1].dot(x) + m_origin.getY());
		}

		/**@brief Return the transform of the vector */
		__device__ Vector2 operator*(const Vector2& x) const
		{
			return (*this)(x);
		}
		void setOrigin(const Vector2 &origin)
		{
			m_origin = origin;
		}
		void setRotation(const tf2::Quaternion& q)
		{
			m_basis.setRotation(q);
		}

	private:
		///Storage for the rotation
		Matrix2x2 m_basis;
		///Storage for the translation
		Vector2   m_origin;
};

void fromMsg(const geometry_msgs::Vector3& in, Vector2& out);
void fromMsg(const geometry_msgs::Transform& in, Transform2D& out);

struct XYCoord
{
	CudaT x;
	CudaT y;
};

template <class T>
struct CudaCostMap
{
	private:
		unsigned int size_x_;
		unsigned int size_y_;
		T resolution_;
		T origin_x_;
		T origin_y_;
	public:
		__host__ __device__
		CudaCostMap(const costmap_2d::Costmap2D *costmap)
			: size_x_(costmap->getSizeInCellsX())
			, size_y_(costmap->getSizeInCellsY())
			, resolution_(costmap->getResolution())
			, origin_x_(costmap->getOriginX())
			, origin_y_(costmap->getOriginY())
		{
		}
		__device__
		bool worldToMap(T wx, T wy, unsigned int& mx, unsigned int& my) const
		{
			if (wx < origin_x_ || wy < origin_y_)
				return false;

			mx = (unsigned int)((wx - origin_x_) / resolution_);
			my = (unsigned int)((wy - origin_y_) / resolution_);

			if (mx < size_x_ && my < size_y_)
				return true;

			return false;
		}
		__device__
		unsigned int getSizeInCellsX() const
		{
			return size_x_;
		}

};

__global__
void calculateKernel(CudaT *arcSegCosts,
		const CudaCostMap<CudaT> cm,
		const Transform2D pathToMapTransform,
		const unsigned int *potentials,
		const XYCoord *xyCoords,
		const size_t N

		)
{
	const size_t tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid == 0) {arcSegCosts[0] = 0; return; }
	if (tid >= N) return;

	Vector2 prevPointIn(xyCoords[tid - 1].x, xyCoords[tid - 1].y);
	Vector2 prevPointTransformed = pathToMapTransform * prevPointIn;

	Vector2 pointIn(xyCoords[tid].x, xyCoords[tid].y);
	Vector2 pointTransformed = pathToMapTransform * pointIn;

	auto thisCost = arcSegCosts[tid];
	thisCost = 0;

	// Then translate from map world coords to map
	// grid cell coordinates ... The map is a grid of
	// equally sized squares.  The worldToMap converts
	// from a real world (in meters) position to an x and y
	// index into the array of those grid cells.
	unsigned int mapStartX;
	unsigned int mapStartY;
	unsigned int mapEndX;
	unsigned int mapEndY;
	if (!cm.worldToMap(prevPointTransformed.getX(), prevPointTransformed.getY(), mapStartX, mapStartY) ||
		!cm.worldToMap(pointTransformed.getX(), pointTransformed.getY(), mapEndX, mapEndY))
	{
		// Large penalty for going off-map, seems to be caused by extra-curvy paths
		thisCost = 1e15;
		return;
	}

	size_t mapSquaresHit = 0;
	const auto cmSizeInCellsX = cm.getSizeInCellsX();

	// An iterator which moves along the line, hitting each
	// integer x,y coordinate exactly once
	LineIterator lineIt(mapStartX, mapStartY, mapEndX, mapEndY);

	// Start of this segment will be the same coord
	// as the end of the last one. Avoid double-counting the
	// cost in those cases. Don't skip it for the very first coord
	// because there's no previous coord to double count if none
	// have been counted yet
	if (tid != 1)
		lineIt.advance();
	while (lineIt.isValid())
	{
		const auto mapXIndex = lineIt.getX();
		const auto mapYIndex = lineIt.getY();
		const auto mapCost = potentials[mapYIndex * cmSizeInCellsX + mapXIndex];
		thisCost += mapCost;
		mapSquaresHit += 1;
		lineIt.advance();
	}
	if (mapSquaresHit)
	{
		thisCost /= static_cast<CudaT>(mapSquaresHit) * static_cast<CudaT>(4.0);
	}
}

template <class T>
ObstacleCostImplCuda<T>::ObstacleCostImplCuda()
{
	cudaSafeCall(cudaStreamCreateWithFlags(&cudaStream_, cudaStreamNonBlocking));
}

template <class T>
ObstacleCostImplCuda<T>::~ObstacleCostImplCuda()
{
	cudaSafeCall(cudaStreamDestroy(cudaStream_));
	if (dArcSegCosts_) cudaSafeCall(cudaFree(dArcSegCosts_));
	if (hArcSegCosts_) cudaSafeCall(cudaFree(hArcSegCosts_));
	if (dXYCoords_) cudaSafeCall(cudaFree(dXYCoords_));
	if (hXYCoords_) cudaSafeCall(cudaFree(hXYCoords_));
	if (dPotentials_) cudaSafeCall(cudaFree(dPotentials_));
}

template <class T>
bool ObstacleCostImplCuda<T>::calculate(std::vector<T> &arcSegCosts,
								std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
								const geometry_msgs::TransformStamped &pathToMapTransformMsg,
								const std::vector<unsigned int> &potentials,
								const std::vector<SegmentState<T>> &xStates,
								const std::vector<SegmentState<T>> &yStates)
{
	if (!costmap)
	{
		arcSegCosts.resize(xStates.size(), static_cast<T>(0));
		return true;
	}
	// Create device cudaT arcSegCosts array, uninitialized
	reallocDeviceMemory(dArcSegCosts_, dArcSegCostsCapacity_, sizeof(CudaT) * xStates.size());
	const auto cm = costmap->getCostmap();
	reallocDeviceMemory(dPotentials_, dPotentialsCapacity_, sizeof(potentials[0]) * potentials.size());
	cudaSafeCall(cudaMemcpyAsync(dPotentials_,
				potentials.data(),
				sizeof(potentials[0]) * potentials.size(),
				cudaMemcpyHostToDevice,
				cudaStream_));

	// TODO - raw memcpy from xStates, yStates, cast to float in kernel before using?
	// Saves the intermediate host buffer, but 2x memcpy size
	reallocHostPinnedMemory(hXYCoords_, hXYCoordsCapacity_, sizeof(hXYCoords_[0]) * xStates.size());
	reallocDeviceMemory(dXYCoords_, dXYCoordsCapacity_, sizeof(dXYCoords_[0]) * xStates.size());
	for (size_t i = 0; i < xStates.size(); i++)
	{
		hXYCoords_[i].x = xStates[i].position;
		hXYCoords_[i].y = yStates[i].position;
	}
	cudaSafeCall(cudaMemcpyAsync(dXYCoords_,
				hXYCoords_,
				sizeof(hXYCoords_[0]) * xStates.size(),
				cudaMemcpyHostToDevice,
				cudaStream_));

	// Call kernel
	Transform2D pathToMapTransform;
	fromMsg(pathToMapTransformMsg.transform, pathToMapTransform);
	constexpr size_t THREADS_PER_BLOCK = 128;
	calculateKernel<<<numBlocks(THREADS_PER_BLOCK, xStates.size()), THREADS_PER_BLOCK, 0, cudaStream_>>>
		(dArcSegCosts_,
		 CudaCostMap<CudaT>(cm),
		 pathToMapTransform,
		 dPotentials_,
		 dXYCoords_,
		 xStates.size());

	// create host cudaT arcSegCosts
	reallocHostPinnedMemory(hArcSegCosts_, hArcSegCostsCapacity_, sizeof(CudaT) * xStates.size());
	cudaSafeCall(cudaMemcpyAsync(hArcSegCosts_,
				dArcSegCosts_,
				sizeof(hArcSegCosts_[0]) * xStates.size(),
				cudaMemcpyDeviceToHost,
				cudaStream_));
	arcSegCosts.resize(xStates.size(), static_cast<T>(0));

	// copy back into arcSegCosts return array, calc total
	cudaSafeCall(cudaStreamSynchronize(cudaStream_));
	for (size_t i = 0; i < arcSegCosts[i]; i++)
	{
		arcSegCosts[i] = hArcSegCosts_[i];
	}

	return true;
}

template class ObstacleCostImplCuda<double>;
//template class ObstacleCostImplCuda<float>;

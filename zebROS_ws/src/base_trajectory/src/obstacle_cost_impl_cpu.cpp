#include "base_local_planner/line_iterator.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "base_trajectory/obstacle_cost_impl_cpu.h"

// 2-d version of 3d transform. Since we're operating in a flat
// plane, save a bunch of math by assuming the z component of
// the transform is 0.
// Original comments from tf2/LinearMath/Vector3.h:
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
		Vector2() = default;
		Vector2(const double x, const double y)
		{
			setValue(x, y);
		}
		Vector2(const tf2::Vector3 &v)
		{
			setValue(v.getX(), v.getY());
		}
		double getX() const { return m_doubles[0]; }
		double getY() const { return m_doubles[1]; }
		void  setX(const double x) { m_doubles[0] = x;};
		void  setY(const double y) { m_doubles[1] = y;};

		double dot(const Vector2& v) const
		{
			return m_doubles[0] * v.m_doubles[0] + m_doubles[1] * v.m_doubles[1];
		}
		void setValue(const double x, const double y)
		{
			m_doubles[0]=x;
			m_doubles[1]=y;
		}
	private:
		double m_doubles[2];
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
	Matrix2x2() { }
	Matrix2x2(const tf2::Matrix3x3& other)
	{
		m_el[0] = other.getRow(0);
		m_el[1] = other.getRow(1);
	}

	/** @brief Get a mutable reference to a row of the matrix as a vector
	*  @param i Row number 0 indexed */
	Vector2&  operator[](int i)
	{
		return m_el[i];
	}

	/** @brief Get a const reference to a row of the matrix as a vector
	*  @param i Row number 0 indexed */
	const Vector2& operator[](int i) const
	{
		return m_el[i];
	}

	void setRotation(const tf2::Quaternion& q)
	{
		const double d = q.length2();
		//tf2FullAssert(d != double(0.0));
		const double s = double(2.0) / d;
		const double xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
		const double                                     wz = q.w() * zs;
		const double xx = q.x() * xs,  xy = q.x() * ys;
		const double yy = q.y() * ys,                    zz = q.z() * zs;
		setValue(double(1.0) - (yy + zz), xy - wz,
			xy + wz, double(1.0) - (xx + zz));
	}

	void setValue(const double xx, const double xy,
				  const double yx, const double yy)
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
		Transform2D() { }
		Transform2D(const tf2::Transform &t_in)
		  : m_basis(t_in.getBasis())
		  , m_origin(t_in.getOrigin())
		{
		}
		/**@brief Return the transform of the vector */
		Vector2 operator()(const Vector2& x) const
		{
			return Vector2(m_basis[0].dot(x) + m_origin.getX(),
						   m_basis[1].dot(x) + m_origin.getY());
		}

		/**@brief Return the transform of the vector */
		Vector2 operator*(const Vector2& x) const
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
		Matrix2x2 m_basis;  ///Storage for the rotation
		Vector2   m_origin; ///Storage for the translation
};

// Original comments from tf2_geometry_msgs/tf2_geometry_msgs.h :
/*
   * Copyright (c) 2008, Willow Garage, Inc.
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

  /** \author Wim Meeussen */

// Create a Vector2 from a Vector3 message by ignoring the input z component
void fromMsg(const geometry_msgs::Vector3& in, Vector2& out)
{
	out = Vector2(in.x, in.y);
}

// Create a 2d transform from a 3d input message
void fromMsg(const geometry_msgs::Transform& in, Transform2D& out)
{
  Vector2 v;
  fromMsg(in.translation, v);
  out.setOrigin(v);
  tf2::Quaternion q;
  fromMsg(in.rotation, q);
  out.setRotation(q);
}

template <class T>
bool ObstacleCostImplCpu<T>::calculate(std::vector<T> &arcSegCosts,
									std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
									const geometry_msgs::TransformStamped &pathToMapTransformMsg,
									const std::vector<unsigned int> &potentials,
									const std::vector<SegmentState<T>> &xStates,
									const std::vector<SegmentState<T>> &yStates)
{
	arcSegCosts.resize(xStates.size());
	std::fill(arcSegCosts.begin(), arcSegCosts.end(), static_cast<T>(0));
	if (!costmap)
	{
		return true;
	}

	for (size_t i = 0; i < xStates.size(); i++)
	{
		ROS_INFO_STREAM("xStates[" << i << "].position = " << xStates[i].position);
		ROS_INFO_STREAM("yStates[" << i << "].position = " << yStates[i].position);
	}


	// Convert from path coordinates (i.e. robot centric) to map frame coordinates
	Transform2D pathToMapTransform;
	fromMsg(pathToMapTransformMsg.transform, pathToMapTransform);
	Vector2 pointIn(xStates[0].position, yStates[0].position);
	Vector2 pointTransformed = pathToMapTransform * pointIn;

	// Initialize various vars used by obstacle checking
	const auto &cm = costmap->getCostmap();
	const auto cmSizeInCellsX = cm->getSizeInCellsX();

	costmap_2d::MapLocation mapStart;
	bool                    prevMapLocationError = !cm->worldToMap(pointTransformed.getX(), pointTransformed.getY(), mapStart.x, mapStart.y);

	// Loop through each sequential pair of coordinate, summing up the value at each costmap
	// grid entry. The total obstacle cost for the path is the sum of the cost at each
	// map grid location along the way.
	for (size_t i = 1; i < xStates.size(); i++)
	{
		// Calculate cost for obstacles along the path
		// Coordinates here are in the robot base frame
		// Transform them from that to the map frame
		pointIn.setX(xStates[i].position);
		pointIn.setY(yStates[i].position);
		pointTransformed = pathToMapTransform * pointIn;
		ROS_INFO_STREAM("pointStamped path frame = (" << xStates[i].position << ", " << xStates[i].position << ")");
		ROS_INFO_STREAM("pointStamped map frame = (" << pointTransformed.getX() << ", " << pointTransformed.getY() << ")");

		// Then translate from map world coords to map
		// grid cell coordinates ... The map is a grid of
		// equally sized squares.  The worldToMap converts
		// from a real world (in meters) position to an x and y
		// index into the array of those grid cells.
		costmap_2d::MapLocation mapEnd;
		const bool currMapLocationError = !cm->worldToMap(pointTransformed.getX(), pointTransformed.getY(), mapEnd.x, mapEnd.y);

		if (prevMapLocationError || currMapLocationError)
		{
			// Large penalty for going off-map, seems to be caused by extra-curvy paths
			arcSegCosts[i] += 1e5;
		}
		else
		{
			//ROS_INFO_STREAM("Checking map coords " << mapStart.x << ", " << mapStart.y << " to "
			//<< mapEnd.x << ", " << mapEnd.y);
			// An iterator which moves along the line, hitting each
			// integer x,y coordinate exactly once
			base_local_planner::LineIterator lineIt(mapStart.x, mapStart.y, mapEnd.x, mapEnd.y);
			size_t thisMapSquaresHit = 0;
			T segCosts = 0;

			// Start of this segment will be the same coord
			// as the end of the last one. Avoid double-counting the
			// cost in those cases. Don't skip it for the very first coord
			// because there's no previous coord to double count if none
			// have been counted yet
			if (i != 1)
				lineIt.advance();
			while (lineIt.isValid())
			{
				const auto mapXIndex = lineIt.getX();
				const auto mapYIndex = lineIt.getY();
				const auto mapCost = potentials[mapYIndex * cmSizeInCellsX + mapXIndex];
				segCosts += mapCost;
				thisMapSquaresHit += 1;
				lineIt.advance();
				ROS_INFO_STREAM("  Coord " << mapXIndex << ", " << mapYIndex << " mapCost = " << mapCost << " segCosts = " << segCosts << " thisMapSquaresHit = " << thisMapSquaresHit);
			}
			if (thisMapSquaresHit)
			{
				arcSegCosts[i] += segCosts / (static_cast<T>(thisMapSquaresHit) * 4.0);
			}
		}
		// Reuse previous path->map frame transform next time through the loop
		mapStart = mapEnd;
		prevMapLocationError = currMapLocationError;
	}

	return true;
}

template class ObstacleCostImplCpu<double>;
//template class ObstacleCostImplCpu<float>;

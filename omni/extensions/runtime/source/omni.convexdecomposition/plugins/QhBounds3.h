// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef QH_BOUNDS3_H
#define QH_BOUNDS3_H

/** \addtogroup foundation
@{
*/

#include "Qh.h"


#if !QH_DOXYGEN
namespace quickhull
{
#endif

// maximum extents defined such that floating point exceptions are avoided for standard use cases
#define QH_MAX_BOUNDS_EXTENTS (QH_MAX_REAL * 0.25f)

/**
\brief Class representing 3D range or axis aligned bounding box.

Stored as minimum and maximum extent corners. Alternate representation
would be center and dimensions.
May be empty or nonempty. For nonempty bounds, minimum <= maximum has to hold for all axes.
Empty bounds have to be represented as minimum = QH_MAX_BOUNDS_EXTENTS and maximum = -QH_MAX_BOUNDS_EXTENTS for all
axes.
All other representations are invalid and the behavior is undefined.
*/
class QhBounds3
{
  public:
	/**
	\brief Default constructor, not performing any initialization for performance reason.
	\remark Use empty() function below to construct empty bounds.
	*/
	 QH_FORCE_INLINE QhBounds3()
	{
	}

	/**
	\brief Construct from two bounding points
	*/
	 QH_FORCE_INLINE QhBounds3(const QhVec3& minimum, const QhVec3& maximum);

	 QH_FORCE_INLINE void operator=(const QhBounds3& other)
	{
		minimum = other.minimum;
		maximum = other.maximum;
	}

	 QH_FORCE_INLINE QhBounds3(const QhBounds3& other)
	{
		minimum = other.minimum;
		maximum = other.maximum;
	}

	/**
	\brief Return empty bounds.
	*/
	static  QH_FORCE_INLINE QhBounds3 empty();

	/**
	\brief returns the AABB containing v0 and v1.
	\param v0 first point included in the AABB.
	\param v1 second point included in the AABB.
	*/
	static  QH_FORCE_INLINE QhBounds3 boundsOfPoints(const QhVec3& v0, const QhVec3& v1);

	/**
	\brief returns the AABB from center and extents vectors.
	\param center Center vector
	\param extent Extents vector
	*/
	static  QH_FORCE_INLINE QhBounds3 centerExtents(const QhVec3& center, const QhVec3& extent);

	/**
	\brief Sets empty to true
	*/
	 QH_FORCE_INLINE void setEmpty();

	/**
	\brief Sets the bounds to maximum size [-QH_MAX_BOUNDS_EXTENTS, QH_MAX_BOUNDS_EXTENTS].
	*/
	 QH_FORCE_INLINE void setMaximal();

	/**
	\brief expands the volume to include v
	\param v Point to expand to.
	*/
	 QH_FORCE_INLINE void include(const QhVec3& v);

	/**
	\brief expands the volume to include b.
	\param b Bounds to perform union with.
	*/
	 QH_FORCE_INLINE void include(const QhBounds3& b);

	 QH_FORCE_INLINE bool isEmpty() const;

	/**
	\brief indicates whether the intersection of this and b is empty or not.
	\param b Bounds to test for intersection.
	*/
	 QH_FORCE_INLINE bool intersects(const QhBounds3& b) const;

	/**
	 \brief computes the 1D-intersection between two AABBs, on a given axis.
	 \param	a		the other AABB
	 \param	axis	the axis (0, 1, 2)
	 */
	 QH_FORCE_INLINE bool intersects1D(const QhBounds3& a, uint32_t axis) const;

	/**
	\brief indicates if these bounds contain v.
	\param v Point to test against bounds.
	*/
	 QH_FORCE_INLINE bool contains(const QhVec3& v) const;

	/**
	 \brief	checks a box is inside another box.
	 \param	box		the other AABB
	 */
	 QH_FORCE_INLINE bool isInside(const QhBounds3& box) const;

	/**
	\brief returns the center of this axis aligned box.
	*/
	 QH_FORCE_INLINE QhVec3 getCenter() const;

	/**
	\brief get component of the box's center along a given axis
	*/
	 QH_FORCE_INLINE double getCenter(uint32_t axis) const;

	/**
	\brief get component of the box's extents along a given axis
	*/
	 QH_FORCE_INLINE double getExtents(uint32_t axis) const;

	/**
	\brief returns the dimensions (width/height/depth) of this axis aligned box.
	*/
	 QH_FORCE_INLINE QhVec3 getDimensions() const;

	/**
	\brief returns the extents, which are half of the width/height/depth.
	*/
	 QH_FORCE_INLINE QhVec3 getExtents() const;

	/**
	\brief scales the AABB.

	This version is safe to call for empty bounds.

	\param scale Factor to scale AABB by.
	*/
	 QH_FORCE_INLINE void scaleSafe(double scale);

	/**
	\brief scales the AABB.

	Calling this method for empty bounds leads to undefined behavior. Use #scaleSafe() instead.

	\param scale Factor to scale AABB by.
	*/
	 QH_FORCE_INLINE void scaleFast(double scale);

	/**
	fattens the AABB in all 3 dimensions by the given distance.

	This version is safe to call for empty bounds.
	*/
	 QH_FORCE_INLINE void fattenSafe(double distance);

	/**
	fattens the AABB in all 3 dimensions by the given distance.

	Calling this method for empty bounds leads to undefined behavior. Use #fattenSafe() instead.
	*/
	 QH_FORCE_INLINE void fattenFast(double distance);

	/**
	checks that the AABB values are not NaN
	*/
	 QH_FORCE_INLINE bool isFinite() const;

	/**
	checks that the AABB values describe a valid configuration.
	*/
	 QH_FORCE_INLINE bool isValid() const;

	/**
	Finds the closest point in the box to the point p. If p is contained, this will be p, otherwise it 
	will be the closest point on the surface of the box.
	*/
	 QH_FORCE_INLINE QhVec3 closestPoint(const QhVec3& p) const;

	QhVec3 minimum, maximum;
};

 QH_FORCE_INLINE QhBounds3::QhBounds3(const QhVec3& minimum_, const QhVec3& maximum_)
: minimum(minimum_), maximum(maximum_)
{
}

 QH_FORCE_INLINE QhBounds3 QhBounds3::empty()
{
	return QhBounds3(QhVec3(QH_MAX_BOUNDS_EXTENTS), QhVec3(-QH_MAX_BOUNDS_EXTENTS));
}

 QH_FORCE_INLINE bool QhBounds3::isFinite() const
{
	return minimum.isFinite() && maximum.isFinite();
}

 QH_FORCE_INLINE QhBounds3 QhBounds3::boundsOfPoints(const QhVec3& v0, const QhVec3& v1)
{
	return QhBounds3(v0.minimum(v1), v0.maximum(v1));
}

 QH_FORCE_INLINE QhBounds3 QhBounds3::centerExtents(const QhVec3& center, const QhVec3& extent)
{
	return QhBounds3(center - extent, center + extent);
}

 QH_FORCE_INLINE void QhBounds3::setEmpty()
{
	minimum = QhVec3(QH_MAX_BOUNDS_EXTENTS);
	maximum = QhVec3(-QH_MAX_BOUNDS_EXTENTS);
}

 QH_FORCE_INLINE void QhBounds3::setMaximal()
{
	minimum = QhVec3(-QH_MAX_BOUNDS_EXTENTS);
	maximum = QhVec3(QH_MAX_BOUNDS_EXTENTS);
}

 QH_FORCE_INLINE void QhBounds3::include(const QhVec3& v)
{
	assert(isValid());
	minimum = minimum.minimum(v);
	maximum = maximum.maximum(v);
}

 QH_FORCE_INLINE void QhBounds3::include(const QhBounds3& b)
{
	assert(isValid());
	minimum = minimum.minimum(b.minimum);
	maximum = maximum.maximum(b.maximum);
}

 QH_FORCE_INLINE bool QhBounds3::isEmpty() const
{
	assert(isValid());
	return minimum.x > maximum.x;
}

 QH_FORCE_INLINE bool QhBounds3::intersects(const QhBounds3& b) const
{
	assert(isValid() && b.isValid());
	return !(b.minimum.x > maximum.x || minimum.x > b.maximum.x || b.minimum.y > maximum.y || minimum.y > b.maximum.y ||
	         b.minimum.z > maximum.z || minimum.z > b.maximum.z);
}

 QH_FORCE_INLINE bool QhBounds3::intersects1D(const QhBounds3& a, uint32_t axis) const
{
	assert(isValid() && a.isValid());
	return maximum[axis] >= a.minimum[axis] && a.maximum[axis] >= minimum[axis];
}

 QH_FORCE_INLINE bool QhBounds3::contains(const QhVec3& v) const
{
	assert(isValid());

	return !(v.x < minimum.x || v.x > maximum.x || v.y < minimum.y || v.y > maximum.y || v.z < minimum.z ||
	         v.z > maximum.z);
}

 QH_FORCE_INLINE bool QhBounds3::isInside(const QhBounds3& box) const
{
	assert(isValid() && box.isValid());
	if(box.minimum.x > minimum.x)
		return false;
	if(box.minimum.y > minimum.y)
		return false;
	if(box.minimum.z > minimum.z)
		return false;
	if(box.maximum.x < maximum.x)
		return false;
	if(box.maximum.y < maximum.y)
		return false;
	if(box.maximum.z < maximum.z)
		return false;
	return true;
}

 QH_FORCE_INLINE QhVec3 QhBounds3::getCenter() const
{
	assert(isValid());
	return (minimum + maximum) * 0.5f;
}

 QH_FORCE_INLINE double QhBounds3::getCenter(uint32_t axis) const
{
	assert(isValid());
	return (minimum[axis] + maximum[axis]) * 0.5f;
}

 QH_FORCE_INLINE double QhBounds3::getExtents(uint32_t axis) const
{
	assert(isValid());
	return (maximum[axis] - minimum[axis]) * 0.5f;
}

 QH_FORCE_INLINE QhVec3 QhBounds3::getDimensions() const
{
	assert(isValid());
	return maximum - minimum;
}

 QH_FORCE_INLINE QhVec3 QhBounds3::getExtents() const
{
	assert(isValid());
	return getDimensions() * 0.5f;
}

 QH_FORCE_INLINE void QhBounds3::scaleSafe(double scale)
{
	assert(isValid());
	if(!isEmpty())
		scaleFast(scale);
}

 QH_FORCE_INLINE void QhBounds3::scaleFast(double scale)
{
	assert(isValid());
	*this = centerExtents(getCenter(), getExtents() * scale);
}

 QH_FORCE_INLINE void QhBounds3::fattenSafe(double distance)
{
	assert(isValid());
	if(!isEmpty())
		fattenFast(distance);
}

 QH_FORCE_INLINE void QhBounds3::fattenFast(double distance)
{
	assert(isValid());
	minimum.x -= distance;
	minimum.y -= distance;
	minimum.z -= distance;

	maximum.x += distance;
	maximum.y += distance;
	maximum.z += distance;
}

 QH_FORCE_INLINE bool QhBounds3::isValid() const
{
	return (isFinite() && (((minimum.x <= maximum.x) && (minimum.y <= maximum.y) && (minimum.z <= maximum.z)) ||
	                       ((minimum.x == QH_MAX_BOUNDS_EXTENTS) && (minimum.y == QH_MAX_BOUNDS_EXTENTS) &&
	                        (minimum.z == QH_MAX_BOUNDS_EXTENTS) && (maximum.x == -QH_MAX_BOUNDS_EXTENTS) &&
	                        (maximum.y == -QH_MAX_BOUNDS_EXTENTS) && (maximum.z == -QH_MAX_BOUNDS_EXTENTS))));
}

 QH_FORCE_INLINE QhVec3 QhBounds3::closestPoint(const QhVec3& p) const
{
	return minimum.maximum(maximum.minimum(p));
}

#if !QH_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif

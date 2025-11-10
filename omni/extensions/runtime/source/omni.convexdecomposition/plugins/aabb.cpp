// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "aabb.h"
#include "QhPreprocessor.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <float.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include <iostream>

using namespace std;

#ifdef _MSC_VER
#    pragma warning(disable : 4100)
#endif


#include <vector>

namespace vcd
{

template <typename T>
inline T Min(T a, T b)
{
    return a < b ? a : b;
}

template <typename T>
inline T Max(T a, T b)
{
    return a > b ? a : b;
}

#define kPi 3.141592653589f
const double k2Pi = 2.0f * kPi;
const double kInvPi = 1.0f / kPi;
const double kInv2Pi = 0.5f / kPi;
const double kDegToRad = kPi / 180.0f;
const double kRadToDeg = 180.0f / kPi;

inline double DegToRad(double t)
{
    return t * kDegToRad;
}

inline double RadToDeg(double t)
{
    return t * kRadToDeg;
}

inline double Sin(double theta)
{
    return sin(theta);
}

inline double Cos(double theta)
{
    return cos(theta);
}

inline void SinCos(double theta, double& s, double& c)
{
    // no optimizations yet
    s = sin(theta);
    c = cos(theta);
}

inline double Tan(double theta)
{
    return tan(theta);
}

inline double Sqrt(double x)
{
    return sqrt(x);
}

inline double ASin(double theta)
{
    return asin(theta);
}

inline double ACos(double theta)
{
    return acos(theta);
}

inline double ATan(double theta)
{
    return atan(theta);
}

inline double ATan2(double x, double y)
{
    return atan2(x, y);
}

inline double Abs(double x)
{
    return fabs(x);
}

inline double Pow(double b, double e)
{
    return pow(b, e);
}

inline double Sgn(double x)
{
    return (x < 0.0f ? -1.0f : 1.0f);
}

inline double Sign(double x)
{
    return x < 0.0f ? -1.0f : 1.0f;
}

inline double Mod(double x, double y)
{
    return fmod(x, y);
}

template <typename T>
inline void Swap(T& a, T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}

template <typename T>
inline T Clamp(T a, T low, T high)
{
    if (low > high)
        Swap(low, high);

    return Max(low, Min(a, high));
}

template <typename V, typename T>
inline V Lerp(const V& start, const V& end, const T& t)
{
    return start + (end - start) * t;
}

inline double InvSqrt(double x)
{
    return 1.0f / sqrt(x);
}

// round towards +infinity
inline int Round(double f)
{
    return int(f + 0.5f);
}

template <typename T>
T Normalize(const T& v)
{
    T a(v);
    a /= Length(v);
    return a;
}

template <typename T>
inline typename T::value_type LengthSq(const T v)
{
    return Dot(v, v);
}

template <typename T>
inline typename T::value_type Length(const T& v)
{
    typename T::value_type lSq = LengthSq(v);
    if (lSq)
        return Sqrt(LengthSq(v));
    else
        return 0.0f;
}

// this is mainly a helper function used by script
template <typename T>
inline typename T::value_type Distance(const T& v1, const T& v2)
{
    return Length(v1 - v2);
}

template <typename T>
inline T SafeNormalize(const T& v, const T& fallback = T())
{
    double l = LengthSq(v);
    if (l > 0.0f)
    {
        return v * InvSqrt(l);
    }
    else
        return fallback;
}

template <typename T>
inline T Sqr(T x)
{
    return x * x;
}

template <typename T>
inline T Cube(T x)
{
    return x * x * x;
}


template <typename T = double>
class XVector3
{
public:
    typedef T value_type;

    XVector3() : x(0.0f), y(0.0f), z(0.0f){};
    XVector3(T a) : x(a), y(a), z(a){};
    XVector3(const T* p) : x(p[0]), y(p[1]), z(p[2]){};
    XVector3(T x_, T y_, T z_) : x(x_), y(y_), z(z_)
    {
    }

    operator T*()
    {
        return &x;
    }
    operator const T*() const
    {
        return &x;
    };

    void Set(T x_, T y_, T z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    XVector3<T> operator*(T scale) const
    {
        XVector3<T> r(*this);
        r *= scale;
        return r;
    }
    XVector3<T> operator/(T scale) const
    {
        XVector3<T> r(*this);
        r /= scale;
        return r;
    }
    XVector3<T> operator+(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r += v;
        return r;
    }
    XVector3<T> operator-(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r -= v;
        return r;
    }
    XVector3<T> operator/(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r /= v;
        return r;
    }
    XVector3<T> operator*(const XVector3<T>& v) const
    {
        XVector3<T> r(*this);
        r *= v;
        return r;
    }

    XVector3<T>& operator*=(T scale)
    {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }
    XVector3<T>& operator/=(T scale)
    {
        T s(1.0f / scale);
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    XVector3<T>& operator+=(const XVector3<T>& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    XVector3<T>& operator-=(const XVector3<T>& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    XVector3<T>& operator/=(const XVector3<T>& v)
    {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }
    XVector3<T>& operator*=(const XVector3<T>& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    bool operator!=(const XVector3<T>& v) const
    {
        return (x != v.x || y != v.y || z != v.z);
    }

    // negate
    XVector3<T> operator-() const
    {
        return XVector3<T>(-x, -y, -z);
    }


    T x, y, z;
};

typedef XVector3<double> Vec3;
typedef XVector3<double> Vector3;

// lhs scalar scale
template <typename T>
XVector3<T> operator*(T lhs, const XVector3<T>& rhs)
{
    XVector3<T> r(rhs);
    r *= lhs;
    return r;
}

template <typename T>
bool operator==(const XVector3<T>& lhs, const XVector3<T>& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

template <typename T>
typename T::value_type Dot3(const T& v1, const T& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline double Dot3(const double* v1, const double* v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}


template <typename T>
inline T Dot(const XVector3<T>& v1, const XVector3<T>& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline Vec3 Cross(const Vec3& b, const Vec3& c)
{
    return Vec3(b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x);
}

template <typename T>
inline XVector3<T> Max(const XVector3<T>& a, const XVector3<T>& b)
{
    return XVector3<T>(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}

template <typename T>
inline XVector3<T> Min(const XVector3<T>& a, const XVector3<T>& b)
{
    return XVector3<T>(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}

template <typename T>
inline XVector3<T> Abs(const XVector3<T>& a)
{
    return XVector3<T>(Abs(a.x), Abs(a.y), Abs(a.z));
}

template <typename T>
inline int LongestAxis(const XVector3<T>& v)
{
    if (v.x > v.y && v.x > v.z)
        return 0;
    else
        return (v.y > v.z) ? 1 : 2;
}

struct Bounds
{
    inline Bounds() : lower(FLT_MAX), upper(-FLT_MAX)
    {
    }

    inline Bounds(const Vec3& lower, const Vec3& upper) : lower(lower), upper(upper)
    {
    }

    inline Vec3 GetCenter() const
    {
        return 0.5 * (lower + upper);
    }

    inline Vec3 GetEdges() const
    {
        return upper - lower;
    }

    inline void Expand(double r)
    {
        lower -= Vec3(r);
        upper += Vec3(r);
    }

    inline void Expand(const Vec3& r)
    {
        lower -= r;
        upper += r;
    }

    inline bool Empty() const
    {
        return lower.x >= upper.x || lower.y >= upper.y || lower.z >= upper.z;
    }

    inline bool Overlaps(const Vec3& p) const
    {
        if (p.x < lower.x || p.y < lower.y || p.z < lower.z || p.x > upper.x || p.y > upper.y || p.z > upper.z)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    inline bool Overlaps(const Bounds& b) const
    {
        if (lower.x > b.upper.x || lower.y > b.upper.y || lower.z > b.upper.z || upper.x < b.lower.x ||
            upper.y < b.lower.y || upper.z < b.lower.z)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    Vec3 lower;
    Vec3 upper;
};

inline Bounds Union(const Bounds& a, const Vec3& b)
{
    return Bounds(Min(a.lower, b), Max(a.upper, b));
}

inline Bounds Union(const Bounds& a, const Bounds& b)
{
    return Bounds(Min(a.lower, b.lower), Max(a.upper, b.upper));
}

inline Bounds Intersection(const Bounds& a, const Bounds& b)
{
    return Bounds(Max(a.lower, b.lower), Min(a.upper, b.upper));
}

#define X 0
#define Y 1
#define Z 2

#define CROSS(dest, v1, v2)                                                                                            \
    dest[0] = v1[1] * v2[2] - v1[2] * v2[1];                                                                           \
    dest[1] = v1[2] * v2[0] - v1[0] * v2[2];                                                                           \
    dest[2] = v1[0] * v2[1] - v1[1] * v2[0];

#define DOT(v1, v2) (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2])

#define SUB(dest, v1, v2)                                                                                              \
    dest[0] = v1[0] - v2[0];                                                                                           \
    dest[1] = v1[1] - v2[1];                                                                                           \
    dest[2] = v1[2] - v2[2];

#define FINDMINMAX(x0, x1, x2, min, max)                                                                               \
    min = max = x0;                                                                                                    \
    if (x1 < min)                                                                                                      \
        min = x1;                                                                                                      \
    if (x1 > max)                                                                                                      \
        max = x1;                                                                                                      \
    if (x2 < min)                                                                                                      \
        min = x2;                                                                                                      \
    if (x2 > max)                                                                                                      \
        max = x2;


/*======================== X-tests ========================*/

#define AXISTEST_X01(a, b, fa, fb)                                                                                     \
    p0 = a * v0[Y] - b * v0[Z];                                                                                        \
    p2 = a * v2[Y] - b * v2[Z];                                                                                        \
    if (p0 < p2)                                                                                                       \
    {                                                                                                                  \
        min = p0;                                                                                                      \
        max = p2;                                                                                                      \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
        min = p2;                                                                                                      \
        max = p0;                                                                                                      \
    }                                                                                                                  \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];                                                                   \
    if (min > rad || max < -rad)                                                                                       \
        return 0;

#define AXISTEST_X2(a, b, fa, fb)                                                                                      \
    p0 = a * v0[Y] - b * v0[Z];                                                                                        \
    p1 = a * v1[Y] - b * v1[Z];                                                                                        \
    if (p0 < p1)                                                                                                       \
    {                                                                                                                  \
        min = p0;                                                                                                      \
        max = p1;                                                                                                      \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
        min = p1;                                                                                                      \
        max = p0;                                                                                                      \
    }                                                                                                                  \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];                                                                   \
    if (min > rad || max < -rad)                                                                                       \
        return 0;

/*======================== Y-tests ========================*/

#define AXISTEST_Y02(a, b, fa, fb)                                                                                     \
    p0 = -a * v0[X] + b * v0[Z];                                                                                       \
    p2 = -a * v2[X] + b * v2[Z];                                                                                       \
    if (p0 < p2)                                                                                                       \
    {                                                                                                                  \
        min = p0;                                                                                                      \
        max = p2;                                                                                                      \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
        min = p2;                                                                                                      \
        max = p0;                                                                                                      \
    }                                                                                                                  \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];                                                                   \
    if (min > rad || max < -rad)                                                                                       \
        return 0;

#define AXISTEST_Y1(a, b, fa, fb)                                                                                      \
    p0 = -a * v0[X] + b * v0[Z];                                                                                       \
    p1 = -a * v1[X] + b * v1[Z];                                                                                       \
    if (p0 < p1)                                                                                                       \
    {                                                                                                                  \
        min = p0;                                                                                                      \
        max = p1;                                                                                                      \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
        min = p1;                                                                                                      \
        max = p0;                                                                                                      \
    }                                                                                                                  \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];                                                                   \
    if (min > rad || max < -rad)                                                                                       \
        return 0;

/*======================== Z-tests ========================*/


#define AXISTEST_Z12(a, b, fa, fb)                                                                                     \
    p1 = a * v1[X] - b * v1[Y];                                                                                        \
    p2 = a * v2[X] - b * v2[Y];                                                                                        \
    if (p2 < p1)                                                                                                       \
    {                                                                                                                  \
        min = p2;                                                                                                      \
        max = p1;                                                                                                      \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
        min = p1;                                                                                                      \
        max = p2;                                                                                                      \
    }                                                                                                                  \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];                                                                   \
    if (min > rad || max < -rad)                                                                                       \
        return 0;

#define AXISTEST_Z0(a, b, fa, fb)                                                                                      \
    p0 = a * v0[X] - b * v0[Y];                                                                                        \
    p1 = a * v1[X] - b * v1[Y];                                                                                        \
    if (p0 < p1)                                                                                                       \
    {                                                                                                                  \
        min = p0;                                                                                                      \
        max = p1;                                                                                                      \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
        min = p1;                                                                                                      \
        max = p0;                                                                                                      \
    }                                                                                                                  \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];                                                                   \
    if (min > rad || max < -rad)                                                                                       \
        return 0;

int planeBoxOverlap(double normal[3], double vert[3], double maxbox[3]) // -NJMP-
{

    int q;

    double vmin[3], vmax[3], v;

    for (q = X; q <= Z; q++)
    {
        v = vert[q]; // -NJMP-

        if (normal[q] > 0.0f)
        {
            vmin[q] = -maxbox[q] - v; // -NJMP-
            vmax[q] = maxbox[q] - v; // -NJMP-
        }
        else
        {
            vmin[q] = maxbox[q] - v; // -NJMP-
            vmax[q] = -maxbox[q] - v; // -NJMP-
        }
    }

    if (DOT(normal, vmin) > 0.0f)
        return 0; // -NJMP-
    if (DOT(normal, vmax) >= 0.0f)
        return 1; // -NJMP-

    return 0;
}


int triBoxOverlap(double boxcenter[3], double boxhalfsize[3], double triverts[3][3])

{

    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
    /*       this gives 3x3=9 more tests */

    double v0[3], v1[3], v2[3];

    //   double axis[3];

    double min, max, p0, p1, p2, rad, fex, fey, fez; // -NJMP- "d" local variable removed

    double normal[3], e0[3], e1[3], e2[3];


    /* This is the fastest branch on Sun */

    /* move everything so that the boxcenter is in (0,0,0) */

    SUB(v0, triverts[0], boxcenter);

    SUB(v1, triverts[1], boxcenter);

    SUB(v2, triverts[2], boxcenter);


    /* compute triangle edges */

    SUB(e0, v1, v0); /* tri edge 0 */

    SUB(e1, v2, v1); /* tri edge 1 */

    SUB(e2, v0, v2); /* tri edge 2 */


    /* Bullet 3:  */

    /*  test the 9 tests first (this was faster) */

    fex = fabs(e0[X]);

    fey = fabs(e0[Y]);

    fez = fabs(e0[Z]);

    AXISTEST_X01(e0[Z], e0[Y], fez, fey);

    AXISTEST_Y02(e0[Z], e0[X], fez, fex);

    AXISTEST_Z12(e0[Y], e0[X], fey, fex);


    fex = fabs(e1[X]);

    fey = fabs(e1[Y]);

    fez = fabs(e1[Z]);

    AXISTEST_X01(e1[Z], e1[Y], fez, fey);

    AXISTEST_Y02(e1[Z], e1[X], fez, fex);

    AXISTEST_Z0(e1[Y], e1[X], fey, fex);


    fex = fabs(e2[X]);

    fey = fabs(e2[Y]);

    fez = fabs(e2[Z]);

    AXISTEST_X2(e2[Z], e2[Y], fez, fey);

    AXISTEST_Y1(e2[Z], e2[X], fez, fex);

    AXISTEST_Z12(e2[Y], e2[X], fey, fex);


    /* Bullet 1: */

    /*  first test overlap in the {x,y,z}-directions */

    /*  find min, max of the triangle each direction, and test for overlap in */

    /*  that direction -- this is equivalent to testing a minimal AABB around */

    /*  the triangle against the AABB */


    /* test in X-direction */

    FINDMINMAX(v0[X], v1[X], v2[X], min, max);

    if (min > boxhalfsize[X] || max < -boxhalfsize[X])
        return 0;


    /* test in Y-direction */

    FINDMINMAX(v0[Y], v1[Y], v2[Y], min, max);

    if (min > boxhalfsize[Y] || max < -boxhalfsize[Y])
        return 0;


    /* test in Z-direction */

    FINDMINMAX(v0[Z], v1[Z], v2[Z], min, max);

    if (min > boxhalfsize[Z] || max < -boxhalfsize[Z])
        return 0;


    /* Bullet 2: */

    /*  test if the box intersects the plane of the triangle */

    /*  compute plane equation of triangle: normal*x+d=0 */

    CROSS(normal, e0, e1);

    // -NJMP- (line removed here)

    if (!planeBoxOverlap(normal, v0, boxhalfsize))
        return 0; // -NJMP-


    return 1; /* box and triangle overlaps */
}


bool TriangleBoxOverlap(Vec3 lower, Vec3 upper, Vec3 a, Vec3 b, Vec3 c)
{
    Vec3 center = (lower + upper) * 0.5f;
    Vec3 halfEdges = (upper - lower) * 0.5f;

    double m[3][3];
    (Vec3&)m[0] = a;
    (Vec3&)m[1] = b;
    (Vec3&)m[2] = c;

    return triBoxOverlap((double*)&center.x, (double*)&halfEdges.x, m) ? true : false;
}

inline double minf(const double a, const double b)
{
    return a < b ? a : b;
}
inline double maxf(const double a, const double b)
{
    return a > b ? a : b;
}

inline bool IntersectRayAABBFast(const Vec3& pos, const Vector3& rcp_dir, const Vector3& min, const Vector3& max, double& t)
{

    double l1 = (min.x - pos.x) * rcp_dir.x, l2 = (max.x - pos.x) * rcp_dir.x, lmin = minf(l1, l2), lmax = maxf(l1, l2);

    l1 = (min.y - pos.y) * rcp_dir.y;
    l2 = (max.y - pos.y) * rcp_dir.y;
    lmin = maxf(minf(l1, l2), lmin);
    lmax = minf(maxf(l1, l2), lmax);

    l1 = (min.z - pos.z) * rcp_dir.z;
    l2 = (max.z - pos.z) * rcp_dir.z;
    lmin = maxf(minf(l1, l2), lmin);
    lmax = minf(maxf(l1, l2), lmax);

    // return ((lmax > 0.f) & (lmax >= lmin));
    // return ((lmax > 0.f) & (lmax > lmin));
    bool hit = ((lmax >= 0.f) & (lmax >= lmin));
    if (hit)
        t = lmin;
    return hit;
}

inline bool IntersectRayAABB(
    const Vec3& start, const Vector3& dir, const Vector3& min, const Vector3& max, double& t, Vector3* /*normal*/)
{
    //! calculate candidate plane on each axis
    double tx = -1.0f, ty = -1.0f, tz = -1.0f;
    bool inside = true;

    //! use unrolled loops

    //! x
    if (start.x < min.x)
    {
        if (dir.x != 0.0f)
            tx = (min.x - start.x) / dir.x;
        inside = false;
    }
    else if (start.x > max.x)
    {
        if (dir.x != 0.0f)
            tx = (max.x - start.x) / dir.x;
        inside = false;
    }

    //! y
    if (start.y < min.y)
    {
        if (dir.y != 0.0f)
            ty = (min.y - start.y) / dir.y;
        inside = false;
    }
    else if (start.y > max.y)
    {
        if (dir.y != 0.0f)
            ty = (max.y - start.y) / dir.y;
        inside = false;
    }

    //! z
    if (start.z < min.z)
    {
        if (dir.z != 0.0f)
            tz = (min.z - start.z) / dir.z;
        inside = false;
    }
    else if (start.z > max.z)
    {
        if (dir.z != 0.0f)
            tz = (max.z - start.z) / dir.z;
        inside = false;
    }

    //! if point inside all planes
    if (inside)
    {
        t = 0.0f;
        return true;
    }

    //! we now have t values for each of possible intersection planes
    //! find the maximum to get the intersection point
    double tmax = tx;
    int taxis = 0;

    if (ty > tmax)
    {
        tmax = ty;
        taxis = 1;
    }
    if (tz > tmax)
    {
        tmax = tz;
        taxis = 2;
    }

    if (tmax < 0.0f)
        return false;

    //! check that the intersection point lies on the plane we picked
    //! we don't test the axis of closest intersection for precision reasons

    //! no eps for now
    double eps = 0.0f;

    Vec3 hit = start + dir * tmax;

    if ((hit.x < min.x - eps || hit.x > max.x + eps) && taxis != 0)
        return false;
    if ((hit.y < min.y - eps || hit.y > max.y + eps) && taxis != 1)
        return false;
    if ((hit.z < min.z - eps || hit.z > max.z + eps) && taxis != 2)
        return false;

    //! output results
    t = tmax;

    return true;
}

// Moller and Trumbore's method
inline bool IntersectRayTriTwoSided(const Vec3& p,
                                    const Vec3& dir,
                                    const Vec3& a,
                                    const Vec3& b,
                                    const Vec3& c,
                                    double& t,
                                    double& u,
                                    double& v,
                                    double& w,
                                    double& sign,
                                    Vec3* normal)
{
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 n = Cross(ab, ac);

    double d = Dot(-dir, n);
    double ood = 1.0f / d; // No need to check for division by zero here as infinity aritmetic will save us...
    Vector3 ap = p - a;

    t = Dot(ap, n) * ood;
    if (t < 0.0f)
        return false;

    Vector3 e = Cross(-dir, ap);
    v = Dot(ac, e) * ood;
    if (v < 0.0f || v > 1.0f) // ...here...
        return false;
    w = -Dot(ab, e) * ood;
    if (w < 0.0f || v + w > 1.0f) // ...and here
        return false;

    u = 1.0f - v - w;
    if (normal)
        *normal = n;

    sign = d;

    return true;
}

inline Vec3 ClosestPointToAABB(const Vec3& p, const Vec3& lower, const Vec3& upper)
{
    Vec3 c;

    for (int i = 0; i < 3; ++i)
    {
        double v = p[i];
        if (v < lower[i])
            v = lower[i];
        if (v > upper[i])
            v = upper[i];
        c[i] = v;
    }

    return c;
}

inline double DistanceToAABB(const Vec3& p, const Vec3& lower, const Vec3& upper)
{
    Vec3 cp = ClosestPointToAABB(p, lower, upper);

    return Length(p - cp);
}

// RTCD 5.1.5, page 142
inline Vec3 ClosestPointOnTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p, double& v, double& w)
{
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ap = p - a;

    double d1 = Dot(ab, ap);
    double d2 = Dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        v = 0.0f;
        w = 0.0f;
        return a;
    }

    Vec3 bp = p - b;
    double d3 = Dot(ab, bp);
    double d4 = Dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
    {
        v = 1.0f;
        w = 0.0f;
        return b;
    }

    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        v = d1 / (d1 - d3);
        w = 0.0f;
        return a + v * ab;
    }

    Vec3 cp = p - c;
    double d5 = Dot(ab, cp);
    double d6 = Dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
    {
        v = 0.0f;
        w = 1.0f;
        return c;
    }

    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        v = 0.0f;
        w = d2 / (d2 - d6);
        return a + w * ac;
    }

    double va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        v = 1.0f - w;
        return b + w * (c - b);
    }

    double denom = 1.0f / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    return a + ab * v + ac * w;
}


class AABBTreeImpl : public AABBTree
{
public:
    AABBTreeImpl(const AABBTreeImpl&);
    AABBTreeImpl& operator=(const AABBTreeImpl&);

    AABBTreeImpl(const Vec3* vertices, uint32_t numVerts, const uint32_t* indices, uint32_t numFaces);

    bool TraceRay(const Vec3& start,
                  const Vector3& dir,
                  double& outT,
                  double& u,
                  double& v,
                  double& w,
                  double& faceSign,
                  uint32_t& faceIndex) const;


    Vector3 GetCenter() const
    {
        return (m_nodes[0].m_minExtents + m_nodes[0].m_maxExtents) * 0.5f;
    }
    Vector3 GetMinExtents() const
    {
        return m_nodes[0].m_minExtents;
    }
    Vector3 GetMaxExtents() const
    {
        return m_nodes[0].m_maxExtents;
    }

    virtual bool raycast(const double* start,
                         const double* dir,
                         double& outT,
                         double& u,
                         double& v,
                         double& w,
                         double& faceSign,
                         uint32_t& faceIndex) const final
    {
        bool ret = TraceRay(*(const Vec3*)start, *(const Vec3*)dir, outT, u, v, w, faceSign, faceIndex);
        return ret;
    }

    virtual void release(void) final
    {
        delete this;
    }

private:
    struct Node
    {
        Node() : m_numFaces(0), m_faces(NULL), m_minExtents(0.0f), m_maxExtents(0.0f)
        {
        }

        union
        {
            uint32_t m_children;
            uint32_t m_numFaces;
        };

        uint32_t* m_faces;
        Vector3 m_minExtents;
        Vector3 m_maxExtents;
    };


    struct BoundsAABB
    {
        BoundsAABB() : m_min(0.0f), m_max(0.0f)
        {
        }

        BoundsAABB(const Vector3& min, const Vector3& max) : m_min(min), m_max(max)
        {
        }

        inline double GetVolume() const
        {
            Vector3 e = m_max - m_min;
            return (e.x * e.y * e.z);
        }

        inline double GetSurfaceArea() const
        {
            Vector3 e = m_max - m_min;
            return 2.0f * (e.x * e.y + e.x * e.z + e.y * e.z);
        }

        inline void Union(const BoundsAABB& b)
        {
            m_min = Min(m_min, b.m_min);
            m_max = Max(m_max, b.m_max);
        }

        Vector3 m_min;
        Vector3 m_max;
    };

    typedef std::vector<uint32_t> IndexArray;
    typedef std::vector<Vec3> PositionArray;
    typedef std::vector<Node> NodeArray;
    typedef std::vector<uint32_t> FaceArray;
    typedef std::vector<BoundsAABB> FaceBoundsArray;

    // partition the objects and return the number of objects in the lower partition
    uint32_t PartitionMedian(Node& n, uint32_t* faces, uint32_t numFaces);
    uint32_t PartitionSAH(Node& n, uint32_t* faces, uint32_t numFaces);

    void Build();

    void BuildRecursive(uint32_t nodeIndex, uint32_t* faces, uint32_t numFaces);

    void TraceRecursive(uint32_t nodeIndex,
                        const Vec3& start,
                        const Vector3& dir,
                        double& outT,
                        double& u,
                        double& v,
                        double& w,
                        double& faceSign,
                        uint32_t& faceIndex) const;


    bool GetClosestPointWithinDistance(
        const Vec3& point, const double maxDis, double& dis, double& v, double& w, uint32_t& faceIndex, Vec3& closest) const;

    virtual bool getClosestPointWithinDistance(const double* point, double maxDistance, double* closestPoint) final
    {
        double dis, v, w;
        uint32_t faceIndex;
        bool hit =
            GetClosestPointWithinDistance(*(const Vec3*)point, maxDistance, dis, v, w, faceIndex, *(Vec3*)closestPoint);
        return hit;
    }

    void GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                  const Vec3& point,
                                                  double& outDisSq,
                                                  double& outV,
                                                  double& outW,
                                                  uint32_t& outFaceIndex,
                                                  Vec3& closest) const;

    void CalculateFaceBounds(uint32_t* faces, uint32_t numFaces, Vector3& outMinExtents, Vector3& outMaxExtents);

    uint32_t GetNumFaces() const
    {
        return m_numFaces;
    }

    uint32_t GetNumNodes() const
    {
        return uint32_t(m_nodes.size());
    }

    // track the next free node
    uint32_t m_freeNode;

    const Vec3* m_vertices;
    const uint32_t m_numVerts;

    const uint32_t* m_indices;
    const uint32_t m_numFaces;

    FaceArray m_faces;
    NodeArray m_nodes;
    FaceBoundsArray m_faceBounds;

    // stats
    uint32_t m_treeDepth;
    uint32_t m_innerNodes;
    uint32_t m_leafNodes;

    uint32_t s_depth{0};
};


AABBTreeImpl::AABBTreeImpl(const Vec3* vertices, uint32_t numVerts, const uint32_t* indices, uint32_t numFaces)
    : m_vertices(vertices), m_numVerts(numVerts), m_indices(indices), m_numFaces(numFaces)
{
    // build stats
    m_treeDepth = 0;
    m_innerNodes = 0;
    m_leafNodes = 0;

    Build();
}

namespace
{

struct FaceSorter
{
    FaceSorter(const Vec3* positions, const uint32_t* indices, uint32_t n, uint32_t axis)
        : m_vertices(positions), m_indices(indices), m_numIndices(n), m_axis(axis)
    {
    }

    inline bool operator()(uint32_t lhs, uint32_t rhs) const
    {
        double a = GetCentroid(lhs);
        double b = GetCentroid(rhs);

        if (a == b)
            return lhs < rhs;
        else
            return a < b;
    }

    inline double GetCentroid(uint32_t face) const
    {
        const Vec3& a = m_vertices[m_indices[face * 3 + 0]];
        const Vec3& b = m_vertices[m_indices[face * 3 + 1]];
        const Vec3& c = m_vertices[m_indices[face * 3 + 2]];

        return (a[m_axis] + b[m_axis] + c[m_axis]) / 3.0f;
    }

    const Vec3* m_vertices;
    const uint32_t* m_indices;
    uint32_t m_numIndices;
    uint32_t m_axis;
};


} // anonymous namespace

void AABBTreeImpl::CalculateFaceBounds(uint32_t* faces, uint32_t numFaces, Vector3& outMinExtents, Vector3& outMaxExtents)
{
    Vector3 minExtents(FLT_MAX);
    Vector3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        Vector3 a = Vector3(m_vertices[m_indices[faces[i] * 3 + 0]]);
        Vector3 b = Vector3(m_vertices[m_indices[faces[i] * 3 + 1]]);
        Vector3 c = Vector3(m_vertices[m_indices[faces[i] * 3 + 2]]);

        minExtents = Min(a, minExtents);
        maxExtents = Max(a, maxExtents);

        minExtents = Min(b, minExtents);
        maxExtents = Max(b, maxExtents);

        minExtents = Min(c, minExtents);
        maxExtents = Max(c, maxExtents);
    }

    outMinExtents = minExtents;
    outMaxExtents = maxExtents;
}

void AABBTreeImpl::Build()
{
    assert(m_numFaces);

    // const double startTime = GetSeconds();

    const uint32_t numFaces = m_numFaces;

    // build initial list of faces
    m_faces.reserve(numFaces);

    // calculate bounds of each face and store
    m_faceBounds.reserve(numFaces);

    std::vector<BoundsAABB> stack;
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        BoundsAABB top;
        CalculateFaceBounds(&i, 1, top.m_min, top.m_max);

        m_faces.push_back(i);
        m_faceBounds.push_back(top);
    }

    m_nodes.reserve(uint32_t(numFaces * 1.5f));

    // allocate space for all the nodes
    m_freeNode = 1;

    // start building
    BuildRecursive(0, &m_faces[0], numFaces);

    assert(s_depth == 0);
}

// partion faces around the median face
uint32_t AABBTreeImpl::PartitionMedian(Node& n, uint32_t* faces, uint32_t numFaces)
{
    FaceSorter predicate(&m_vertices[0], &m_indices[0], m_numFaces * 3, LongestAxis(n.m_maxExtents - n.m_minExtents));
    std::nth_element(faces, faces + numFaces / 2, faces + numFaces, predicate);

    return numFaces / 2;
}

// partion faces based on the surface area heuristic
uint32_t AABBTreeImpl::PartitionSAH(Node& n, uint32_t* faces, uint32_t numFaces)
{
    QH_UNUSED(n);
    uint32_t bestAxis = 0;
    uint32_t bestIndex = 0;
    double bestCost = FLT_MAX;

    for (uint32_t a = 0; a < 3; ++a)
    {
        // sort faces by centroids
        FaceSorter predicate(&m_vertices[0], &m_indices[0], m_numFaces * 3, a);
        std::sort(faces, faces + numFaces, predicate);

        // two passes over data to calculate upper and lower bounds
        vector<double> cumulativeLower(numFaces);
        vector<double> cumulativeUpper(numFaces);

        BoundsAABB lower;
        BoundsAABB upper;

        for (uint32_t i = 0; i < numFaces; ++i)
        {
            lower.Union(m_faceBounds[faces[i]]);
            upper.Union(m_faceBounds[faces[numFaces - i - 1]]);

            cumulativeLower[i] = lower.GetSurfaceArea();
            cumulativeUpper[numFaces - i - 1] = upper.GetSurfaceArea();
        }

        double invTotalSA = 1.0f / cumulativeUpper[0];

        // test all split positions
        for (uint32_t i = 0; i < numFaces - 1; ++i)
        {
            double pBelow = cumulativeLower[i] * invTotalSA;
            double pAbove = cumulativeUpper[i] * invTotalSA;

            double cost = 0.125f + (pBelow * i + pAbove * (numFaces - i));
            if (cost <= bestCost)
            {
                bestCost = cost;
                bestIndex = i;
                bestAxis = a;
            }
        }
    }

    // re-sort by best axis
    FaceSorter predicate(&m_vertices[0], &m_indices[0], m_numFaces * 3, bestAxis);
    std::sort(faces, faces + numFaces, predicate);

    return bestIndex + 1;
}

void AABBTreeImpl::BuildRecursive(uint32_t nodeIndex, uint32_t* faces, uint32_t numFaces)
{
    const uint32_t kMaxFacesPerLeaf = 6;

    // if we've run out of nodes allocate some more
    if (nodeIndex >= m_nodes.size())
    {
        uint32_t s = std::max(uint32_t(1.5f * m_nodes.size()), 512U);

        // cout << "Resizing tree, current size: " << m_nodes.size()*sizeof(Node) << " new size: " << s*sizeof(Node) <<
        // endl;

        m_nodes.resize(s);
    }

    // a reference to the current node, need to be careful here as this reference may become invalid if array is resized
    Node& n = m_nodes[nodeIndex];

    // track max tree depth
    ++s_depth;
    m_treeDepth = max(m_treeDepth, s_depth);

    CalculateFaceBounds(faces, numFaces, n.m_minExtents, n.m_maxExtents);

    // calculate bounds of faces and add node
    if (numFaces <= kMaxFacesPerLeaf)
    {
        n.m_faces = faces;
        n.m_numFaces = numFaces;

        ++m_leafNodes;
    }
    else
    {
        ++m_innerNodes;

        // face counts for each branch
        const uint32_t leftCount = PartitionMedian(n, faces, numFaces);
        // const uint32_t leftCount = PartitionSAH(n, faces, numFaces);
        const uint32_t rightCount = numFaces - leftCount;

        // alloc 2 nodes
        m_nodes[nodeIndex].m_children = m_freeNode;

        // allocate two nodes
        m_freeNode += 2;

        // split faces in half and build each side recursively
        BuildRecursive(m_nodes[nodeIndex].m_children + 0, faces, leftCount);
        BuildRecursive(m_nodes[nodeIndex].m_children + 1, faces + leftCount, rightCount);
    }

    --s_depth;
}

struct StackEntry
{
    uint32_t m_node;
    double m_dist;
};


#if 0
void AABBTreeImpl::TraceBounds(const Vec3& lower, const Vector3& upper, std::vector<int>& triangles) const
{
    TraceBoundsRecursive(0, lower, upper, triangles);
}

void AABBTreeImpl::TraceBoundsRecursive(uint32_t nodeIndex,
                                    const Vec3& lower,
                                    const Vector3& upper,
                                    std::vector<int>& triangles) const
{
    const Node& node = m_nodes[nodeIndex];

    Bounds bounds(lower, upper);

    if (bounds.Overlaps(Bounds(node.m_minExtents, node.m_maxExtents)))
    {
        if (node.m_faces)
        {
            for (int i = 0; i < int(node.m_numFaces); ++i)
            {
                Vec3 a = m_vertices[m_indices[node.m_faces[i] * 3 + 0]];
                Vec3 b = m_vertices[m_indices[node.m_faces[i] * 3 + 1]];
                Vec3 c = m_vertices[m_indices[node.m_faces[i] * 3 + 2]];

                if (TriangleBoxOverlap(lower, upper, a, b, c))
                {
                    triangles.push_back(node.m_faces[i]);
                }
            }
        }
        else
        {
            TraceBoundsRecursive(node.m_children + 0, lower, upper, triangles);
            TraceBoundsRecursive(node.m_children + 1, lower, upper, triangles);
        }
    }
}
#endif

bool AABBTreeImpl::TraceRay(const Vec3& start,
                            const Vector3& dir,
                            double& outT,
                            double& u,
                            double& v,
                            double& w,
                            double& faceSign,
                            uint32_t& faceIndex) const
{
    outT = FLT_MAX;
    TraceRecursive(0, start, dir, outT, u, v, w, faceSign, faceIndex);
    return (outT != FLT_MAX);
}

void AABBTreeImpl::TraceRecursive(uint32_t nodeIndex,
                                  const Vec3& start,
                                  const Vector3& dir,
                                  double& outT,
                                  double& outU,
                                  double& outV,
                                  double& outW,
                                  double& faceSign,
                                  uint32_t& faceIndex) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == NULL)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        double dist[2] = { FLT_MAX, FLT_MAX };

        IntersectRayAABB(start, dir, leftChild.m_minExtents, leftChild.m_maxExtents, dist[0], NULL);
        IntersectRayAABB(start, dir, rightChild.m_minExtents, rightChild.m_maxExtents, dist[1], NULL);

        uint32_t closest = 0;
        uint32_t furthest = 1;

        if (dist[1] < dist[0])
        {
            closest = 1;
            furthest = 0;
        }

        if (dist[closest] < outT)
            TraceRecursive(node.m_children + closest, start, dir, outT, outU, outV, outW, faceSign, faceIndex);

        if (dist[furthest] < outT)
            TraceRecursive(node.m_children + furthest, start, dir, outT, outU, outV, outW, faceSign, faceIndex);
    }
    else
    {
        Vector3 normal;
        double t, u, v, w, s;

        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i] * 3;

            const Vec3& a = m_vertices[m_indices[indexStart + 0]];
            const Vec3& b = m_vertices[m_indices[indexStart + 1]];
            const Vec3& c = m_vertices[m_indices[indexStart + 2]];
            if (IntersectRayTriTwoSided(start, dir, a, b, c, t, u, v, w, s, NULL))
            {
                if (t < outT)
                {
                    outT = t;
                    outU = u;
                    outV = v;
                    outW = w;
                    faceSign = s;
                    faceIndex = node.m_faces[i];
                }
            }
        }
    }
}

bool AABBTreeImpl::GetClosestPointWithinDistance(
    const Vec3& point, const double maxDis, double& dis, double& v, double& w, uint32_t& faceIndex, Vec3& closest) const
{
    dis = maxDis;
    faceIndex = uint32_t(~0);
    double disSq = dis * dis;

    GetClosestPointWithinDistanceSqRecursive(0, point, disSq, v, w, faceIndex, closest);
    dis = sqrt(disSq);

    return (faceIndex < (~((unsigned int)0)));
}

void AABBTreeImpl::GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                            const Vec3& point,
                                                            double& outDisSq,
                                                            double& outV,
                                                            double& outW,
                                                            uint32_t& outFaceIndex,
                                                            Vec3& closestPoint) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == NULL)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        // double dist[2] = { FLT_MAX, FLT_MAX };
        Vec3 lp = ClosestPointToAABB(point, leftChild.m_minExtents, leftChild.m_maxExtents);
        Vec3 rp = ClosestPointToAABB(point, rightChild.m_minExtents, rightChild.m_maxExtents);


        uint32_t closest = 0;
        uint32_t furthest = 1;
        double dcSq = LengthSq(point - lp);
        double dfSq = LengthSq(point - rp);
        if (dfSq < dcSq)
        {
            closest = 1;
            furthest = 0;
            swap(dfSq, dcSq);
        }

        if (dcSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(
                node.m_children + closest, point, outDisSq, outV, outW, outFaceIndex, closestPoint);
        }

        if (dfSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(
                node.m_children + furthest, point, outDisSq, outV, outW, outFaceIndex, closestPoint);
        }
    }
    else
    {

        double v, w;
        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i] * 3;

            const Vec3& a = m_vertices[m_indices[indexStart + 0]];
            const Vec3& b = m_vertices[m_indices[indexStart + 1]];
            const Vec3& c = m_vertices[m_indices[indexStart + 2]];

            Vec3 cp = ClosestPointOnTriangle(a, b, c, point, v, w);
            double disSq = LengthSq(cp - point);

            if (disSq < outDisSq)
            {
                closestPoint = cp;
                outDisSq = disSq;
                outV = v;
                outW = w;
                outFaceIndex = node.m_faces[i];
            }
        }
    }
}

AABBTree* AABBTree::create(const double* vertices, uint32_t numVerts, const uint32_t* indices, uint32_t numFaces)
{
    auto ret = new AABBTreeImpl((const Vec3*)vertices, numVerts, indices, numFaces);
    return static_cast<AABBTree*>(ret);
}


} // namespace aabbtree

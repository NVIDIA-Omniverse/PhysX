// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTEXTAUTHORINGVSA_H
#define NVBLASTEXTAUTHORINGVSA_H

namespace Nv
{
namespace Blast
{

/*
    This code copied from APEX GSA
*/

namespace VSA
{
typedef float real;

struct VS3D_Halfspace_Set
{
    virtual real    farthest_halfspace(real plane[4], const real point[4]) = 0;
};


// Simple types and operations for internal calculations
struct Vec3 { real x, y, z; };  // 3-vector 
inline Vec3 vec3(real x, real y, real z) { Vec3 r; r.x = x; r.y = y; r.z = z; return r; }   // vector builder
inline Vec3 operator +  (const Vec3& a, const Vec3& b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }            // vector addition
inline Vec3 operator *  (real s, const Vec3& v) { return vec3(s*v.x, s*v.y, s*v.z); }               // scalar multiplication
inline real operator |  (const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }              // dot product
inline Vec3 operator ^  (const Vec3& a, const Vec3& b) { return vec3(a.y*b.z - b.y*a.z, a.z*b.x - b.z*a.x, a.x*b.y - b.x*a.y); }    // cross product

struct Vec4 { Vec3 v; real w; };    // 4-vector split into 3-vector and scalar parts
inline Vec4 vec4(const Vec3& v, real w) { Vec4 r; r.v = v; r.w = w; return r; } // vector builder
inline real operator | (const Vec4& a, const Vec4& b) { return (a.v | b.v) + a.w*b.w; }         // dot product

// More accurate perpendicular
inline Vec3 perp(const Vec3& a, const Vec3& b)
{
    Vec3 c = a^b;   // Cross-product gives perpendicular
#if VS3D_HIGH_ACCURACY || REAL_DOUBLE
    const real c2 = c | c;
    if (c2 != 0) c = c + (1 / c2)*((a | c)*(c^b) + (b | c)*(a^c));  // Improvement to (a b)^T(c) = (0)
#endif
    return c;
}

// Square
inline real sq(real x) { return x*x; }

// Returns index of the extremal element in a three-element set {e0, e1, e2} based upon comparisons c_ij. The extremal index m is such that c_mn is true, or e_m == e_n, for all n.
inline int  ext_index(int c_10, int c_21, int c_20) { return c_10 << c_21 | (c_21&c_20) << 1; }

// Returns index (0, 1, or 2) of minimum argument
inline int  index_of_min(real x0, real x1, real x2) { return ext_index((int)(x1 < x0), (int)(x2 < x1), (int)(x2 < x0)); }

// Compare fractions with positive deominators.  Returns a_num*sqrt(a_rden2) > b_num*sqrt(b_rden2)
inline bool frac_gt(real a_num, real a_rden2, real b_num, real b_rden2)
{
    const bool a_num_neg = a_num < 0;
    const bool b_num_neg = b_num < 0;
    return a_num_neg != b_num_neg ? b_num_neg : ((a_num*a_num*a_rden2 > b_num*b_num*b_rden2) != a_num_neg);
}

// Returns index (0, 1, or 2) of maximum fraction with positive deominators
inline int  index_of_max_frac(real x0_num, real x0_rden2, real x1_num, real x1_rden2, real x2_num, real x2_rden2)
{
    return ext_index((int)frac_gt(x1_num, x1_rden2, x0_num, x0_rden2), (int)frac_gt(x2_num, x2_rden2, x1_num, x1_rden2), (int)frac_gt(x2_num, x2_rden2, x0_num, x0_rden2));
}

// Compare values given their signs and squares.  Returns a > b.  a2 and b2 may have any constant offset applied to them.
inline bool sgn_sq_gt(real sgn_a, real a2, real sgn_b, real b2) { return sgn_a*sgn_b < 0 ? (sgn_b < 0) : ((a2 > b2) != (sgn_a < 0)); }

// Returns index (0, 1, or 2) of maximum value given their signs and squares.  sq_x0, sq_x1, and sq_x2 may have any constant offset applied to them.
inline int  index_of_max_sgn_sq(real sgn_x0, real sq_x0, real sgn_x1, real sq_x1, real sgn_x2, real sq_x2)
{
    return ext_index((int)sgn_sq_gt(sgn_x1, sq_x1, sgn_x0, sq_x0), (int)sgn_sq_gt(sgn_x2, sq_x2, sgn_x1, sq_x1), (int)sgn_sq_gt(sgn_x2, sq_x2, sgn_x0, sq_x0));
}

// Project 2D (homogeneous) vector onto 2D half-space boundary
inline void project2D(Vec3& r, const Vec3& plane, real delta, real recip_n2, real eps2)
{
    r = r + (-delta*recip_n2)*vec3(plane.x, plane.y, 0);
    r = r + (-(r | plane)*recip_n2)*vec3(plane.x, plane.y, 0);  // Second projection for increased accuracy
    if ((r | r) > eps2) return;
    r = (-plane.z*recip_n2)*vec3(plane.x, plane.y, 0);
    r.z = 1;
}


// Update function for vs3d_test
static bool vs3d_update(Vec4& p, Vec4 S[4], int& plane_count, const Vec4& q, real eps2)
{
    // h plane is the last plane
    const Vec4& h = S[plane_count - 1];

    // Handle plane_count == 1 specially (optimization; this could be commented out)
    if (plane_count == 1)
    {
        // Solution is objective projected onto h plane
        p = q;
        p.v = p.v + -(p | h)*h.v;
        if ((p | p) <= eps2) p = vec4(-h.w*h.v, 1); // If p == 0 then q is a direction vector, any point in h is a support point
        return true;
    }

    // Create basis in the h plane
    const int min_i = index_of_min(h.v.x*h.v.x, h.v.y*h.v.y, h.v.z*h.v.z);
    const Vec3 y = h.v^vec3((real)(min_i == 0), (real)(min_i == 1), (real)(min_i == 2));
    const Vec3 x = y^h.v;

    // Use reduced vector r instead of p
    Vec3 r = { x | q.v, y | q.v, q.w*(y | y) }; // (x|x) = (y|y) = square of plane basis scale

    // If r == 0 (within epsilon), then it is a direction vector, and we have a bounded solution
    if ((r | r) <= eps2) r.z = 1;

    // Create plane equations in the h plane.  These will not be normalized in general.
    int N = 0;          // Plane count in h subspace
    Vec3 R[3];          // Planes in h subspace
    real recip_n2[3];   // Plane normal vector reciprocal lengths squared
    real delta[3];      // Signed distance of objective to the planes
    int index[3];       // Keep track of original plane indices
    for (int i = 0; i < plane_count - 1; ++i)
    {
        const Vec3& vi = S[i].v;
        const real cos_theta = h.v | vi;
        R[N] = vec3(x | vi, y | vi, S[i].w - h.w*cos_theta);
        index[N] = i;
        const real n2 = R[N].x*R[N].x + R[N].y*R[N].y;
        if (n2 >= eps2)
        {
            const real lin_norm = (real)1.5 - (real)0.5*n2; // 1st-order approximation to 1/sqrt(n2) expanded about n2 = 1
            R[N] = lin_norm*R[N];   // We don't need normalized plane equations, but rescaling (even with an approximate normalization) gives better numerical behavior
            recip_n2[N] = 1 / (R[N].x*R[N].x + R[N].y*R[N].y);
            delta[N] = r | R[N];
            ++N;    // Keep this plane
        }
        else if (cos_theta < 0) return false;   // Parallel cases are redundant and rejected, anti-parallel cases are 1D voids
    }

    // Now work with the N-sized R array of half-spaces in the h plane
    switch (N)
    {
    case 1: one_plane :
        if (delta[0] < 0) N = 0;    // S[0] is redundant, eliminate it
        else project2D(r, R[0], delta[0], recip_n2[0], eps2);
        break;
    case 2: two_planes :
        if (delta[0] < 0 && delta[1] < 0) N = 0;    // S[0] and S[1] are redundant, eliminate them
        else
        {
            const int max_d_index = (int)frac_gt(delta[1], recip_n2[1], delta[0], recip_n2[0]);
            project2D(r, R[max_d_index], delta[max_d_index], recip_n2[max_d_index], eps2);
            const int min_d_index = max_d_index ^ 1;
            const real new_delta_min = r | R[min_d_index];
            if (new_delta_min < 0)
            {
                index[0] = index[max_d_index];
                N = 1;  // S[min_d_index] is redundant, eliminate it
            }
            else
            {
                // Set r to the intersection of R[0] and R[1] and keep both
                r = perp(R[0], R[1]);
                if (r.z*r.z*recip_n2[0] * recip_n2[1] < eps2)
                {
                    if (R[0].x*R[1].x + R[0].y*R[1].y < 0) return false;    // 2D void found
                    goto one_plane;
                }
                r = (1 / r.z)*r;    // We could just as well multiply r by sgn(r.z); we just need to ensure r.z > 0
            }
        }
        break;
    case 3:
        if (delta[0] < 0 && delta[1] < 0 && delta[2] < 0) N = 0;    // S[0], S[1], and S[2] are redundant, eliminate them
        else
        {
            const Vec3 row_x = { R[0].x, R[1].x, R[2].x };
            const Vec3 row_y = { R[0].y, R[1].y, R[2].y };
            const Vec3 row_w = { R[0].z, R[1].z, R[2].z };
            const Vec3 cof_w = perp(row_x, row_y);
            const bool detR_pos = (row_w | cof_w) > 0;
            const int nrw_sgn0 = cof_w.x*cof_w.x*recip_n2[1] * recip_n2[2] < eps2 ? 0 : (((int)((cof_w.x > 0) == detR_pos) << 1) - 1);
            const int nrw_sgn1 = cof_w.y*cof_w.y*recip_n2[2] * recip_n2[0] < eps2 ? 0 : (((int)((cof_w.y > 0) == detR_pos) << 1) - 1);
            const int nrw_sgn2 = cof_w.z*cof_w.z*recip_n2[0] * recip_n2[1] < eps2 ? 0 : (((int)((cof_w.z > 0) == detR_pos) << 1) - 1);

            if ((nrw_sgn0 | nrw_sgn1 | nrw_sgn2) >= 0) return false;    // 3D void found

            const int positive_width_count = ((nrw_sgn0 >> 1) & 1) + ((nrw_sgn1 >> 1) & 1) + ((nrw_sgn2 >> 1) & 1);
            if (positive_width_count == 1)
            {
                // A single positive width results from a redundant plane.  Eliminate it and peform N = 2 calculation.
                const int pos_width_index = ((nrw_sgn1 >> 1) & 1) | (nrw_sgn2 & 2); // Calculates which index corresponds to the positive-width side
                R[pos_width_index] = R[2];
                recip_n2[pos_width_index] = recip_n2[2];
                delta[pos_width_index] = delta[2];
                index[pos_width_index] = index[2];
                N = 2;
                goto two_planes;
            }

            // Find the max dot product of r and R[i]/|R_normal[i]|.  For numerical accuracy when the angle between r and the i^{th} plane normal is small, we take some care below:
            const int max_d_index = r.z != 0
                ? index_of_max_frac(delta[0], recip_n2[0], delta[1], recip_n2[1], delta[2], recip_n2[2])    // displacement term resolves small-angle ambiguity, just use dot product
                : index_of_max_sgn_sq(delta[0], -sq(r.x*R[0].y - r.y*R[0].x)*recip_n2[0], delta[1], -sq(r.x*R[1].y - r.y*R[1].x)*recip_n2[1], delta[2], -sq(r.x*R[2].y - r.y*R[2].x)*recip_n2[2]);  // No displacement term.  Use wedge product to find the sine of the angle.

            // Project r onto max-d plane
            project2D(r, R[max_d_index], delta[max_d_index], recip_n2[max_d_index], eps2);
            N = 1;  // Unless we use a vertex in the loop below
            const int index_max = index[max_d_index];

            // The number of finite widths should be >= 2.  If not, it should be 0, but in any case it implies three parallel lines in the plane, which we should not have here.
            // If we do have three parallel lines (# of finite widths < 2), we've picked the line corresponding to the half-plane farthest from r, which is correct.
            const int finite_width_count = (nrw_sgn0 & 1) + (nrw_sgn1 & 1) + (nrw_sgn2 & 1);
            if (finite_width_count >= 2)
            {
                const int i_remaining[2] = { (1 << max_d_index) & 3, (3 >> max_d_index) ^ 1 };  // = {(max_d_index+1)%3, (max_d_index+2)%3}
                const int i_select = (int)frac_gt(delta[i_remaining[1]], recip_n2[i_remaining[1]], delta[i_remaining[0]], recip_n2[i_remaining[0]]);    // Select the greater of the remaining dot products
                for (int i = 0; i < 2; ++i)
                {
                    const int j = i_remaining[i_select^i];  // i = 0 => the next-greatest, i = 1 => the least
                    if ((r | R[j]) >= 0)
                    {
                        r = perp(R[max_d_index], R[j]);
                        r = (1 / r.z)*r;    // We could just as well multiply r by sgn(r.z); we just need to ensure r.z > 0
                        index[1] = index[j];
                        N = 2;
                        break;
                    }
                }
            }

            index[0] = index_max;
        }
        break;
    }

    // Transform r back to 3D space
    p = vec4(r.x*x + r.y*y + (-r.z*h.w)*h.v, r.z);

    // Pack S array with kept planes
    if (N < 2 || index[1] != 0) { for (int i = 0; i < N; ++i) S[i] = S[index[i]]; } // Safe to copy columns in order
    else { const Vec4 temp = S[0]; S[0] = S[index[0]]; S[1] = temp; }   // Otherwise use temp storage to avoid overwrite
    S[N] = h;
    plane_count = N + 1;

    return true;
}


// Performs the VS algorithm for D = 3
inline int vs3d_test(VS3D_Halfspace_Set& halfspace_set, real* q = nullptr)
{
    // Objective = q if it is not NULL, otherwise it is the origin represented in homogeneous coordinates
    const Vec4 objective = q ? (q[3] != 0 ? vec4((1 / q[3])*vec3(q[0], q[1], q[2]), 1) : *(Vec4*)q) : vec4(vec3(0, 0, 0), 1);

    // Tolerance for 3D void simplex algorithm
    const real eps_f = (real)1 / (sizeof(real) == 4 ? (1L << 23) : (1LL << 52));    // Floating-point epsilon
#if VS3D_HIGH_ACCURACY || REAL_DOUBLE
    const real eps = 8 * eps_f;
#else
    const real eps = 80 * eps_f;
#endif
    const real eps2 = eps*eps;  // Using epsilon squared

    // Maximum allowed iterations of main loop.  If exceeded, error code is returned
    const int max_iteration_count = 50;

    // State
    Vec4 S[4];              // Up to 4 planes
    int plane_count = 0;    // Number of valid planes
    Vec4 p = objective;     // Test point, initialized to objective

    // Default result, changed to valid result if found in loop below
    int result = -1;

    // Iterate until a stopping condition is met or the maximum number of iterations is reached
    for (int i = 0; result < 0 && i < max_iteration_count; ++i)
    {
        Vec4& plane = S[plane_count++];
        real delta = halfspace_set.farthest_halfspace(&plane.v.x, &p.v.x);
#if VS3D_UNNORMALIZED_PLANE_HANDLING != 0
        const real recip_norm = vs3d_recip_sqrt(plane.v | plane.v);
        plane = vec4(recip_norm*plane.v, recip_norm*plane.w);
        delta *= recip_norm;
#endif
        if (delta <= 0 || delta*delta <= eps2*(p | p)) result = 1;  // Intersection found
        else if (!vs3d_update(p, S, plane_count, objective, eps2)) result = 0;  // Void simplex found
    }

    // If q is given, fill it with the solution (normalize p.w if it is not zero)
    if (q) *(Vec4*)q = (p.w != 0) ? vec4((1 / p.w)*p.v, 1) : p;

    return result;
}

} // namespace VSA

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTEXTAUTHORINGVSA_H

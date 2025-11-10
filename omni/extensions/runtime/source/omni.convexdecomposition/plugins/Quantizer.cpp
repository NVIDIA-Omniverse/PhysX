// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <vector>
#include <assert.h>

#include "Quantizer.h"

///////////////////////////////////////////////////////////////////////
//	    C Implementation of Wu's Color Quantizer (v. 2)
//	    (see Graphics Gems vol. II, pp. 126-133)
//
// Author:	Xiaolin Wu
// Dept. of Computer Science
// Univ. of Western Ontario
// London, Ontario N6A 5B7
// wu@csd.uwo.ca
// 
// Algorithm: Greedy orthogonal bipartition of RGB space for variance
// 	   minimization aided by inclusion-exclusion tricks.
// 	   For speed no nearest neighbor search is done. Slightly
// 	   better performance can be expected by more sophisticated
// 	   but more expensive versions.
// 
// The author thanks Tom Lane at Tom_Lane@G.GP.CS.CMU.EDU for much of
// additional documentation and a cure to a previous bug.
// 
// Free to distribute, comments and suggestions are appreciated.
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
// History
// -------
// July 2000:  C++ Implementation of Wu's Color Quantizer
//             and adaptation for the FreeImage 2 Library
//             Author: Herve Drolon (drolon@infonie.fr)
// March 2004: Adaptation for the FreeImage 3 library (port to big endian processors)
//             Author: Herve Drolon (drolon@infonie.fr)
///////////////////////////////////////////////////////////////////////

namespace quickhull
{

#define FI_RGBA_RED				2
#define FI_RGBA_GREEN			1
#define FI_RGBA_BLUE			0
#define FI_RGBA_ALPHA			3
#define FI_RGBA_RED_MASK		0x00FF0000
#define FI_RGBA_GREEN_MASK		0x0000FF00
#define FI_RGBA_BLUE_MASK		0x000000FF
#define FI_RGBA_ALPHA_MASK		0xFF000000
#define FI_RGBA_RED_SHIFT		16
#define FI_RGBA_GREEN_SHIFT		8
#define FI_RGBA_BLUE_SHIFT		0
#define FI_RGBA_ALPHA_SHIFT		24

////////////////////////////////////////////////////////////////
class WuVec3
{
public:

	WuVec3(double _x,double _y,double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	WuVec3(void)
	{
	}

	WuVec3(const double *p)
	{
		x = p[0];
		y = p[1];
		z = p[2];
	}

	double	x;
	double	y;
	double	z;
};

	typedef std::vector< WuVec3 > Vec3Vector;


/**
  Xiaolin Wu color quantization algorithm
*/
class WuColorQuantizer
{
public:
	// Constructor - Input parameter: DIB 24-bit to be quantized
	WuColorQuantizer(void);
	// Destructor
	~WuColorQuantizer();
	// Quantizer - Return value: quantized 8-bit (color palette) DIB
	void Quantize(int32_t PaletteSize,Vec3Vector &outputVertices);

	void addColor(double x,double y,double z);


typedef struct tagBox 
{
    int32_t r0;			 // min value, exclusive
    int32_t r1;			 // max value, inclusive
    int32_t g0;  
    int32_t g1;  
    int32_t b0;  
    int32_t b1;
    int32_t vol;
} Box;

private:
	int32_t table[256];
    double *mSumSquared;
	int32_t *mWeight;
	int32_t *mSumX;
	int32_t *mSumY;
	int32_t *mSumZ;

	void M3D(int32_t *vwt, int32_t *vmr, int32_t *vmg, int32_t *vmb, double *m2);
	int32_t Vol(Box *cube, int32_t *mmt);
	int32_t Bottom(Box *cube, uint8_t dir, int32_t *mmt);
	int32_t Top(Box *cube, uint8_t dir, int32_t pos, int32_t *mmt);
	double Var(Box *cube);
	double Maximize(Box *cube, uint8_t dir, int32_t first, int32_t last , int32_t *cut,
				   int32_t whole_r, int32_t whole_g, int32_t whole_b, int32_t whole_w);
	bool Cut(Box *set1, Box *set2);
	void Mark(Box *cube, int32_t label, uint8_t *tag);

};



// Size of a 3D array : 33 x 33 x 33
#define SIZE_3D	35937

// 3D array indexation
#define INDEX(r, g, b)	((r << 10) + (r << 6) + r + (g << 5) + g + b)

#define MAXCOLOR	1024

// Constructor / Destructor

WuColorQuantizer::WuColorQuantizer(void)
{
	// Allocate 3D arrays
	mSumSquared = (double*)malloc(SIZE_3D * sizeof(double));
	mWeight = (int32_t*)malloc(SIZE_3D * sizeof(int32_t));
	mSumX = (int32_t*)malloc(SIZE_3D * sizeof(int32_t));
	mSumY = (int32_t*)malloc(SIZE_3D * sizeof(int32_t));
	mSumZ = (int32_t*)malloc(SIZE_3D * sizeof(int32_t));

	memset(mSumSquared, 0, SIZE_3D * sizeof(double));
	memset(mWeight, 0, SIZE_3D * sizeof(int32_t));
	memset(mSumX, 0, SIZE_3D * sizeof(int32_t));
	memset(mSumY, 0, SIZE_3D * sizeof(int32_t));
	memset(mSumZ, 0, SIZE_3D * sizeof(int32_t));

	for(int32_t i = 0; i < 256; i++)
		table[i] = i * i;

}

WuColorQuantizer::~WuColorQuantizer() 
{
	if(mSumSquared)	free(mSumSquared);
	if(mWeight)	free(mWeight);
	if(mSumX)	free(mSumX);
	if(mSumY)	free(mSumY);
	if(mSumZ)	free(mSumZ);
}


// Histogram is in elements 1..HISTSIZE along each axis,
// element 0 is for base or marginal value
// NB: these must start out 0!

// Build 3-D color histogram of counts, r/g/b, c^2

void WuColorQuantizer::addColor(double x,double y,double z)
{
	uint32_t red = (uint32_t)(x*128+128);
	uint32_t green = (uint32_t)(y*128+128);
	uint32_t blue = (uint32_t)(z*128+128);
	assert( red < 256 );
	assert( green < 256 );
	assert( blue < 256 );

	uint32_t inr = (red >> 3) + 1;
	uint32_t ing = (green >> 3) + 1;
	uint32_t inb = (blue >> 3) + 1;
	uint32_t ind = INDEX(inr, ing, inb);
 
	mWeight[ind]++;
	mSumX[ind] += red;
	mSumY[ind] += green;
	mSumZ[ind] += blue;
	mSumSquared[ind] += table[red] + table[green] + table[blue];
}


// At conclusion of the histogram step, we can interpret
// mWeight[r][g][b] = sum over voxel of P(c)
// mSumX[r][g][b] = sum over voxel of r*P(c)  ,  similarly for mSumY, mSumZ
// m2[r][g][b] = sum over voxel of c^2*P(c)
// Actually each of these should be divided by 'ImageSize' to give the usual
// interpretation of P() as ranging from 0 to 1, but we needn't do that here.


// We now convert histogram into moments so that we can rapidly calculate
// the sums of the above quantities over any desired box.

// Compute cumulative moments
void WuColorQuantizer::M3D(int32_t *vwt, int32_t *vmr, int32_t *vmg, int32_t *vmb, double *m2) 
{
	uint32_t ind1, ind2;
	uint8_t i, r, g, b;
	int32_t line, line_r, line_g, line_b;
	int32_t area[33], area_r[33], area_g[33], area_b[33];
	double line2, area2[33];

	for(r = 1; r <= 32; r++) 
	{
		for(i = 0; i <= 32; i++) 
		{
			area2[i] = 0;
			area[i] = area_r[i] = area_g[i] = area_b[i] = 0;
		}
		for(g = 1; g <= 32; g++) 
		{
			line2 = 0;
			line = line_r = line_g = line_b = 0;
			for(b = 1; b <= 32; b++) 
			{			 
				ind1 = INDEX(r, g, b); // [r][g][b]
				line += vwt[ind1];
				line_r += vmr[ind1]; 
				line_g += vmg[ind1]; 
				line_b += vmb[ind1];
				line2 += m2[ind1];
				area[b] += line;
				area_r[b] += line_r;
				area_g[b] += line_g;
				area_b[b] += line_b;
				area2[b] += line2;
				ind2 = ind1 - 1089; // [r-1][g][b]
				vwt[ind1] = vwt[ind2] + area[b];
				vmr[ind1] = vmr[ind2] + area_r[b];
				vmg[ind1] = vmg[ind2] + area_g[b];
				vmb[ind1] = vmb[ind2] + area_b[b];
				m2[ind1] = m2[ind2] + area2[b];
			}
		}
	}
}

	// Compute sum over a box of any given statistic
	int32_t 
		WuColorQuantizer::Vol( Box *cube, int32_t *mmt ) {
			return( mmt[INDEX(cube->r1, cube->g1, cube->b1)] 
			- mmt[INDEX(cube->r1, cube->g1, cube->b0)]
			- mmt[INDEX(cube->r1, cube->g0, cube->b1)]
			+ mmt[INDEX(cube->r1, cube->g0, cube->b0)]
			- mmt[INDEX(cube->r0, cube->g1, cube->b1)]
			+ mmt[INDEX(cube->r0, cube->g1, cube->b0)]
			+ mmt[INDEX(cube->r0, cube->g0, cube->b1)]
			- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
	}

	// The next two routines allow a slightly more efficient calculation
	// of Vol() for a proposed subbox of a given box.  The sum of Top()
	// and Bottom() is the Vol() of a subbox split in the given direction
	// and with the specified new upper bound.


	// Compute part of Vol(cube, mmt) that doesn't depend on r1, g1, or b1
	// (depending on dir)

	int32_t 
		WuColorQuantizer::Bottom(Box *cube, uint8_t dir, int32_t *mmt) 
    {
			switch(dir)
			{
			case FI_RGBA_RED:
				return( - mmt[INDEX(cube->r0, cube->g1, cube->b1)]
				+ mmt[INDEX(cube->r0, cube->g1, cube->b0)]
				+ mmt[INDEX(cube->r0, cube->g0, cube->b1)]
				- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
			case FI_RGBA_GREEN:
				return( - mmt[INDEX(cube->r1, cube->g0, cube->b1)]
				+ mmt[INDEX(cube->r1, cube->g0, cube->b0)]
				+ mmt[INDEX(cube->r0, cube->g0, cube->b1)]
				- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
			case FI_RGBA_BLUE:
				return( - mmt[INDEX(cube->r1, cube->g1, cube->b0)]
				+ mmt[INDEX(cube->r1, cube->g0, cube->b0)]
				+ mmt[INDEX(cube->r0, cube->g1, cube->b0)]
				- mmt[INDEX(cube->r0, cube->g0, cube->b0)] );
			}

			return 0;
	}


	// Compute remainder of Vol(cube, mmt), substituting pos for
	// r1, g1, or b1 (depending on dir)

	int32_t 	WuColorQuantizer::Top(Box *cube, uint8_t dir, int32_t pos, int32_t *mmt) 
	{
			switch(dir)
			{
			case FI_RGBA_RED:
				return( mmt[INDEX(pos, cube->g1, cube->b1)] 
				-mmt[INDEX(pos, cube->g1, cube->b0)]
				-mmt[INDEX(pos, cube->g0, cube->b1)]
				+mmt[INDEX(pos, cube->g0, cube->b0)] );
			case FI_RGBA_GREEN:
				return( mmt[INDEX(cube->r1, pos, cube->b1)] 
				-mmt[INDEX(cube->r1, pos, cube->b0)]
				-mmt[INDEX(cube->r0, pos, cube->b1)]
				+mmt[INDEX(cube->r0, pos, cube->b0)] );
			case FI_RGBA_BLUE:
				return( mmt[INDEX(cube->r1, cube->g1, pos)]
				-mmt[INDEX(cube->r1, cube->g0, pos)]
				-mmt[INDEX(cube->r0, cube->g1, pos)]
				+mmt[INDEX(cube->r0, cube->g0, pos)] );
			}

			return 0;
	}

	// Compute the weighted variance of a box 
	// NB: as with the raw statistics, this is really the variance * ImageSize 

	double	WuColorQuantizer::Var(Box *cube) 
	{
		double dr = (double) Vol(cube, mSumX); 
		double dg = (double) Vol(cube, mSumY); 
		double db = (double) Vol(cube, mSumZ);
		double xx =  mSumSquared[INDEX(cube->r1, cube->g1, cube->b1)] 
			-mSumSquared[INDEX(cube->r1, cube->g1, cube->b0)]
			-mSumSquared[INDEX(cube->r1, cube->g0, cube->b1)]
			+mSumSquared[INDEX(cube->r1, cube->g0, cube->b0)]
			-mSumSquared[INDEX(cube->r0, cube->g1, cube->b1)]
			+mSumSquared[INDEX(cube->r0, cube->g1, cube->b0)]
			+mSumSquared[INDEX(cube->r0, cube->g0, cube->b1)]
			-mSumSquared[INDEX(cube->r0, cube->g0, cube->b0)];

		return (xx - (dr*dr+dg*dg+db*db)/(double)Vol(cube,mWeight));    
	}

	// We want to minimize the sum of the variances of two subboxes.
	// The sum(c^2) terms can be ignored since their sum over both subboxes
	// is the same (the sum for the whole box) no matter where we split.
	// The remaining terms have a minus sign in the variance formula,
	// so we drop the minus sign and MAXIMIZE the sum of the two terms.

	double	WuColorQuantizer::Maximize(Box *cube, uint8_t dir, int32_t first, int32_t last , int32_t *cut, int32_t whole_r, int32_t whole_g, int32_t whole_b, int32_t whole_w) 
	{
			int32_t half_r, half_g, half_b, half_w;
			int32_t i;
			double temp;

			int32_t base_r = Bottom(cube, dir, mSumX);
			int32_t base_g = Bottom(cube, dir, mSumY);
			int32_t base_b = Bottom(cube, dir, mSumZ);
			int32_t base_w = Bottom(cube, dir, mWeight);

			double max = 0.0;

			*cut = -1;

			for (i = first; i < last; i++) {
				half_r = base_r + Top(cube, dir, i, mSumX);
				half_g = base_g + Top(cube, dir, i, mSumY);
				half_b = base_b + Top(cube, dir, i, mSumZ);
				half_w = base_w + Top(cube, dir, i, mWeight);

				// now half_x is sum over lower half of box, if split at i

				if (half_w == 0) {		// subbox could be empty of pixels!
					continue;			// never split into an empty box
				} else {
					temp = ((double)half_r*half_r + (double)half_g*half_g + (double)half_b*half_b)/half_w;
				}

				half_r = whole_r - half_r;
				half_g = whole_g - half_g;
				half_b = whole_b - half_b;
				half_w = whole_w - half_w;

				if (half_w == 0) {		// subbox could be empty of pixels!
					continue;			// never split into an empty box
				} else {
					temp += ((double)half_r*half_r + (double)half_g*half_g + (double)half_b*half_b)/half_w;
				}

				if (temp > max) {
					max=temp;
					*cut=i;
				}
			}

			return max;
	}

	bool
		WuColorQuantizer::Cut(Box *set1, Box *set2) {
			uint8_t dir;
			int32_t cutr, cutg, cutb;

			int32_t whole_r = Vol(set1, mSumX);
			int32_t whole_g = Vol(set1, mSumY);
			int32_t whole_b = Vol(set1, mSumZ);
			int32_t whole_w = Vol(set1, mWeight);

			double maxr = Maximize(set1, FI_RGBA_RED, set1->r0+1, set1->r1, &cutr, whole_r, whole_g, whole_b, whole_w);    
			double maxg = Maximize(set1, FI_RGBA_GREEN, set1->g0+1, set1->g1, &cutg, whole_r, whole_g, whole_b, whole_w);    
			double maxb = Maximize(set1, FI_RGBA_BLUE, set1->b0+1, set1->b1, &cutb, whole_r, whole_g, whole_b, whole_w);

			if ((maxr >= maxg) && (maxr >= maxb)) {
				dir = FI_RGBA_RED;

				if (cutr < 0) {
					return false; // can't split the box
				}
			} else if ((maxg >= maxr) && (maxg>=maxb)) {
				dir = FI_RGBA_GREEN;
			} else {
				dir = FI_RGBA_BLUE;
			}

			set2->r1 = set1->r1;
			set2->g1 = set1->g1;
			set2->b1 = set1->b1;

			switch (dir) {
		case FI_RGBA_RED:
			set2->r0 = set1->r1 = cutr;
			set2->g0 = set1->g0;
			set2->b0 = set1->b0;
			break;

		case FI_RGBA_GREEN:
			set2->g0 = set1->g1 = cutg;
			set2->r0 = set1->r0;
			set2->b0 = set1->b0;
			break;

		case FI_RGBA_BLUE:
			set2->b0 = set1->b1 = cutb;
			set2->r0 = set1->r0;
			set2->g0 = set1->g0;
			break;
			}

			set1->vol = (set1->r1-set1->r0)*(set1->g1-set1->g0)*(set1->b1-set1->b0);
			set2->vol = (set2->r1-set2->r0)*(set2->g1-set2->g0)*(set2->b1-set2->b0);

			return true;
	}


	void
		WuColorQuantizer::Mark(Box *cube, int32_t label, uint8_t *tag) 
    {
			for (int32_t r = cube->r0 + 1; r <= cube->r1; r++) 
            {
				for (int32_t g = cube->g0 + 1; g <= cube->g1; g++) 
                {
					for (int32_t b = cube->b0 + 1; b <= cube->b1; b++) 
                    {
						tag[INDEX(r, g, b)] = (uint8_t)label;
					}
				}
			}
	}

// Wu Quantization algorithm
void WuColorQuantizer::Quantize(int32_t PaletteSize,Vec3Vector &outputVertices) 
{
	uint8_t *tag = NULL;

	if ( PaletteSize > MAXCOLOR )
	{
		PaletteSize = MAXCOLOR;
	}
	Box	cube[MAXCOLOR];
	int32_t	next;
	int32_t i, weight;
	int32_t k;
	double vv[MAXCOLOR], temp;

	// Compute moments
	M3D(mWeight, mSumX, mSumY, mSumZ, mSumSquared);

	cube[0].r0 = cube[0].g0 = cube[0].b0 = 0;
	cube[0].r1 = cube[0].g1 = cube[0].b1 = 32;
	next = 0;

	for (i = 1; i < PaletteSize; i++) 
	{
		if(Cut(&cube[next], &cube[i])) 
		{
			// volume test ensures we won't try to cut one-cell box
			vv[next] = (cube[next].vol > 1) ? Var(&cube[next]) : 0;
			vv[i] = (cube[i].vol > 1) ? Var(&cube[i]) : 0;
		} 
		else 
		{
			vv[next] = 0.0;   // don't try to split this box again
			i--;              // didn't create box i
		}

		next = 0; temp = vv[0];

		for (k = 1; k <= i; k++) 
		{
			if (vv[k] > temp) 
			{
				temp = vv[k]; next = k;
			}
		}

		if (temp <= 0.0) 
		{
			PaletteSize = i + 1;
			// Error: "Only got 'PaletteSize' boxes"
			break;
		}
	}

	// Partition done
	// the space for array mSumSquared can be freed now
	free(mSumSquared);
	mSumSquared = NULL;

	// create an optimized palette
	tag = (uint8_t*) malloc(SIZE_3D * sizeof(uint8_t));
	memset(tag, 0, SIZE_3D * sizeof(uint8_t));

	for (k = 0; k < PaletteSize ; k++) 
	{
		Mark(&cube[k], k, tag);
		weight = Vol(&cube[k], mWeight);

		if (weight) 
		{
			int32_t red	= (int32_t)(((double)Vol(&cube[k], mSumX) / (double)weight) + 0.5f);
			int32_t green = (int32_t)(((double)Vol(&cube[k], mSumY) / (double)weight) + 0.5f);
			int32_t blue	= (int32_t)(((double)Vol(&cube[k], mSumZ) / (double)weight) + 0.5f);
			assert( red >= 0 && red < 256 );
			assert( green >= 0 && green < 256 );
			assert( blue >= 0 && blue < 256 );
			WuVec3 v;
			v.x = (red-128.0f)/128.0f;
			v.y = (green-128.0f)/128.0f;
			v.z = (blue-128.0f)/128.0f;
			outputVertices.push_back(v);
		} 
		else 
		{
		}
	}
    free(tag);
}


class MyWuQuantizer : public WuQuantizer
{
public:
	MyWuQuantizer(void)
	{
		mScale = WuVec3(1,1,1);
		mCenter = WuVec3(0,0,0);
	}

	// use the Wu quantizer with 10 bits of resolution on each axis.  Precision down to 0.0009765625
	// All input data is normalized to a unit cube.

	virtual const double * wuQuantize3D(uint32_t vcount,
		const double *vertices,
		bool denormalizeResults,
		uint32_t maxVertices,
		uint32_t &outputCount)
	{
		const double *ret = NULL;
		outputCount = 0;

		normalizeInput(vcount,vertices);

		WuColorQuantizer wcq;

		for (uint32_t i=0; i<vcount; i++)
		{
			const WuVec3 &v = mNormalizedInput[i];
			wcq.addColor(v.x,v.y,v.z);
		}

		wcq.Quantize(maxVertices,mQuantizedOutput);

		outputCount = (uint32_t)mQuantizedOutput.size();

		if ( outputCount > 0 )
		{
			if ( denormalizeResults )
			{
				for (uint32_t i=0; i<outputCount; i++)
				{
					WuVec3 &v = mQuantizedOutput[i];

					v.x = v.x*mScale.x + mCenter.x;
					v.y = v.y*mScale.y + mCenter.y;
					v.z = v.z*mScale.z + mCenter.z;

					mQuantizedOutput.push_back(v);
				}
			}
			ret = &mQuantizedOutput[0].x;
		}


		return ret;
	}

	virtual void release(void)
	{
		delete this;
	}

	virtual const double * getDenormalizeScale(void) const 
	{
		return &mScale.x;
	}

	virtual const double * getDenormalizeCenter(void) const
	{
		return &mCenter.x;
	}



private:

	void normalizeInput(uint32_t vcount,const double *vertices)
	{
		mNormalizedInput.resize(vcount);
		mQuantizedOutput.clear();

		WuVec3 bmin(vertices);
		WuVec3 bmax(vertices);

		for (uint32_t i=1; i<vcount; i++)
		{
			WuVec3 v(&vertices[i*3]);

			if ( v.x < bmin.x ) 
			{
				bmin.x = v.x;
			}
			else if ( v.x > bmax.x )
			{
				bmax.x = v.x;
			}

			if ( v.y < bmin.y ) 
			{
				bmin.y = v.y;
			}
			else if ( v.y > bmax.y )
			{
				bmax.y = v.y;
			}

			if ( v.z < bmin.z ) 
			{
				bmin.z = v.z;
			}
			else if ( v.z > bmax.z )
			{
				bmax.z = v.z;
			}
		}

		mCenter.x = (bmin.x+bmax.x)*0.5f;
		mCenter.y = (bmin.y+bmax.y)*0.5f;
		mCenter.z = (bmin.z+bmax.z)*0.5f;

		double dx = bmax.x-bmin.x;
		double dy = bmax.y-bmin.y;
		double dz = bmax.z-bmin.z;

		if ( dx == 0 )
		{
			mScale.x = 1;
		}
		else
		{
			dx = dx*1.001f;
			mScale.x = dx*0.5f;
		}
		if ( dy == 0 )
		{
			mScale.y = 1;
		}
		else
		{
			dy = dy*1.001f;
			mScale.y = dy*0.5f;
		}
		if ( dz == 0 )
		{
			mScale.z = 1;
		}
		else
		{
			dz = dz*1.001f;
			mScale.z = dz*0.5f;
		}

		WuVec3 recip;
		recip.x = 1.0f / mScale.x;
		recip.y = 1.0f / mScale.y;
		recip.z = 1.0f / mScale.z;

		for (uint32_t i=0; i<vcount; i++)
		{
			WuVec3 v(&vertices[i*3]);

			v.x = (v.x-mCenter.x)*recip.x;
			v.y = (v.y-mCenter.y)*recip.y;
			v.z = (v.z-mCenter.z)*recip.z;

			assert( v.x >= -1 && v.x <= 1 );
			assert( v.y >= -1 && v.y <= 1 );
			assert( v.z >= -1 && v.z <= 1 );

			mNormalizedInput[i] = v;
		}
	}

	virtual ~MyWuQuantizer(void)
	{

	}

	WuVec3		mScale;
	WuVec3		mCenter;
	Vec3Vector	mNormalizedInput;
	Vec3Vector	mQuantizedOutput;
};

WuQuantizer * WuQuantizer::create(void)
{
	MyWuQuantizer *m = new MyWuQuantizer;
	return static_cast< WuQuantizer *>(m);
}


}; 

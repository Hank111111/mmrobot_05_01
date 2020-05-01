#pragma once
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Copyright (C) 2014, Itseez, Inc, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

//#include "precomp_v3_4_5.hpp"
//#include "opencv2/core/hal/intrin.hpp"

#include <algorithm>
#include <iterator>
#include <opencv2/opencv.hpp>
using namespace cv;
namespace cv_v3_4_5
{
	/****************************************************************************************\
	*                                 C++11 override / final                                 *
	\****************************************************************************************/

	#ifndef CV_OVERRIDE
	#  ifdef CV_CXX11
	#    define CV_OVERRIDE override
	#  endif
	#endif
	#ifndef CV_OVERRIDE
	#  define CV_OVERRIDE
	#endif

	#ifndef CV_FINAL
	#  ifdef CV_CXX11
	#    define CV_FINAL final
	#  endif
	#endif
	#ifndef CV_FINAL
	#  define CV_FINAL
	#endif

	#ifdef __CV_AVX_GUARD
	#define CV_INSTRUMENT_REGION(); __CV_AVX_GUARD CV_INSTRUMENT_REGION_();
	#else
	#define CV_INSTRUMENT_REGION(); CV_INSTRUMENT_REGION_();
	#endif

	// Classical Hough Transform
	struct LinePolar
	{
		float rho;
		float angle;
	};

	/** Variants of a Hough transform */
	enum
	{
		CV_HOUGH_STANDARD =0,
		CV_HOUGH_PROBABILISTIC =1,
		CV_HOUGH_MULTI_SCALE =2,
		CV_HOUGH_GRADIENT =3
	};

	struct hough_cmp_gt
	{
		hough_cmp_gt(const int* _aux) : aux(_aux) {}
		inline bool operator()(int l1, int l2) const
		{
			return aux[l1] > aux[l2] || (aux[l1] == aux[l2] && l1 < l2);
		}
		const int* aux;
	};


	/*
	Here image is an input raster;
	step is it's step; size characterizes it's ROI;
	rho and theta are discretization steps (in pixels and radians correspondingly).
	threshold is the minimum number of pixels in the feature for it
	to be a candidate for line. lines is the output
	array of (rho, theta) pairs. linesMax is the buffer size (number of pairs).
	Functions return the actual number of found lines.
	*/

	// Multi-Scale variant of Classical Hough Transform

	struct hough_index
	{
		hough_index() : value(0), rho(0.f), theta(0.f) {}
		hough_index(int _val, float _rho, float _theta)
			: value(_val), rho(_rho), theta(_theta) {}

		int value;
		float rho, theta;
	};


	/****************************************************************************************\
	*                              Probabilistic Hough Transform                             *
	\****************************************************************************************/


	void HoughLines(InputArray _image, OutputArray lines,
		double rho, double theta, int threshold,
		double srn, double stn, double min_theta, double max_theta);


	void HoughLinesPointSet(InputArray _point, OutputArray _lines, int lines_max, int threshold,
		double min_rho, double max_rho, double rho_step,
		double min_theta, double max_theta, double theta_step);


	void HoughCircles( InputArray image, OutputArray circles,
                               int method, double dp, double minDist,
                               double param1 = 100, double param2 = 100,
                               int minRadius = 0, int maxRadius = 0 );
} // \namespace cv


/* End of file. */

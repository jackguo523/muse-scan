// focus measure metrics for auto-focus process including:
// 1. graylevel standard deviation
// 2. spatial frequency
// 3. Brenner's first differentiation
// 4. histogram entropy

#pragma once

#ifndef FMEASURE_H
#define FMEASURE_H

#include <vector>
#include <algorithm>

namespace fm {
	enum fmetric;

	template<typename T>
	inline void rgb2gray(const T *in, T *out, int width, int height, int luma = 1);

	template<typename T>
	float glsd(const T *in, int width, int height);

	template<typename T>
	float spfq(const T *in, int width, int height);

	template<typename T>
	float bren(const T *in, int width, int height);

	template<typename T>
	float hise(const T *in, int width, int height);

	template<typename T>
	float eval_fm(const T *in, int width, int height, fmetric alg);
}

#endif
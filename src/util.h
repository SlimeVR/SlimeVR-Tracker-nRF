/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMENRF_UTILS
#define SLIMENRF_UTILS

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884f
#endif

#ifndef EPS
#define EPS 1e-6f
#endif

// Saturate int to 16 bits
// Optimized to a single ARM assembler instruction
#define SATURATE_INT16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

#define SATURATE_UINT11(x) ((x) > 2047 ? 2047 : ((x) < 0 ? 0 : (x)))
#define SATURATE_UINT10(x) ((x) > 1023 ? 1023 : ((x) < 0 ? 0 : (x)))

#define TO_FIXED_15(x) ((int16_t)SATURATE_INT16((x) * (1 << 15)))
#define TO_FIXED_11(x) ((int16_t)SATURATE_INT16((x) * (1 << 11)))
#define TO_FIXED_10(x) ((int16_t)SATURATE_INT16((x) * (1 << 10)))
#define TO_FIXED_7(x) ((int16_t)SATURATE_INT16((x) * (1 << 7)))
#define FIXED_15_TO_DOUBLE(x) (((double)(x)) / (1 << 15))
#define FIXED_11_TO_DOUBLE(x) (((double)(x)) / (1 << 11))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))
#define FIXED_7_TO_DOUBLE(x) (((double)(x)) / (1 << 7))
#define ABS(x) (x < 0 ? -x : x)

#define CONST_EARTH_GRAVITY 9.80665f

void q_normalize(const float *q, float *out);
void q_multiply(const float *x, const float *y, float *out);
void q_conj(const float *q, float *out);
void q_negate(const float *q, float *out);
float q_diff_mag(const float *x, const float *y);
void v_rotate(const float *v, const float *q, float *out);
float v_avg(const float *a);
float v_diff_mag(const float *a, const float *b);
bool q_epsilon(const float *x, const float *y, float eps);
bool v_epsilon(const float *a, const float *b, float eps);

// TODO: does this need to be moved?
void apply_BAinv(float xyz[3], float BAinv[4][3]);

void a_to_lin_a(const float *q, const float *a, float *lin_a);

void q_fem(const float *q, float *out);
void q_iem(const float *v, float *out);

#endif

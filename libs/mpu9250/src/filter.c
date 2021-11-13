/**
 * @file filter.c
 * @brief      This is a collection of functions for generating and implementing
 *             discrete SISO filters for arbitrary transfer functions.
 *
 * @author     James Strawson
 * @date       2016
 */

#include <stdio.h>
#include <math.h>
#include <string.h> // for memset
#include <stdlib.h>
#include <stdint.h>

#include "filter.h"
#include "polynomial.h"

#include "algebra_common.h"

#include "cli.h"

// local function
static int __print_poly_z(vector_t v)
{
	int i;
	// utf8 characters for superscript digits
	static char *super[] = {"\xe2\x81\xb0", "\xc2\xb9", "\xc2\xb2",
							"\xc2\xb3", "\xe2\x81\xb4", "\xe2\x81\xb5", "\xe2\x81\xb6",
							"\xe2\x81\xb7", "\xe2\x81\xb8", "\xe2\x81\xb9"};
	if (unlikely(v.len > 10))
	{
		cli_printf("ERROR in filter_print, filter order must be <=10\n");
		return -1;
	}
	for (i = 0; i < (v.len - 2); i++)
		cli_printf("%7.4fz%s + ", v.d[i], super[v.len - i - 1]);
	if (v.len >= 2)
		cli_printf("%7.4fz  + ", v.d[v.len - 2]);
	cli_printf("%7.4f\n", v.d[v.len - 1]);
	return 0;
}

filter_t filter_empty(void)
{
	filter_t f = RC_FILTER_INITIALIZER;
	return f;
}

int filter_alloc(filter_t *f, vector_t num, vector_t den, double dt)
{
	// sanity checks
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_alloc, dt must be >0\n");
		return -1;
	}
	if (unlikely(!num.initialized || !den.initialized))
	{
		cli_printf("ERROR in filter_alloc, vector uninitialized\n");
		return -1;
	}
	if (unlikely(num.len > den.len))
	{
		cli_printf("ERROR in filter_alloc, improper transfer function\n");
		return -1;
	}
	if (unlikely(fabs(den.d[0]) < zero_tolerance))
	{
		cli_printf("ERROR in filter_alloc, first coefficient in denominator is 0\n");
		return -1;
	}
	// free existing memory, this also zeros out all fields
	filter_free(f);
	// move in vectors
	if (unlikely(vector_duplicate(num, &f->num)))
	{
		cli_printf("ERROR in filter_alloc, failed to duplicate numerator\n");
		return -1;
	}
	if (unlikely(vector_duplicate(den, &f->den)))
	{
		cli_printf("ERROR in filter_alloc, failed to duplicate denominator\n");
		vector_free(&f->num);
		return -1;
	}
	// allocate buffers making sure they are at least 2 in length
	int buflen = den.len;
	if (buflen < 2)
		buflen = 2;
	if (unlikely(ringbuf_alloc(&f->in_buf, buflen)))
	{
		cli_printf("ERROR in filter_alloc, failed to allocate ring buffer\n");
		vector_free(&f->num);
		vector_free(&f->den);
		return -1;
	}
	if (unlikely(ringbuf_alloc(&f->out_buf, buflen)))
	{
		cli_printf("ERROR in filter_alloc, failed to allocate ring buffer\n");
		vector_free(&f->num);
		vector_free(&f->den);
		ringbuf_free(&f->in_buf);
		return -1;
	}
	// populate remaining values, everything else zero'd by filter_free
	f->dt = dt;
	f->order = den.len - 1;
	f->initialized = 1;
	return 0;
}

int filter_alloc_from_arrays(filter_t *f, double dt, double *num, int numlen,
							 double *den, int denlen)
{
	// sanity checks
	if (unlikely(numlen < 1 || denlen < 1))
	{
		cli_printf("ERROR in filter_alloc_from_arrays, numlen & denlen must be >=1\n");
		return -1;
	}
	if (unlikely(numlen > denlen))
	{
		cli_printf("ERROR in filter_alloc_from_arrays, improper transfer function\n");
		return -1;
	}
	if (unlikely(num == NULL || den == NULL || f == NULL))
	{
		cli_printf("ERROR in filter_alloc_from_arrays, received null pointer\n");
		return -1;
	}
	if (unlikely(dt < 0.0))
	{
		cli_printf("ERROR in filter_alloc_from_arrays, dt must be >0\n");
		return -1;
	}
	if (unlikely(fabs(den[0]) < zero_tolerance))
	{
		cli_printf("ERROR in filter_alloc_from_arrays, first coefficient in denominator is 0\n");
		return -1;
	}
	// free existing memory, this also zeros out all fields
	filter_free(f);
	// copy numerator and denominators over
	if (unlikely(vector_from_array(&f->num, num, numlen)))
	{
		cli_printf("ERROR in filter_alloc_from_arrays, failed to alloc vector\n");
		return -1;
	}
	if (unlikely(vector_from_array(&f->den, den, denlen)))
	{
		cli_printf("ERROR in filter_alloc_from_arrays, failed to alloc vector\n");
		vector_free(&f->num);
		return -1;
	}
	// allocate buffers
	if (unlikely(ringbuf_alloc(&f->in_buf, denlen)))
	{
		cli_printf("ERROR in filter_alloc, failed to allocate ring buffer\n");
		vector_free(&f->num);
		vector_free(&f->den);
		return -1;
	}
	if (unlikely(ringbuf_alloc(&f->out_buf, denlen)))
	{
		cli_printf("ERROR in filter_alloc, failed to allocate ring buffer\n");
		vector_free(&f->num);
		vector_free(&f->den);
		ringbuf_free(&f->in_buf);
		return -1;
	}
	// populate remaining values, everything else zero'd by filter_free
	f->dt = dt;
	f->order = denlen - 1;
	f->initialized = 1;
	return 0;
}

int filter_duplicate(filter_t *f, filter_t old)
{
	if (unlikely(!old.initialized))
	{
		cli_printf("ERROR in filter_duplicate, old filter not initialized\n");
		return -1;
	}
	if (filter_alloc(f, old.num, old.den, old.dt))
	{
		cli_printf("ERROR in filter_duplicate, failed to alloc memory\n");
		return -1;
	}
	f->gain = old.gain;
	f->sat_en = old.sat_en;
	f->sat_min = old.sat_min;
	f->sat_max = old.sat_max;
	f->ss_en = old.ss_en;
	f->ss_steps = old.ss_steps;
	return 0;
}

int filter_free(filter_t *f)
{
	filter_t new = RC_FILTER_INITIALIZER;
	if (unlikely(f == NULL))
	{
		cli_printf("ERROR in filter_free, received NULL pointer\n");
		return -1;
	}
	ringbuf_free(&f->in_buf);
	ringbuf_free(&f->out_buf);
	vector_free(&f->num);
	vector_free(&f->den);
	*f = new;
	return 0;
}

double filter_march(filter_t *f, double new_input)
{
	int i, rel_deg;
	double tmp1 = 0.0;
	double tmp2 = 0.0;
	double new_out;
	// sanity checks
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_march, filter uninitialized\n");
		return -1.0;
	}
	// log new input
	ringbuf_insert(&f->in_buf, new_input);
	f->newest_input = new_input;
	// relative degree should never be negative as filter_alloc checks
	// for improper transfer functions
	rel_deg = f->den.len - f->num.len;
	// evaluate the difference equation
	for (i = 0; i < (f->num.len); i++)
	{
		tmp1 += f->num.d[i] * ringbuf_get_value(&f->in_buf, i + rel_deg);
	}
	if (fabs(f->gain - 1.0) > zero_tolerance)
		tmp1 = tmp1 * f->gain;
	for (i = 0; i < (f->order); i++)
	{
		tmp2 -= f->den.d[i + 1] * ringbuf_get_value(&f->out_buf, i);
	}
	new_out = tmp2 + tmp1;
	// scale in case denominator doesn't have a leading 1
	if (fabs(f->den.d[0] - 1.0) > zero_tolerance)
		new_out /= f->den.d[0];
	// soft start limits
	if (f->ss_en && f->step < f->ss_steps)
	{
		double a = f->sat_max * (f->step / f->ss_steps);
		double b = f->sat_min * (f->step / f->ss_steps);
		if (new_out > a)
			new_out = a;
		if (new_out < b)
			new_out = b;
	}
	// saturate and set flag
	if (f->sat_en)
	{
		if (new_out > f->sat_max)
		{
			new_out = f->sat_max;
			f->sat_flag = 1;
		}
		else if (new_out < f->sat_min)
		{
			new_out = f->sat_min;
			f->sat_flag = 1;
		}
		else
			f->sat_flag = 0;
	}
	// record the output to filter struct and ring buffer
	f->newest_output = new_out;
	ringbuf_insert(&f->out_buf, new_out);
	// increment steps
	f->step++;
	return new_out;
}

int filter_reset(filter_t *f)
{
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_reset, filter uninitialized\n");
		return -1;
	}
	ringbuf_reset(&f->in_buf);
	ringbuf_reset(&f->out_buf);
	f->newest_input = 0.0;
	f->newest_output = 0.0;
	f->sat_flag = 0;
	f->step = 0;
	return 0;
}

int filter_print(filter_t f)
{
	int i;
	if (unlikely(!f.initialized))
	{
		cli_printf("ERROR in filter_print, filter not initialized yet\n");
		return -1;
	}
	if (unlikely(f.order > 9))
	{
		cli_printf("ERROR in filter_print, filter order must be <=10\n");
		return -1;
	}

	cli_printf("order: %d\n", f.order);
	cli_printf("timestep dt: %0.4f\n", f.dt);

	// print numerator
	__print_poly_z(f.num);
	cli_printf("--------");
	for (i = 0; i < f.order; i++)
		cli_printf("------------");
	cli_printf("\n");
	__print_poly_z(f.den);

	return 0;
}

int filter_enable_saturation(filter_t *f, double min, double max)
{
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_enable_saturation, filter uninitialized\n");
		return -1;
	}
	if (unlikely(min > max))
	{
		cli_printf("ERORR in filter_enable_saturation, max must be >= min\n");
		return -1;
	}
	f->sat_en = 1;
	f->sat_min = min;
	f->sat_max = max;
	return 0;
}

int filter_get_saturation_flag(filter_t *f)
{
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_get_saturation_flag, filter uninitialized\n");
		return -1;
	}
	return f->sat_flag;
}

int filter_enable_soft_start(filter_t *f, double seconds)
{
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_enable_soft_start, filter uninitialized\n");
		return -1;
	}
	if (unlikely(seconds <= 0.0))
	{
		cli_printf("ERROR in filter_enable_soft_start, seconds must be >=0\n");
		return -1;
	}
	if (unlikely(!f->sat_en))
	{
		cli_printf("ERROR in filter_enable_soft_start, saturation must be enabled first\n");
		return -1;
	}
	f->ss_en = 1;
	f->ss_steps = seconds / f->dt;
	return 0;
}

double filter_previous_input(filter_t *f, int steps)
{
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_previous_input, filter uninitialized\n");
		return -1.0;
	}
	return ringbuf_get_value(&f->in_buf, steps);
}

double filter_previous_output(filter_t *f, int steps)
{
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_previous_output, filter uninitialized\n");
		return -1.0;
	}
	return ringbuf_get_value(&f->out_buf, steps);
}

int filter_prefill_inputs(filter_t *f, double in)
{
	int i;
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_prefill_inputs, filter uninitialized\n");
		return -1;
	}
	for (i = 0; i <= f->order; i++)
	{
		ringbuf_insert(&f->in_buf, in);
	}
	f->newest_input = in;
	return 0;
}

int filter_prefill_outputs(filter_t *f, double out)
{
	int i;
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_prefill_outputs, filter uninitialized\n");
		return -1;
	}
	for (i = 0; i <= f->order; i++)
	{
		ringbuf_insert(&(f->out_buf), out);
	}
	f->newest_output = out;
	return 0;
}

int filter_multiply(filter_t f1, filter_t f2, filter_t *f3)
{
	vector_t newnum = RC_VECTOR_INITIALIZER;
	vector_t newden = RC_VECTOR_INITIALIZER;
	// sanity checks
	if (unlikely(!f1.initialized || !f2.initialized))
	{
		cli_printf("ERROR in filter_multiply, filter uninitialized\n");
		return -1;
	}
	if (unlikely(fabs(f1.dt - f2.dt) > zero_tolerance))
	{
		cli_printf("ERROR in filter_multiply, timestep dt must match\n");
		return -1;
	}
	// multiply out the transfer function coefficients
	if (unlikely(poly_conv(f1.num, f2.num, &newnum)))
	{
		cli_printf("ERROR in filter_multiply, failed to polyconv\n");
		return -1;
	}
	if (unlikely(poly_conv(f1.den, f2.den, &newden)))
	{
		cli_printf("ERROR in filter_multiply, failed to polyconv\n");
		vector_free(&newnum);
		return -1;
	}
	// create the filter
	if (unlikely(filter_alloc(f3, newnum, newden, f1.dt)))
	{
		cli_printf("ERROR in filter_multiply, failed to alloc filter\n");
		return -1;
	}
	f3->gain = f1.gain * f2.gain;
	vector_free(&newnum);
	vector_free(&newden);
	return 0;
}

int filter_multiply_three(filter_t f1, filter_t f2, filter_t f3, filter_t *out)
{
	vector_t newnum = RC_VECTOR_INITIALIZER;
	vector_t newden = RC_VECTOR_INITIALIZER;
	vector_t tmp = RC_VECTOR_INITIALIZER;

	// sanity checks
	if (unlikely(!f1.initialized || !f2.initialized || !f3.initialized))
	{
		cli_printf("ERROR in filter_multiply_three, filter uninitialized\n");
		return -1;
	}
	if (unlikely(fabs(f1.dt - f2.dt) > zero_tolerance || fabs(f2.dt - f3.dt) > zero_tolerance))
	{
		cli_printf("ERROR in filter_multiply_three, timestep dt must match\n");
		return -1;
	}
	// multiply out the transfer function coefficients
	if (unlikely(poly_conv(f1.num, f2.num, &tmp)))
	{
		cli_printf("ERROR in filter_multiply_three, failed to polyconv\n");
		return -1;
	}
	if (unlikely(poly_conv(f3.num, tmp, &newnum)))
	{
		cli_printf("ERROR in filter_multiply_three, failed to polyconv\n");
		vector_free(&tmp);
		return -1;
	}
	if (unlikely(poly_conv(f1.den, f2.den, &tmp)))
	{
		cli_printf("ERROR in filter_multiply_three, failed to polyconv\n");
		vector_free(&newnum);
		vector_free(&tmp);
		return -1;
	}
	if (unlikely(poly_conv(tmp, f3.den, &newden)))
	{
		cli_printf("ERROR in filter_multiply_three, failed to polyconv\n");
		vector_free(&newnum);
		vector_free(&tmp);
		return -1;
	}
	// create the filter
	if (unlikely(filter_alloc(out, newnum, newden, f1.dt)))
	{
		cli_printf("ERROR in filter_multiply_three, failed to alloc filter\n");
		vector_free(&tmp);
		vector_free(&newnum);
		vector_free(&newden);
		return -1;
	}
	out->gain = f1.gain * f2.gain * f3.gain;
	vector_free(&tmp);
	vector_free(&newnum);
	vector_free(&newden);
	return 0;
}

int filter_c2d_tustin(filter_t *f, double dt, vector_t num, vector_t den, double w)
{
	int i, j, m, n;
	double a, c, A0;
	vector_t numZ = RC_VECTOR_INITIALIZER;
	vector_t denZ = RC_VECTOR_INITIALIZER;
	vector_t p1 = RC_VECTOR_INITIALIZER;
	vector_t p2 = RC_VECTOR_INITIALIZER;
	vector_t temp = RC_VECTOR_INITIALIZER;
	vector_t v1 = RC_VECTOR_INITIALIZER;
	vector_t v2 = RC_VECTOR_INITIALIZER;
	// sanity checks
	if (unlikely(!num.initialized || !den.initialized))
	{
		cli_printf("ERROR in filter_c2d_tustin, vector uninitialized\n");
		return -1;
	}
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_c2d_tustin, dt must be positive\n");
		return -1;
	}
	if (unlikely(w > (M_PI / dt)))
	{
		cli_printf("ERROR in filter_c2d_tustin, w larger than nyquist frequency\n");
		return -1;
	}
	a = 2.0 * (1.0 - cos(w * dt)) / (w * dt * sin(w * dt));
	c = 2.0 / (a * dt);
	m = num.len - 1; // order of num
	n = den.len - 1; // order of den
	vector_zeros(&numZ, n + 1);
	vector_zeros(&denZ, n + 1);
	vector_alloc(&p1, 2); // (z - 1)
	p1.d[0] = 1.0;
	p1.d[1] = -1.0;
	vector_alloc(&p2, 2); // (z + 1)
	p2.d[0] = 1.0;
	p2.d[1] = 1.0;

	// from zeroth up to and including mth
	for (i = 0; i <= m; i++)
	{
		poly_power(p1, m - i, &v1);
		poly_power(p2, n - m + i, &v2);
		poly_conv(v1, v2, &temp);
		for (j = 0; j < n + 1; j++)
			numZ.d[j] += num.d[i] * pow(c, m - i) * temp.d[j];
	}
	for (i = 0; i <= n; i++)
	{
		poly_power(p1, n - i, &v1);
		poly_power(p2, i, &v2);
		poly_conv(v1, v2, &temp);
		for (j = 0; j < n + 1; j++)
			denZ.d[j] += den.d[i] * pow(c, n - i) * temp.d[j];
	}
	A0 = denZ.d[0];

	// normalize
	for (i = 0; i < n + 1; i++)
	{
		numZ.d[i] = numZ.d[i] / A0;
		denZ.d[i] = denZ.d[i] / A0;
	}
	// free up memory
	vector_free(&p1);
	vector_free(&p2);
	vector_free(&temp);
	vector_free(&v1);
	vector_free(&v2);
	// make the filter
	if (unlikely(filter_alloc(f, numZ, denZ, dt)))
	{
		cli_printf("ERROR in filter_c2d_tustin, failed to alloc filter\n");
		vector_free(&numZ);
		vector_free(&denZ);
		return -1;
	}
	vector_free(&numZ);
	vector_free(&denZ);
	return 0;
}

int filter_normalize(filter_t *f)
{
	int i;
	double val;
	// sanity checks
	if (unlikely(!f->initialized))
	{
		cli_printf("ERROR in filter_normalize, vector uninitialized\n");
		return -1;
	}
	val = f->den.d[0];
	if (unlikely(fabs(val) < zero_tolerance))
	{
		cli_printf("ERROR in filter_normalize, leading coefficient is 0\n");
		return -1;
	}
	// if already normalized, just return
	if (fabs(val - 1.0) < zero_tolerance)
		return 0;
	for (i = 0; i < f->num.len; i++)
		f->num.d[i] /= val;
	for (i = 1; i < f->den.len; i++)
		f->den.d[i] /= val;
	f->den.d[0] = 1.0;
	return 0;
}

int filter_first_order_lowpass(filter_t *f, double dt, double time_constant)
{

	double c, num[2], den[2];
	// sanity checks
	if (unlikely(time_constant <= 0.0))
	{
		cli_printf("ERROR in filter_first_order_lowpass, time constant must be >0\n");
		return -1;
	}
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_first_order_lowpass, dt must be >0\n");
		return -1;
	}

	c = dt / time_constant;
	num[0] = c;
	num[1] = 0.0;
	den[0] = 1.0;
	den[1] = c - 1.0;

	// make the filter
	if (unlikely(filter_alloc_from_arrays(f, dt, num, 2, den, 2)))
	{
		cli_printf("ERROR in filter_first_order_lowpass, failed to alloc filter\n");
		return -1;
	}
	return 0;
}

int filter_first_order_highpass(filter_t *f, double dt, double time_constant)
{
	double c, num[2], den[2];

	// sanity checks
	if (unlikely(time_constant <= 0.0))
	{
		cli_printf("ERROR in filter_first_order_highpass, time constant must be >0\n");
		return -1;
	}
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_first_order_highpass, dt must be >0\n");
		return -1;
	}

	c = dt / time_constant;
	num[0] = 1.0 - c;
	num[1] = c - 1.0;
	den[0] = 1.0;
	den[1] = c - 1.0;

	// make the filter
	if (unlikely(filter_alloc_from_arrays(f, dt, num, 2, den, 2)))
	{
		cli_printf("ERROR in filter_first_order_highpass, failed to alloc filter\n");
		return -1;
	}
	return 0;
}

int filter_butterworth_lowpass(filter_t *f, int order, double dt, double wc)
{
	vector_t num = RC_VECTOR_INITIALIZER;
	vector_t den = RC_VECTOR_INITIALIZER;
	if (unlikely(order < 1))
	{
		cli_printf("ERROR in filter_butterworth_lowpass, order must be >=1\n");
		return -1;
	}
	if (unlikely(poly_butter(order, wc, &den)))
	{
		cli_printf("ERROR in filter_butterworth_lowpass, failed to find butterworth polynomial\n");
		return -1;
	}
	vector_alloc(&num, 1);
	num.d[0] = 1.0;
	if (unlikely(filter_c2d_tustin(f, dt, num, den, wc)))
	{
		cli_printf("ERROR in filter_butterworth_lowpass, failed to c2d_tustin\n");
		vector_free(&num);
		vector_free(&den);
		return -1;
	}
	vector_free(&num);
	vector_free(&den);
	return 0;
}

int filter_butterworth_highpass(filter_t *f, int order, double dt, double wc)
{
	vector_t num = RC_VECTOR_INITIALIZER;
	vector_t den = RC_VECTOR_INITIALIZER;
	if (unlikely(order < 1))
	{
		cli_printf("ERROR in filter_butterworth_highpass, order must be >=1\n");
		return -1;
	}
	if (unlikely(poly_butter(order, wc, &den)))
	{
		cli_printf("ERROR in filter_butterworth_highpass, failed to find butterwoth polynomial\n");
		return -1;
	}
	// numerator consists of all zeros at the origin
	vector_zeros(&num, order + 1);
	num.d[0] = 1.0;

	if (unlikely(filter_c2d_tustin(f, dt, num, den, wc)))
	{
		cli_printf("ERROR in filter_butterworth_highpass, failed to c2d_tustin\n");
		vector_free(&num);
		vector_free(&den);
		return -1;
	}
	vector_free(&num);
	vector_free(&den);
	return 0;
}

int filter_moving_average(filter_t *f, int samples, double dt)
{
	int i;
	vector_t num = RC_VECTOR_INITIALIZER;
	vector_t den = RC_VECTOR_INITIALIZER;
	// sanity checks
	if (unlikely(samples < 2))
	{
		cli_printf("ERROR in filter_moving_average, samples must be >=2\n");
		return -1;
	}
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_moving_average, dt must be >0\n");
		return -1;
	}
	vector_alloc(&num, samples);
	vector_alloc(&den, samples);
	// fill in coefficients
	for (i = 0; i < samples; i++)
	{
		num.d[i] = 1.0 / samples;
		den.d[i] = 0.0;
	}
	den.d[0] = 1.0;
	// make the filter
	if (unlikely(filter_alloc(f, num, den, dt)))
	{
		cli_printf("ERROR in filter_moving_average, failed to alloc filter\n");
		vector_free(&num);
		vector_free(&den);
		return -1;
	}
	f->dt = dt;
	vector_free(&num);
	vector_free(&den);
	return 0;
}

int filter_integrator(filter_t *f, double dt)
{
	vector_t num = RC_VECTOR_INITIALIZER;
	vector_t den = RC_VECTOR_INITIALIZER;
	// sanity checks
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_integrator, dt must be >0\n");
		return -1;
	}
	vector_alloc(&num, 1);
	vector_alloc(&den, 2);
	num.d[0] = dt;
	den.d[0] = 1.0;
	den.d[1] = -1.0;
	// make the filter
	if (unlikely(filter_alloc(f, num, den, dt)))
	{
		cli_printf("ERROR in filter_integrator, failed to alloc filter\n");
		vector_free(&num);
		vector_free(&den);
		return -1;
	}
	vector_free(&num);
	vector_free(&den);
	return 0;
}

int filter_double_integrator(filter_t *f, double dt)
{
	vector_t num = RC_VECTOR_INITIALIZER;
	vector_t den = RC_VECTOR_INITIALIZER;
	// sanity checks
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_double_integrator, dt must be >0\n");
		return -1;
	}
	vector_alloc(&num, 1);
	vector_alloc(&den, 3);
	num.d[0] = dt * dt;
	den.d[0] = 1.0;
	den.d[1] = -2.0;
	den.d[2] = 1.0;
	// make the filter
	if (unlikely(filter_alloc(f, num, den, dt)))
	{
		cli_printf("ERROR in filter_double_integrator, failed to alloc filter\n");
		vector_free(&num);
		vector_free(&den);
		return -1;
	}
	vector_free(&num);
	vector_free(&den);
	return 0;
}

int filter_pid(filter_t *f, double kp, double ki, double kd, double Tf, double dt)
{
	vector_t num = RC_VECTOR_INITIALIZER;
	vector_t den = RC_VECTOR_INITIALIZER;
	// sanity checks
	if (unlikely(dt < 0.0))
	{
		cli_printf("ERROR in filter_pid, dt must be >0\n");
		return -1;
	}
	if (unlikely(Tf <= dt / 2))
	{
		cli_printf("ERROR in filter_pid, Tf must be > dt/2 for stability\n");
		return -1;
	}

	// 1st order PD filter with rolloff
	if ((fabs(ki) < zero_tolerance) && (fabs(kd) > zero_tolerance))
	{
		vector_alloc(&num, 2);
		vector_alloc(&den, 2);
		num.d[0] = ((kp * Tf) + kd) / Tf;
		num.d[1] = ((kp * (dt - Tf)) - kd) / Tf;
		den.d[0] = 1.0;
		den.d[1] = -(Tf - dt) / Tf;
	}
	// 1st order PI filter
	else if ((fabs(ki) > zero_tolerance) && (fabs(kd) < zero_tolerance))
	{
		vector_alloc(&num, 2);
		vector_alloc(&den, 2);
		num.d[0] = kp;
		num.d[1] = ((ki * dt) - kp);
		den.d[0] = 1.0;
		den.d[1] = -1.0;
	}
	// 0th order proportional gain only
	else if ((fabs(ki) < zero_tolerance) && (fabs(kd) < zero_tolerance))
	{
		vector_alloc(&num, 1);
		vector_alloc(&den, 1);
		num.d[0] = kp;
		den.d[0] = 1.0;
	}
	//otherwise 2nd order PID with roll off
	else
	{
		vector_alloc(&num, 3);
		vector_alloc(&den, 3);
		num.d[0] = (kp * Tf + kd) / Tf;
		num.d[1] = (ki * dt * Tf + kp * (dt - Tf) - kp * Tf - 2.0 * kd) / Tf;
		num.d[2] = (((ki * dt - kp) * (dt - Tf)) + kd) / Tf;
		den.d[0] = 1.0;
		den.d[1] = (dt - (2.0 * Tf)) / Tf;
		den.d[2] = (Tf - dt) / Tf;
	}
	// make the filter
	if (unlikely(filter_alloc(f, num, den, dt)))
	{
		cli_printf("ERROR in filter_pid, failed to alloc filter\n");
		vector_free(&num);
		vector_free(&den);
		return -1;
	}
	vector_free(&num);
	vector_free(&den);
	return 0;
}

int filter_third_order_complement(filter_t *lp, filter_t *hp, double freq, double damp, double dt)
{
	int i;
	double a, b, c, t;
	vector_t den = RC_VECTOR_INITIALIZER;
	vector_t numlp = RC_VECTOR_INITIALIZER;
	vector_t numhp = RC_VECTOR_INITIALIZER;

	// sanity checks
	if (unlikely(freq <= 0.0))
	{
		cli_printf("ERROR in filter_third_order_complement, freq must be >0\n");
		return -1;
	}
	if (unlikely(damp <= 0.0))
	{
		cli_printf("ERROR in filter_third_order_complement, damp must be >0\n");
		return -1;
	}
	if (unlikely(dt <= 0.0))
	{
		cli_printf("ERROR in filter_third_order_complement, dt must be >0\n");
		return -1;
	}

	a = freq + 2.0 * damp * freq;
	b = (1.0 + 2.0 * damp) * freq * freq;
	c = freq * freq * freq;
	t = 2.0 / dt;

	vector_alloc(&den, 4);
	den.d[0] = t * t * t + a * t * t + b * t + c;
	den.d[1] = -3.0 * t * t * t - a * t * t + b * t + 3.0 * c;
	den.d[2] = 3.0 * t * t * t - a * t * t - b * t + 3.0 * c;
	den.d[3] = -t * t * t + a * t * t - b * t + c;

	vector_alloc(&numlp, 4);
	numlp.d[0] = b * t + c;
	numlp.d[1] = b * t + 3.0 * c;
	numlp.d[2] = -b * t + 3.0 * c;
	numlp.d[3] = -b * t + c;

	if (filter_alloc(lp, numlp, den, dt))
		return -1;
	if (filter_normalize(lp))
		return -1;

	// calculate hp as 1-lp
	vector_alloc(&numhp, 4);
	for (i = 0; i < 4; i++)
		numhp.d[i] = lp->den.d[i] - lp->num.d[i];

	filter_alloc(hp, numhp, lp->den, dt);

	vector_free(&den);
	vector_free(&numlp);
	vector_free(&numhp);
	return 0;
}

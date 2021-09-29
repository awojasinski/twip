/**
 * @headerfile math/vector.c
 *
 * @brief      A collection of hardware-accelerated linear algebra functions
 *             used heavily by the rest of the math API.
 *
 *             A small vector_t struct contains information about the
 *             vector's size and a pointer to where dynamically allocated memory
 *             exists that stores the actual data for the vector. Use
 *             vector_alloc to dynamically allocate memory for each new
 *             vector. Then use vector_free and to free the memory when you
 *             are done using it. See the remaining vector, matrix, and linear
 *             algebra functions for more details.
 *
 * @author     James Strawson
 * @date       2016
 */

#include <stdio.h>
#include <stdlib.h> // for malloc,calloc,free
#include <string.h> // for memcpy
#include <math.h>   // for sqrt, pow, etc
#include <float.h>  // for FLT_MAX DBL_MAX

#include <other.h>
#include <vector.h>
#include "algebra_common.h"

#include "cli.h"

int vector_alloc(vector_t *v, int length)
{
    // sanity checks
    if (unlikely(length < 1))
    {
        cli_printf("ERROR in vector_alloc, length must be >=1\n");
        return -1;
    }
    if (unlikely(v == NULL))
    {
        cli_printf("ERROR in vector_alloc, received NULL pointer\n");
        return -1;
    }
    // if v is already allocated and of the right size, nothing to do!
    if (v->initialized && v->len == length)
        return 0;
    // free any old memory
    vector_free(v);
    // allocate contiguous memory for the vector
    v->d = (double *)malloc(length * sizeof(double));
    if (unlikely(v->d == NULL))
    {
        cli_printf("ERROR in vector_alloc, not enough memory\n");
        return -1;
    }
    v->len = length;
    v->initialized = 1;
    return 0;
}

int vector_free(vector_t *v)
{
    vector_t new = RC_VECTOR_INITIALIZER;
    if (unlikely(v == NULL))
    {
        cli_printf("ERROR vector_free, received NULL pointer\n");
        return -1;
    }
    // free memory
    if (v->initialized)
        free(v->d);
    // zero out the struct
    *v = new;
    return 0;
}

vector_t vector_empty(void)
{
    vector_t out = RC_VECTOR_INITIALIZER;
    return out;
}

int vector_zeros(vector_t *v, int length)
{
    if (unlikely(length < 1))
    {
        cli_printf("ERROR in vector_zeros, length must be >=1\n");
        return -1;
    }
    if (unlikely(v == NULL))
    {
        cli_printf("ERROR in vector_zeros, received NULL pointer\n");
        return -1;
    }
    // free any old memory
    vector_free(v);
    // allocate contiguous zeroed-out memory for the vector
    v->d = (double *)calloc(length, sizeof(double));
    if (unlikely(v->d == NULL))
    {
        cli_printf("ERROR in vector_zeros, not enough memory\n");
        return -1;
    }
    v->len = length;
    v->initialized = 1;
    return 0;
}

int vector_ones(vector_t *v, int length)
{
    int i;
    if (unlikely(vector_alloc(v, length)))
    {
        cli_printf("ERROR in vector_ones, failed to allocate vector\n");
        return -1;
    }
    for (i = 0; i < length; i++)
        v->d[i] = 1.0f;
    return 0;
}

int vector_random(vector_t *v, int length)
{
    int i;
    if (unlikely(vector_alloc(v, length)))
    {
        cli_printf("ERROR vector_random, failed to allocate vector\n");
        return -1;
    }
    for (i = 0; i < length; i++)
        v->d[i] = get_random_double();
    return 0;
}

int vector_fibonnaci(vector_t *v, int length)
{
    int i;
    if (unlikely(vector_alloc(v, length)))
    {
        cli_printf("ERROR vector_fibonnaci, failed to allocate vector\n");
        return -1;
    }
    v->d[0] = 1.0f;
    if (length > 1)
        v->d[1] = 1.0f;
    for (i = 2; i < length; i++)
        v->d[i] = v->d[i - 1] + v->d[i - 2];
    return 0;
}

int vector_from_array(vector_t *v, double *ptr, int length)
{
    // sanity check pointer
    if (unlikely(ptr == NULL))
    {
        cli_printf("ERROR in vector_from_array, received NULL pointer\n");
        return -1;
    }
    // make sure there is enough space in v
    if (unlikely(vector_alloc(v, length)))
    {
        cli_printf("ERROR in vector_from_array, failed to allocate vector\n");
        return -1;
    }
    // duplicate memory over
    memcpy(v->d, ptr, length * sizeof(double));
    return 0;
}

int vector_duplicate(vector_t a, vector_t *b)
{
    // sanity check
    if (unlikely(!a.initialized))
    {
        cli_printf("ERROR in duplicate_vector, a not initialized\n");
        return -1;
    }
    // make sure there is enough space in b
    if (unlikely(vector_alloc(b, a.len)))
    {
        cli_printf("ERROR in duplicate_vector, failed to allocate vector\n");
        return -1;
    }
    // copy memory over
    memcpy(b->d, a.d, a.len * sizeof(double));
    return 0;
}

int vector_print(vector_t v)
{
    int i;
    if (unlikely(!v.initialized))
    {
        cli_printf("ERROR in vector_print, vector not initialized yet\n");
        return -1;
    }
    for (i = 0; i < v.len; i++)
        cli_printf("%7.4f  ", v.d[i]);
    cli_printf("\n");
    return 0;
}

int vector_print_sci(vector_t v)
{
    int i;
    if (unlikely(!v.initialized))
    {
        cli_printf("ERROR in vector_print_sci, vector not initialized yet\n");
        return -1;
    }
    for (i = 0; i < v.len; i++)
        cli_printf("%11.4e  ", v.d[i]);
    cli_printf("\n");
    return 0;
}

int vector_zero_out(vector_t *v)
{
    int i;
    if (unlikely(v->initialized != 1))
    {
        cli_printf("ERROR in vector_zero_out,vector not initialized yet\n");
        return -1;
    }
    for (i = 0; i < v->len; i++)
        v->d[i] = 0.0;
    return 0;
}

int vector_times_scalar(vector_t *v, double s)
{
    int i;
    if (unlikely(!v->initialized))
    {
        cli_printf("ERROR in vector_times_scalar, vector uninitialized\n");
        return -1;
    }
    for (i = 0; i < (v->len); i++)
        v->d[i] *= s;
    return 0;
}

double vector_norm(vector_t v, double p)
{
    double norm = 0.0f;
    int i;
    if (unlikely(!v.initialized))
    {
        cli_printf("ERROR in vector_norm, vector not initialized yet\n");
        return -1;
    }
    if (unlikely(p <= 0.0))
    {
        cli_printf("ERROR in vector_norm, p must be a positive real value\n");
        return -1;
    }
    // shortcut for 1-norm
    if (p < 1.001 && p > 0.999)
    {
        for (i = 0; i < v.len; i++)
            norm += fabs(v.d[i]);
        return norm;
    }
    // shortcut for 2-norm
    if (p < 2.001 && p > 1.999)
    {
        for (i = 0; i < v.len; i++)
            norm += v.d[i] * v.d[i];
        return sqrt(norm);
    }
    // generic norm formula, rarely used.
    for (i = 0; i < v.len; i++)
        norm += pow(fabs(v.d[i]), p);
    // take the pth root
    return pow(norm, (1.0 / p));
}

int vector_max(vector_t v)
{
    int i;
    int index = 0;
    double tmp = -DBL_MAX;
    if (unlikely(!v.initialized))
    {
        cli_printf("ERROR in vector_max, vector not initialized yet\n");
        return -1;
    }
    for (i = 0; i < v.len; i++)
    {
        if (v.d[i] > tmp)
        {
            index = i;
            tmp = v.d[i];
        }
    }
    return index;
}

int vector_min(vector_t v)
{
    int i;
    int index = 0;
    double tmp = DBL_MAX;
    if (unlikely(!v.initialized))
    {
        cli_printf("ERROR in vector_min, vector not initialized yet\n");
        return -1;
    }
    for (i = 0; i < v.len; i++)
    {
        if (v.d[i] < tmp)
        {
            index = i;
            tmp = v.d[i];
        }
    }
    return index;
}

double vector_std_dev(vector_t v)
{
    int i;
    double mean, mean_sqr, diff;
    if (unlikely(!v.initialized))
    {
        cli_printf("ERROR in vector_std_dev, vector not initialized yet\n");
        return -1.0f;
    }
    // shortcut for length 1
    if (v.len == 1)
        return 0.0f;
    // calculate mean
    mean = 0.0f;
    for (i = 0; i < v.len; i++)
        mean += v.d[i];
    mean = mean / (double)v.len;
    // calculate mean square
    mean_sqr = 0.0f;
    for (i = 0; i < v.len; i++)
    {
        diff = v.d[i] - mean;
        mean_sqr += diff * diff;
    }
    return sqrt(mean_sqr / (double)(v.len - 1));
}

double vector_mean(vector_t v)
{
    int i;
    double sum = 0.0f;
    if (unlikely(!v.initialized))
    {
        cli_printf("ERROR in vector_mean, vector not initialized yet\n");
        return -1.0f;
    }
    // calculate mean
    for (i = 0; i < v.len; i++)
        sum += v.d[i];
    return sum / (double)v.len;
}

int vector_projection(vector_t v, vector_t e, vector_t *p)
{
    int i;
    double factor;
    // sanity checks
    if (unlikely(!v.initialized || !e.initialized))
    {
        cli_printf("ERROR in vector_projection, received uninitialized vector\n");
        return -1;
    }
    if (unlikely(v.len != e.len))
    {
        cli_printf("ERROR in vector_projection, vectors not of same length\n");
        return -1;
    }
    if (unlikely(vector_alloc(p, v.len)))
    {
        cli_printf("ERROR in vector_projection, failed to allocate p\n");
        return -1;
    }
    factor = vector_dot_product(v, e) / vector_dot_product(e, e);
    for (i = 0; i < v.len; i++)
        p->d[i] = factor * e.d[i];
    return 0;
}

double vector_dot_product(vector_t v1, vector_t v2)
{
    if (unlikely(!v1.initialized || !v2.initialized))
    {
        cli_printf("ERROR in vector_dot_product, vector uninitialized\n");
        return -1.0f;
    }
    if (unlikely(v1.len != v2.len))
    {
        cli_printf("ERROR in vector_dot_product, dimension mismatch\n");
        return -1.0f;
    }
    return __vectorized_mult_accumulate(v1.d, v2.d, v1.len);
}

int vector_cross_product(vector_t v1, vector_t v2, vector_t *p)
{
    // sanity checks
    if (unlikely(!v1.initialized || !v2.initialized))
    {
        cli_printf("ERROR in vector_cross_product, vector not initialized yet.\n");
        return -1;
    }
    if (unlikely(v1.len != 3 || v2.len != 3))
    {
        cli_printf("ERROR in vector_cross_product, vector must have length 3\n");
        return -1;
    }
    if (unlikely(vector_alloc(p, 3)))
    {
        cli_printf("ERROR in vector_cross_product, failed to allocate p\n");
        return -1;
    }
    p->d[0] = (v1.d[1] * v2.d[2]) - (v1.d[2] * v2.d[1]);
    p->d[1] = (v1.d[2] * v2.d[0]) - (v1.d[0] * v2.d[2]);
    p->d[2] = (v1.d[0] * v2.d[1]) - (v1.d[1] * v2.d[0]);
    return 0;
}

int vector_sum(vector_t v1, vector_t v2, vector_t *s)
{
    int i;
    // sanity checks
    if (unlikely(!v1.initialized || !v2.initialized))
    {
        cli_printf("ERROR in vector_sum, received uninitialized vector\n");
        return -1;
    }
    if (unlikely(v1.len != v2.len))
    {
        cli_printf("ERROR in vector_sum, vectors not of same length\n");
        return -1;
    }
    if (unlikely(vector_alloc(s, v1.len)))
    {
        cli_printf("ERROR in vector_sum, failed to allocate s\n");
        return -1;
    }
    for (i = 0; i < v1.len; i++)
        s->d[i] = v1.d[i] + v2.d[i];
    return 0;
}

int vector_sum_inplace(vector_t *v1, vector_t v2)
{
    int i;
    // sanity checks
    if (unlikely(!v1->initialized || !v2.initialized))
    {
        cli_printf("ERROR in vector_sum_inplace, received uninitialized vector\n");
        return -1;
    }
    if (unlikely(v1->len != v2.len))
    {
        cli_printf("ERROR in vector_sum_inplace, vectors not of same length\n");
        return -1;
    }
    for (i = 0; i < v1->len; i++)
        v1->d[i] += v2.d[i];
    return 0;
}

int vector_subtract(vector_t v1, vector_t v2, vector_t *s)
{
    int i;
    // sanity checks
    if (unlikely(!v1.initialized || !v2.initialized))
    {
        cli_printf("ERROR in vector_substract, received uninitialized vector\n");
        return -1;
    }
    if (unlikely(v1.len != v2.len))
    {
        cli_printf("ERROR in vector_substract, vectors not of same length\n");
        return -1;
    }
    if (unlikely(vector_alloc(s, v1.len)))
    {
        cli_printf("ERROR in vector_substract, failed to allocate s\n");
        return -1;
    }
    for (i = 0; i < v1.len; i++)
        s->d[i] = v1.d[i] - v2.d[i];
    return 0;
}

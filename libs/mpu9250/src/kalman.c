/*
 * @file kalman.h
 *
 * @brief      Kalman filter implementation
 * @date       April 2018
 * @author     Eric Nauli Sihite & James Strawson
 */

#include <stdio.h>
#include "algebra.h"
#include "kalman.h"
#include "algebra_common.h"

#include "cli.h"

kalman_t kalman_empty(void)
{
	kalman_t kf = RC_KALMAN_INITIALIZER;
	return kf;
}

int kalman_alloc_lin(kalman_t *kf, matrix_t F, matrix_t G, matrix_t H, matrix_t Q, matrix_t R, matrix_t Pi)
{
	int Nx;

	// sanity checks
	if (kf == NULL)
	{
		cli_printf("ERROR in kalman_alloc_lin, received NULL pointer\n");
		return -1;
	}
	if (!F.initialized || !H.initialized)
	{
		cli_printf("ERROR in kalman_alloc, received uninitialized F or H\n");
		return -1;
	}
	if (!Q.initialized || !R.initialized)
	{
		cli_printf("ERROR in kalman_alloc, received initialized P or Q\n");
		return -1;
	}
	if (F.rows != F.cols)
	{
		cli_printf("ERROR in kalman_alloc, F must be square\n");
		return -1;
	}
	if (H.cols != F.cols)
	{
		cli_printf("ERROR in kalman_alloc, F and H must have same number of columns\n");
		return -1;
	}
	if (G.rows != F.rows)
	{
		cli_printf("ERROR in kalman_alloc, F and G must have same number of rows\n");
		return -1;
	}
	if (Q.rows != Q.cols)
	{
		cli_printf("ERROR in kalman_alloc_ekf, Q must be square\n");
		return -1;
	}
	if (R.rows != R.cols)
	{
		cli_printf("ERROR in kalman_alloc_ekf, R must be square\n");
		return -1;
	}

	// free existing memory, this also zero's out the struct
	if (kalman_free(kf) == -1)
		return -1;

	// allocate memory
	Nx = F.cols;
	if (matrix_duplicate(F, &kf->F) == -1)
		return -1;
	if (matrix_duplicate(G, &kf->G) == -1)
		return -1;
	if (matrix_duplicate(H, &kf->H) == -1)
		return -1;

	if (matrix_duplicate(Q, &kf->Q) == -1)
		return -1;
	if (matrix_duplicate(R, &kf->R) == -1)
		return -1;
	if (matrix_duplicate(Pi, &kf->P) == -1)
		return -1;
	if (matrix_duplicate(Pi, &kf->Pi) == -1)
		return -1;

	if (vector_zeros(&kf->x_est, Nx) == -1)
		return -1;
	if (vector_zeros(&kf->x_pre, Nx) == -1)
		return -1;
	kf->initialized = 1;
	return 0;
}

int kalman_alloc_ekf(kalman_t *kf, matrix_t Q, matrix_t R, matrix_t Pi)
{
	// sanity checks
	if (kf == NULL)
	{
		cli_printf("ERROR in kalman_alloc_ekf, received NULL pointer\n");
		return -1;
	}
	if (!Q.initialized || !R.initialized || !Pi.initialized)
	{
		cli_printf("ERROR in kalman_alloc_ekf, received uninitialized matrix\n");
		return -1;
	}
	if (Q.rows != Q.cols)
	{
		cli_printf("ERROR in kalman_alloc_ekf, Q must be square\n");
		return -1;
	}
	if (R.rows != R.cols)
	{
		cli_printf("ERROR in kalman_alloc_ekf, R must be square\n");
		return -1;
	}

	// free existing memory, this also zero's out the struct
	kalman_free(kf);

	// allocate memory
	matrix_duplicate(Q, &kf->Q);
	matrix_duplicate(R, &kf->R);
	matrix_duplicate(Pi, &kf->Pi);
	matrix_duplicate(Pi, &kf->P);
	vector_zeros(&kf->x_est, Q.rows);
	vector_zeros(&kf->x_pre, Q.rows);
	kf->initialized = 1;
	return 0;
}

int kalman_free(kalman_t *kf)
{
	kalman_t new = RC_KALMAN_INITIALIZER;
	// sanity checks
	if (kf == NULL)
	{
		cli_printf("ERROR in kalman_free, received NULL pointer\n");
		return -1;
	}
	matrix_free(&kf->F);
	matrix_free(&kf->G);
	matrix_free(&kf->H);

	matrix_free(&kf->Q);
	matrix_free(&kf->R);
	matrix_free(&kf->P);
	matrix_free(&kf->Pi);

	vector_free(&kf->x_est);
	vector_free(&kf->x_pre);

	*kf = new;
	return 0;
}

int kalman_reset(kalman_t *kf)
{
	// sanity checks
	if (kf == NULL)
	{
		cli_printf("ERROR in kalman_reset, received NULL pointer\n");
		return -1;
	}
	if (kf->initialized != 1)
	{
		cli_printf("ERROR in kalman_reset, kf uninitialized\n");
		return -1;
	}
	// set P to I*P_init
	matrix_duplicate(kf->Pi, &kf->P);
	vector_zero_out(&kf->x_est);
	vector_zero_out(&kf->x_pre);
	kf->step = 0;
	return 0;
}

int kalman_update_lin(kalman_t *kf, vector_t u, vector_t y)
{
	matrix_t L = RC_MATRIX_INITIALIZER;
	matrix_t newP = RC_MATRIX_INITIALIZER;
	matrix_t S = RC_MATRIX_INITIALIZER;
	matrix_t FT = RC_MATRIX_INITIALIZER;
	vector_t h = RC_VECTOR_INITIALIZER;
	vector_t z = RC_VECTOR_INITIALIZER;
	vector_t tmp1 = RC_VECTOR_INITIALIZER;
	vector_t tmp2 = RC_VECTOR_INITIALIZER;

	// sanity checks
	if (unlikely(kf == NULL))
	{
		cli_printf("ERROR in kalman_lin_update, received NULL pointer\n");
		return -1;
	}
	if (unlikely(kf->initialized != 1))
	{
		cli_printf("ERROR in kalman_lin_update, kf uninitialized\n");
		return -1;
	}
	if (unlikely(u.initialized != 1 || y.initialized != 1))
	{
		cli_printf("ERROR in kalman_lin_update received uninitialized vector\n");
		return -1;
	}
	if (unlikely(u.len != kf->G.cols))
	{
		cli_printf("ERROR in kalman_lin_update u must have same dimension as columns of G\n");
		return -1;
	}
	if (unlikely(y.len != kf->H.rows))
	{
		cli_printf("ERROR in kalman_lin_update y must have same dimension as rows of H\n");
		return -1;
	}

	// for linear case only, calculate x_pre from linear system model
	// x_pre = x[k|k-1] = F*x[k-1|k-1] +  G*u[k-1]
	matrix_times_col_vec(kf->F, kf->x_est, &tmp1);
	matrix_times_col_vec(kf->G, u, &tmp2);
	vector_sum(tmp1, tmp2, &kf->x_pre);

	// F is constant in this linear case
	// P[k|k-1] = F*P[k-1|k-1]*F^T + Q
	matrix_multiply(kf->F, kf->P, &newP); // newP = F*P_old
	matrix_transpose(kf->F, &FT);
	matrix_right_multiply_inplace(&newP, FT); // P = F*P*F^T
	matrix_add_inplace(&newP, kf->Q);		  // P = F*P*F^T + Q
	matrix_symmetrize(&newP);				  // Force symmetric P

	// h[k] = H * x_pre[k]
	matrix_times_col_vec(kf->H, kf->x_pre, &h);

	// H is constant in the linear case
	// S = H*P*H^T + R
	// Calculate H^T, borrow S for H^T
	matrix_transpose(kf->H, &S); // S = H^T
	// Calculate a part of L in advance before we modify S = H^T
	matrix_multiply(newP, S, &L);			 // K = P*(H^T)
	matrix_left_multiply_inplace(newP, &S);	 // S = P*H^T
	matrix_left_multiply_inplace(kf->H, &S); // S = H*(P*H^T)
	matrix_add_inplace(&S, kf->R);			 // S = H*P*H^T + R

	// L = P*(H^T)*(S^-1)
	algebra_invert_matrix_inplace(&S);	  // S2^(-1) = S^(-1)
	matrix_right_multiply_inplace(&L, S); // L = (P*H^T)*(S^-1)

	// x[k|k] = x[k|k-1] + K[k]*(y[k]-h[k])
	vector_subtract(y, h, &z);				 // z = k-h
	matrix_times_col_vec(L, z, &tmp1);		 // temp = L*z
	vector_sum(kf->x_pre, tmp1, &kf->x_est); // x_est = x + K*y

	// P[k|k] = (I - L*H)*P = P[k|k-1] - L*H*P[k|k-1], reuse the matrix S.
	matrix_multiply(kf->H, newP, &S);	 // S = H*P
	matrix_left_multiply_inplace(L, &S); // S = L*(H*P)
	matrix_subtract_inplace(&newP, S);	 // P = P - K*H*P
	matrix_symmetrize(&newP);			 // Force symmetric P
	matrix_duplicate(newP, &kf->P);

	// cleanup
	matrix_free(&L);
	matrix_free(&newP);
	matrix_free(&S);
	matrix_free(&FT);
	vector_free(&h);
	vector_free(&z);
	vector_free(&tmp1);
	vector_free(&tmp2);

	kf->step++;
	return 0;
}

int kalman_update_ekf(kalman_t *kf, matrix_t F, matrix_t H, vector_t x_pre, vector_t y, vector_t h)
{
	matrix_t L = RC_MATRIX_INITIALIZER;
	matrix_t newP = RC_MATRIX_INITIALIZER;
	matrix_t S = RC_MATRIX_INITIALIZER;
	matrix_t FT = RC_MATRIX_INITIALIZER;
	vector_t z = RC_VECTOR_INITIALIZER;
	vector_t tmp1 = RC_VECTOR_INITIALIZER;
	vector_t tmp2 = RC_VECTOR_INITIALIZER;

	// sanity checks
	if (unlikely(kf == NULL))
	{
		cli_printf("ERROR in kalman_ekf_update, received NULL pointer\n");
		return -1;
	}
	if (unlikely(kf->initialized != 1))
	{
		cli_printf("ERROR in kalman_ekf_update, kf uninitialized\n");
		return -1;
	}
	if (unlikely(F.initialized != 1 || H.initialized != 1))
	{
		cli_printf("ERROR in kalman_ekf_update received uninitialized matrix\n");
		return -1;
	}
	if (unlikely(x_pre.initialized != 1 || y.initialized != 1 || h.initialized != 1))
	{
		cli_printf("ERROR in kalman_ekf_update received uninitialized vector\n");
		return -1;
	}
	if (unlikely(F.rows != F.cols))
	{
		cli_printf("ERROR in kalman_ekf_update F must be square\n");
		return -1;
	}
	if (unlikely(x_pre.len != F.rows))
	{
		cli_printf("ERROR in kalman_ekf_update x_pre must have same dimension as rows of F\n");
		return -1;
	}
	if (unlikely(x_pre.len != H.cols))
	{
		cli_printf("ERROR in kalman_ekf_update x_pre must have same dimension as columns of H\n");
		return -1;
	}
	if (unlikely(y.len != H.rows))
	{
		cli_printf("ERROR in kalman_ekf_update y must have same dimension as rows of H\n");
		return -1;
	}
	if (unlikely(y.len != h.len))
	{
		cli_printf("ERROR in kalman_ekf_update y must have same dimension h\n");
		return -1;
	}

	// copy in new jacobians and x prediction
	matrix_duplicate(F, &kf->F);
	vector_duplicate(x_pre, &kf->x_pre);
	matrix_duplicate(H, &kf->H);

	// F is new now in non-linear case
	// P[k|k-1] = F*P[k-1|k-1]*F^T + Q
	matrix_multiply(kf->F, kf->P, &newP); // P_new = F*P_old
	matrix_transpose(kf->F, &FT);
	matrix_right_multiply_inplace(&newP, FT); // P = F*P*F^T
	matrix_add(newP, kf->Q, &kf->P);		  // P = F*P*F^T + Q
	matrix_symmetrize(&kf->P);				  // Force symmetric P

	// H is constant in the linear case
	// S = H*P*H^T + R
	// Calculate H^T, borrow S for H^T
	matrix_transpose(kf->H, &S); // S = H^T
	// Calculate a part of L in advance before we modify S = H^T
	matrix_multiply(kf->P, S, &L);			 // K = P*(H^T)
	matrix_left_multiply_inplace(kf->P, &S); // S = P*H^T
	matrix_left_multiply_inplace(kf->H, &S); // S = H*(P*H^T)
	matrix_add_inplace(&S, kf->R);			 // S = H*P*H^T + R

	// L = P*(H^T)*(S^-1)
	algebra_invert_matrix_inplace(&S);	  // S2^(-1) = S^(-1)
	matrix_right_multiply_inplace(&L, S); // L = (P*H^T)*(S^-1)

	// x[k|k] = x[k|k-1] + L[k]*(y[k]-h[k])
	vector_subtract(y, h, &z);				 // z = k-h
	matrix_times_col_vec(L, z, &tmp1);		 // temp = L*z
	vector_sum(kf->x_pre, tmp1, &kf->x_est); // x_est = x + L*y

	// P[k|k] = (I - L*H)*P = P - L*H*P, reuse the matrix S.
	matrix_multiply(kf->H, newP, &S);	 // S = H*P
	matrix_left_multiply_inplace(L, &S); // S = L*(H*P)
	matrix_subtract_inplace(&newP, S);	 // P = P - K*H*P
	matrix_symmetrize(&newP);			 // Force symmetric P
	matrix_duplicate(newP, &kf->P);

	// cleanup
	matrix_free(&L);
	matrix_free(&newP);
	matrix_free(&S);
	matrix_free(&FT);
	vector_free(&z);
	vector_free(&tmp1);
	vector_free(&tmp2);

	kf->step++;
	return 0;
}

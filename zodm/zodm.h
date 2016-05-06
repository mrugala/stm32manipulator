#ifndef __ZODM_H__
#define __ZODM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "matrix.h"
#include "stdbool.h"

typedef struct
{
	float alpha_deg;
	float beta_deg;
	float gamma_deg;
} input_angles_t;

typedef struct
{
	float x_mm;
	float y_mm;
	float z_mm;
} input_coord_t;

#define ZODM_ERROR_CODE(x) \
		x(ZODM_RETURN_OK) \
		x(NULL_INPUT_ERROR) \
		x(NULL_Q_VECT_ERROR) \
		x(ZODM_ROTATION_ERROR) \
		x(ZODM_TOP) \

#define C(x) x,

typedef enum
{
	ZODM_ERROR_CODE(C)
} zodm_error_t;

#undef C

/*
 * Calculate ZODM for 5 degrees of freedom using angles and coordinates.
 */
zodm_error_t calc_zodm_5_angles(input_angles_t* in_angles, input_coord_t* in_coord, 
	float q_vect[/*static*/ 5]);

/*
 * Calculate ZODM for 5 degrees of freedom using rotation matrix and coordinates.
 */
zodm_error_t calc_zodm_5_rot(matrix_t* rot, input_coord_t* in_coord, float q_vect[/*static*/ 5]);

bool assert_equal_f(float a, float b, float epsilon);

#ifdef __cplusplus
}
#endif

#endif /*__ZODM_H__*/
#include "zodm.h"
#include <stdbool.h>
#include <stdio.h>
#include "comm.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define C(x) #x,

__attribute__((unused)) static const char* zodm_error_str[] = {
	ZODM_ERROR_CODE(C)
};

#undef C

typedef struct
{
	/*float a1;*/
	float a2;
	/*float a3;*/
	/*float a4;*/
	/*float a5;*/
	/*float d1;*/
	/*float d2;*/
	/*float d3;*/
	float d4;
	/*float d5;*/
} ZDH_t;

static ZDH_t ZDH = { 120, 100 };

bool assert_equal_f(float a, float b, float epsilon)
{
	float diff = a - b;
	return (diff < epsilon) && (-diff < epsilon);
}

zodm_error_t calc_zodm_5_angles(input_angles_t* in_angles, input_coord_t* in_coord, float q_vect[/*static*/ 5])
{
	if (!in_angles || !in_coord)
		return NULL_INPUT_ERROR;

	if (!q_vect)
		return NULL_Q_VECT_ERROR;

	matrix_t rot = { 0 };
	if (get_rotation_matrix(in_angles->alpha_deg, in_angles->beta_deg, in_angles->gamma_deg, &rot))
		return ZODM_ROTATION_ERROR;

	return calc_zodm_5_rot(&rot, in_coord, q_vect);
}

zodm_error_t calc_zodm_5_rot(matrix_t* rot, input_coord_t* in_coord, float q_vect[/*static*/ 5])
{
	if (!rot || !in_coord)
		return NULL_INPUT_ERROR;

	if (!q_vect)
		return NULL_Q_VECT_ERROR;

	float len = sqrtf(powf(in_coord->x_mm, 2) + powf(in_coord->y_mm, 2) + powf(in_coord->z_mm, 2));

	if (len > ZDH.a2 + ZDH.d4)
	{
		in_coord->x_mm *= (ZDH.a2 + ZDH.d4) / len;
		in_coord->y_mm *= (ZDH.a2 + ZDH.d4) / len;
		in_coord->z_mm *= (ZDH.a2 + ZDH.d4) / len;
		len = sqrtf(powf(in_coord->x_mm, 2) + powf(in_coord->y_mm, 2) + powf(in_coord->z_mm, 2));
	}

	q_vect[0] = atan2f(in_coord->y_mm, in_coord->x_mm);
	float temp = *access_matrix_cell(rot, 1, 2) * cosf(q_vect[0])
		- *access_matrix_cell(rot, 0, 2) * sinf(q_vect[0]);
	q_vect[3] = atan2f(sqrtf(1 - powf(temp, 2)), temp);

	if (!assert_equal_f(sinf(q_vect[3]), 0, (float)0.0001))
	{
		float q12 = atan2f(*access_matrix_cell(rot, 2, 2), *access_matrix_cell(rot, 1, 0) 
			* cosf(q_vect[0]) + *access_matrix_cell(rot, 0, 0) * sinf(q_vect[0]));
		temp = -ZDH.d4 * sinf(q12) + in_coord->x_mm * cosf(q_vect[0]) + in_coord->y_mm * sinf(q_vect[0]);
		q_vect[1] = atan2f(ZDH.d4 * cosf(q12) - in_coord->z_mm, temp);
		temp = (powf(in_coord->x_mm, 2) + powf(in_coord->y_mm, 2)
			+ powf(in_coord->z_mm, 2) - powf(ZDH.a2, 2) - powf(ZDH.d4, 2)) / (2 * ZDH.a2 * ZDH.d4);
		q_vect[2] = atan2f(temp, sqrtf(1 - powf(temp, 2)));
		q_vect[4] = atan2f((-*access_matrix_cell(rot, 2, 0) * cosf(q12) - sinf(q12))
			* (*access_matrix_cell(rot, 0, 0) * cosf(q_vect[0]) + *access_matrix_cell(rot, 1, 0) 
			* sinf(q_vect[0])), (-*access_matrix_cell(rot, 2, 1) * cosf(q12) - sinf(q12))
			* (*access_matrix_cell(rot, 0, 1) * cosf(q_vect[0]) + *access_matrix_cell(rot, 1, 1) 
			* sinf(q_vect[0])));
	}
	else
	{
		float q124 = atan2f(*access_matrix_cell(rot, 2, 0), *access_matrix_cell(rot, 2, 1));
		temp = (powf(in_coord->x_mm, 2) + powf(in_coord->y_mm, 2) + powf(in_coord->z_mm, 2) - powf(ZDH.a2, 2)
			- powf(ZDH.d4, 2)) / (2 * ZDH.a2 * ZDH.d4);
		q_vect[2] = atan2f(sqrtf(1 - powf(temp, 2)), temp);
		q_vect[1] = atan2f(in_coord->z_mm, sqrtf(powf(in_coord->x_mm, 2) + powf(in_coord->y_mm, 2)))
			+ atan2f(ZDH.d4 * sinf(q_vect[2]), ZDH.a2 + ZDH.d4 * cosf(q_vect[2]));
		q_vect[4] = q124 - q_vect[1] - q_vect[2];
		printToUSART(USART3, "przypadek osobliwy");
	}

	int i;
	for (i = 0; i < 5; i++)
	{
		q_vect[i] *= (180 / (float)M_PI);
	}

	return ZODM_RETURN_OK;
}

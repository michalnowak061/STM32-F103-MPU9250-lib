/*
 * quaternion.h
 *
 *  Created on: Dec 19, 2019
 *      Author: macbookmichal
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "math.h"

struct quaternion {

	float w;
	float x;
	float y;
	float z;
};

struct euler {

	float roll;
	float pitch;
	float yaw;
};

struct rot_matrix {

	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
};

struct vector {

	float x;
	float y;
	float z;
};

void quaternion_init(struct quaternion *q);

void euler_init(struct euler *e);

struct quaternion quaternion_tensor_product(struct quaternion q1, struct quaternion q2);

void quaternion_normalise(struct quaternion *q);

void quaternion_to_matrix(struct quaternion *q, struct rot_matrix *m);

void matrix_to_euler(struct rot_matrix *m, struct euler *e);

#endif /* INC_QUATERNION_H_ */

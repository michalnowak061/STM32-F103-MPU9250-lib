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

	double w;
	double x;
	double y;
	double z;
};

struct euler {

	float roll;
	float pitch;
	float yaw;
};

struct rot_matrix {

	double m11, m12, m13;
	double m21, m22, m23;
	double m31, m32, m33;
};

void quaternion_init(struct quaternion *q);

void euler_init(struct euler *e);

struct quaternion quaternion_tensor_product(struct quaternion *q1, struct quaternion *q2);

void quaternion_normalise(struct quaternion *q);

void quaternion_to_matrix(struct quaternion *q, struct rot_matrix *m);

void matrix_to_euler(struct rot_matrix *m, struct euler *e);

#endif /* INC_QUATERNION_H_ */

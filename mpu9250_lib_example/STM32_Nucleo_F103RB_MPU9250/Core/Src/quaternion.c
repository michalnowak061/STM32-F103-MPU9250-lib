/*
 * quaternion.c
 *
 *  Created on: Dec 19, 2019
 *      Author: macbookmichal
 */

#include "quaternion.h"

void quaternion_init(struct quaternion *q) {

	q->w = 1;
	q->x = 0;
	q->y = 0;
	q->z = 0;
}

void euler_init(struct euler *e) {

	e->roll = 0;
	e->pitch = 0;
	e->yaw = 0;
}

struct quaternion quaternion_tensor_product(struct quaternion *q1, struct quaternion *q2) {

	struct quaternion tensor_product;

	tensor_product.w = (q1->w * q2->w) - (q1->x * q2->x) - (q1->y * q2->y) - (q1->z * q2->z);
	tensor_product.x = (q1->w * q2->x) + (q1->x * q2->w) + (q1->y * q2->z) - (q1->z * q2->y);
	tensor_product.y = (q1->w * q2->y) - (q1->x * q2->z) + (q1->y * q2->w) + (q1->z * q2->x);
	tensor_product.z = (q1->w * q2->z) + (q1->x * q2->y) - (q1->y * q2->x) + (q1->z * q2->w);

	return tensor_product;
}

void quaternion_normalise(struct quaternion *q) {

	double norm = sqrt( pow(q->w,2) + pow(q->x,2) + pow(q->y,2) + pow(q->z,2) );

	q->w = q->w / norm;
	q->x = q->x / norm;
	q->y = q->y / norm;
	q->z = q->z / norm;
}

void quaternion_to_matrix(struct quaternion *q, struct rot_matrix *m) {

	m->m11 = 1 - ( 2 * pow(q->y,2) ) - ( 2 * pow(q->z,2) );
	m->m12 = ( 2 * q->x * q->y ) - ( 2 * q->z * q->w );
	m->m13 = ( 2 * q->x * q->z ) + ( 2 * q->y * q->w );

	m->m21 = ( 2 * q->x * q->y ) + ( 2 * q->z * q->w );
	m->m22 = 1 - ( 2 * pow(q->x,2) ) - ( 2 * pow(q->z,2) );
	m->m23 = ( 2 * q->y * q->z ) - ( 2 * q->x * q->w );

	m->m31 = ( 2 * q->x * q->z ) - ( 2 * q->y * q->w );
	m->m32 = ( 2 * q->y * q->z ) + ( 2 * q->x * q->w );
	m->m33 = 1 - ( 2 * pow(q->x,2) ) - ( 2 * pow(q->y,2) );
}

void matrix_to_euler(struct rot_matrix *m, struct euler *e) {

	double Beta = atan2f(-m->m31, sqrt( pow(m->m11,2) + pow(m->m21,2) ) );

	e->pitch = Beta * (180 / M_PI);
	e->roll  = atan2(m->m32 / cosf(Beta), m->m33 / cos(Beta) ) * (180 / M_PI);
	e->yaw   = atan2(m->m21 / cosf(Beta), m->m11 / cos(Beta) ) * (180 / M_PI);
}

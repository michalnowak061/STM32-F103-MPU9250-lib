/*
 * kalman.c
 *
 *  Created on: 29.08.2019
 *  Author: mnowak5
 */

#include "kalman.h"

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void Kalman_filter_init(struct Kalman *k, double q, double r) {

	// stan
	k->stan_theta = 0;
	k->stan_omega = 0;
	k->stan_pomiar_theta = 0;
	k->stan_pomiar_omega = 0;

	// kalman
	k->kalman_Q = q;
	k->kalman_R = r;

	k->kalman_theta = 0;
	k->kalman_omega = 0;
	k->kalman_g_bias = 0;

	k->kalman_P11 = k->kalman_R;
	k->kalman_P13 = 0;
	k->kalman_P21 = 0;
	k->kalman_P31 = 0;
	k->kalman_P33 = 0;

	k->kalman_K1 = 0;
	k->kalman_K2 = 0;
	k->kalman_K3 = 0;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

float Kalman_filter_calculate(struct Kalman *k, float Acce, float Gyro, float dt) {

	// pomiar
	k->stan_pomiar_theta = Acce;
	k->stan_pomiar_omega = Gyro;

	// predykcja
	k->kalman_theta = k->kalman_theta + ( k->stan_pomiar_omega - k->kalman_g_bias ) * dt;
	k->kalman_omega = k->stan_pomiar_omega - k->kalman_g_bias;

	k->kalman_P11 = k->kalman_P11 - k->kalman_P31 * dt + k->kalman_P33 * dt * dt - k->kalman_P13 * dt + k->kalman_Q;
	k->kalman_P13 = k->kalman_P13 - k->kalman_P33 * dt;
	k->kalman_P21 = k->kalman_P33 * dt - k->kalman_P31;
	k->kalman_P31 = k->kalman_P31 - k->kalman_P33 * dt;
	k->kalman_P33 = k->kalman_P33 + k->kalman_Q;

	// korekcja
	k->kalman_K1 = k->kalman_P11 * ( 1 / ( k->kalman_P11 + k->kalman_R ) );
	k->kalman_K2 = k->kalman_P21 * ( 1 / ( k->kalman_P11 + k->kalman_R ) );
	k->kalman_K3 = k->kalman_P31 * ( 1 / ( k->kalman_P11 + k->kalman_R ) );

	k->kalman_theta  = k->kalman_theta  + k->kalman_K1 * ( k->stan_pomiar_theta - k->kalman_theta);
	k->kalman_omega  = k->kalman_omega  + k->kalman_K2 * ( k->stan_pomiar_theta - k->kalman_theta);
	k->kalman_g_bias = k->kalman_g_bias + k->kalman_K3 * ( k->stan_pomiar_theta - k->kalman_theta);

	// a posteriori
	k->kalman_P11 = ( 1 - k->kalman_K1 ) * k->kalman_P11;
	k->kalman_P13 = ( 1 - k->kalman_K1 ) * k->kalman_P13;
	k->kalman_P21 = k->kalman_P21 - k->kalman_P11 * k->kalman_K2;
	k->kalman_P31 = k->kalman_P31 - k->kalman_P11 * k->kalman_K3;
	k->kalman_P33 = k->kalman_P33 - k->kalman_P13 * k->kalman_K3;

	// aktualizacja wektora stanu
	k->stan_theta = k->kalman_theta;
	k->stan_omega = k->kalman_omega;

	return k->stan_theta;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/*
 * kalman.h
 *
 *  Created on: 29.08.2019
 *      Author: mnowak5
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

struct Kalman {

	double stan_theta; /* odchylenie */
	double stan_omega; /* predkosc odchylania */
	double stan_pomiar_theta; /* odchylenie */
	double stan_pomiar_omega; /* predkosc katowa */

	double kalman_Q; /* Wartosci przekatnej macierzy Q */
	double kalman_R; /* Wariancja pomiaru */
	double kalman_theta; /* odchylenie */
	double kalman_omega; /* predkosc katowa */
	double kalman_g_bias; /* dryft zyroskopu */
	double kalman_P11; /* wartosci macierzy kowariancji P */
	double kalman_P13;
	double kalman_P21;
	double kalman_P31;
	double kalman_P33;
	double kalman_K1; /* wartosci macierzy K */
	double kalman_K2;
	double kalman_K3;
};

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

void Kalman_filter_init(struct Kalman *k, double q, double r);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

float Kalman_filter_calculate(struct Kalman *k, float Acce, float Gyro, float dt);

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

#endif /* INC_KALMAN_H_ */

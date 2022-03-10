#pragma once
#define _USE_MATH_DEFINES
#include "Eigen/Eigen"
#include <random>

static double distance_coefficient = 2;// / 2.2;
static double gamma_correction_coefficient = 2.2;
static double P_RR = 0.8;

inline double fast_power(double x, int n)
{
	if (n == 0) return 1;
	if (n == 1) return x;
	double y = fast_power(x, n / 2);
	double res = y * y;
	if (n % 2 == 1)
		res = res * x;
	return res;
}

inline double fast_power(double x, double n)
{
	int c = std::floor(n);
	double res = n - c;
	return fast_power(x, c) * std::pow(x, res);
}

inline double get_area(Eigen::Vector3d v[3])
{
	double area = 0;
	Eigen::Vector3d e[2] = { v[1] - v[0], v[2] - v[0] };
	//Eigen::Vector3d normal = e[0].cross(e[1]).normalized();
	area = e[0].cross(e[1]).norm() * 0.5;
	return area;
}

inline double get_random_double()
{
	static std::random_device dev;
	static std::mt19937 rng(dev());
	static std::uniform_real_distribution<double> dist(0.0, 1.0); // distribution in range [1, 6]

	return dist(rng);
}



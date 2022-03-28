#pragma once
#define _USE_MATH_DEFINES
#include "Eigen/Eigen"
#include <random>

// global defintions

static double distance_coefficient = 2;// / 2.2;
static double gamma_correction_coefficient = 2.2;
static double P_RR = 0.8;

//double MIS_alpha = -2;
//double specular_coefficient = 0;
//int shading_depth = -1;

//extern double MIS_alpha;
//extern double specular_coefficient;
//extern int shading_depth;

static double MIS_alpha = -2;
static double specular_coefficient = 0;
static int shading_depth = -1;
static int color_type = -1;
static int kd_distribution = -1;
static int specular_exists = -1;
static int specular_large_coefficient = -1;

static int sample_dir_light = -1;
static int sample_all_light = -1;
static int sample_by_group = -1;

static int one_light_pdf_choice = -1;
static int group_light_pdf_choice = -1;
static int all_light_pdf_choice = -1;

static int MIS_Diffuse = -1;
static int MIS_BRDF = -1;
static int MIS_Method = -1;
static int MIS_correction_coefficient = -1;
static int MIS_unhit_sample = -1;
static int MIS_sample_area = -1;

static int MIS_triangle_unhit_sample;
static int MIS_specular_unhit_sample;
static int MIS_diffuse_unhit_sample;
static int specular_unhit_continue;
static int diffuse_unhit_continue;

static int I_think_correct = -1;
static int I_think_correct_further = -1;
static int _paper = -1;
static double divide_coefficient = -1;
static int normalize_rho_s = -1;
static int control_rho_s = -1;
static int control_K = -1;
static bool _SAH = false;
static int refraction_or_not = -1;
static int sphere_correction = -1;

inline double fast_power(double x, int n) // fast power to accelerate
{
	if (n == 0) return 1;
	if (n == 1) return x;
	double y = fast_power(x, n / 2);
	double res = y * y;
	if (n % 2 == 1)
		res = res * x;
	return res;
}

inline double fast_power(double x, double n) // fast power to accelerate
{
	int c = std::floor(n);
	double res = n - c;
	return fast_power(x, c) * std::pow(x, res);
}

inline double get_area(Eigen::Vector3d v[3]) // get area of triangle mesh
{
	double area = 0;
	Eigen::Vector3d e[2] = { v[1] - v[0], v[2] - v[0] };
	//Eigen::Vector3d normal = e[0].cross(e[1]).normalized();
	area = e[0].cross(e[1]).norm() * 0.5;
	return area;
}

inline double get_random_double() // get double random in [0, 1];
{
	static std::random_device dev;
	static std::mt19937 rng(dev());
	static std::uniform_real_distribution<double> dist(0.0, 1.0);

	return dist(rng);
}



#define _USE_MATH_DEFINES
#include <random>
inline double get_random_double()
{
	static std::random_device dev;
	static std::mt19937 rng(dev());
	static std::uniform_real_distribution<double> dist(0.0, 1.0); // distribution in range [1, 6]

	return dist(rng);
}

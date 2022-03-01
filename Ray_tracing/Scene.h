#pragma once
#include <Eigen/Eigen>
#include "Light.h"
struct Scene
{
	Eigen::Vector3d eye_value;
	Eigen::Vector3d lookat_value;
	Eigen::Vector3d up_value;
	double fovy_value;
	int width_value;
	int height_value;
	std::vector<Light> lights;
	Scene()
	{
		eye_value = Eigen::Vector3d(0, 1, 6.8);
		lookat_value = Eigen::Vector3d(0, 1, 5.8);
		up_value = Eigen::Vector3d(0, 1, 0);
		fovy_value = 19.5;
		width_value = 1024 / 2;
		height_value = 1024 / 2;
		lights.push_back(Light("Light", Eigen::Vector3d(17, 12, 4)));
	}
};
#pragma once
#include "Eigen/Eigen"

struct Light
{
	std::string name = ""; // name of light mesh
	Eigen::Vector3d Radiance = Eigen::Vector3d::Zero(); // radiance of light mesh
	Light(std::string name, Eigen::Vector3d Radiance) // initialize
	{
		this->name = name;
		this->Radiance = Radiance;
	}
	Light()
	{

	}
};
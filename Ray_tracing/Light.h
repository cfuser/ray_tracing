#pragma once
#include "Eigen/Eigen"

struct Light
{
	std::string name = "";
	Eigen::Vector3d Radiance = Eigen::Vector3d::Zero();
	Light(std::string name, Eigen::Vector3d Radiance)
	{
		this->name = name;
		this->Radiance = Radiance;
	}
	Light()
	{

	}
};
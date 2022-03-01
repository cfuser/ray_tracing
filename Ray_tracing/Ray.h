#pragma once
#include "Eigen/Eigen"
#include "TriangleMesh.h"

struct Ray
{
	Eigen::Vector3d origin;
	Eigen::Vector3d direction, inv_direction;
	TriangleMesh *mesh;
	Ray(Eigen::Vector3d origin, Eigen::Vector3d direction)
	{
		this->origin = origin;
		this->direction = direction;
		this->direction = this->direction.normalized();
		this->inv_direction = Eigen::Vector3d(1.0 / direction[0], 1.0 / direction[1], 1.0 / direction[2]);
		this->mesh = nullptr;
	}
};
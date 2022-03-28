#pragma once
#include "Eigen/Eigen"
#include "TriangleMesh.h"

struct Ray
{
	Eigen::Vector3d origin; // ray position
	Eigen::Vector3d direction, inv_direction; // ray direction
	TriangleMesh *mesh; // mesh
	double Ni; // Ni
	bool in_material = 0; // in Ni_material or not, 1 for true, 0 for false
	Ray(Eigen::Vector3d origin, Eigen::Vector3d direction) //initialize
	{
		this->origin = origin;
		this->direction = direction;
		this->direction = this->direction.normalized();
		this->inv_direction = Eigen::Vector3d(1.0 / direction[0], 1.0 / direction[1], 1.0 / direction[2]);
		this->mesh = nullptr;
		this->Ni = -1;
		this->in_material = 0;
	}
};
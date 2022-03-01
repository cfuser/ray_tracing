#pragma once
#include "Eigen/Eigen"
#include "TriangleMesh.h"

struct Intersection
{
	bool happened;
	Eigen::Vector3d coords;
	Eigen::Vector3d normal;
	Eigen::Vector2d tex_coords;
	double distance;
	TriangleMesh *mesh;
	Intersection()
	{
		happened = false;
		coords = Eigen::Vector3d(-1, -1, -1);
		normal = Eigen::Vector3d(0, 0, 0);
		tex_coords = Eigen::Vector2d(-1, -1);
		distance = DBL_MAX;
		mesh = nullptr;
	}
};
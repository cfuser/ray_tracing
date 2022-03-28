#pragma once
#include "Eigen/Eigen"
#include "TriangleMesh.h"

struct Intersection
{
	bool happened; // intersection or not, 1 for happened, 0 for unhappened
	Eigen::Vector3d coords; // intersection coordinates
	Eigen::Vector3d normal; // intersection normal
	Eigen::Vector2d tex_coords; // intersection texture coordinates
	double distance; // intersection distance from ray position to intersection coordinates
	TriangleMesh *mesh; // point to intetrsected triangle mesh
	Intersection() // initialize
	{
		happened = false;
		coords = Eigen::Vector3d(-1, -1, -1);
		normal = Eigen::Vector3d(0, 0, 0);
		tex_coords = Eigen::Vector2d(-1, -1);
		distance = DBL_MAX;
		mesh = nullptr;
	}
};
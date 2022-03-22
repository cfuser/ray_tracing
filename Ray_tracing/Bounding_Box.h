#pragma once
#include "Ray.h"
#include "iostream"
#include "TriangleMesh.h"

struct Bounding_Box
{
	double x_min = DBL_MAX, x_max = -DBL_MAX;
	double y_min = DBL_MAX, y_max = -DBL_MAX;
	double z_min = DBL_MAX, z_max = -DBL_MAX;
	double surface_area = 0;
	Bounding_Box(std::vector<TriangleMesh> *meshes)
	{
		for (TriangleMesh mesh : (*meshes))
		{
			for (int i = 0; i < 3; i++)
			{
				this->x_max = std::max(this->x_max, mesh.v[i][0]);
				this->x_min = std::min(this->x_min, mesh.v[i][0]);
				this->y_max = std::max(this->y_max, mesh.v[i][1]);
				this->y_min = std::min(this->y_min, mesh.v[i][1]);
				this->z_max = std::max(this->z_max, mesh.v[i][2]);
				this->z_min = std::min(this->z_min, mesh.v[i][2]);
			}
		}
		this->surface_area = (x_max - x_min) * (y_max - y_min) + (y_max - y_min) * (z_max - z_min) + (z_max - z_min) * (x_max - x_min);
	}
	Bounding_Box(TriangleMesh mesh)
	{
		for (int i = 0; i < 3; i++)
		{
			this->x_max = std::max(this->x_max, mesh.v[i][0]);
			this->x_min = std::min(this->x_min, mesh.v[i][0]);
			this->y_max = std::max(this->y_max, mesh.v[i][1]);
			this->y_min = std::min(this->y_min, mesh.v[i][1]);
			this->z_max = std::max(this->z_max, mesh.v[i][2]);
			this->z_min = std::min(this->z_min, mesh.v[i][2]);
		}
		this->surface_area = (x_max - x_min) * (y_max - y_min) + (y_max - y_min) * (z_max - z_min) + (z_max - z_min) * (x_max - x_min);
	}
	Bounding_Box()
	{
	}

	Bounding_Box Union(Bounding_Box add);
	

	double get_surface_area();

	bool IntersectP(Ray &ray, Eigen::Vector3d invDir, std::array<int, 3>& dirIsNeg);

	void Print();
};

extern "C" _declspec(dllexport) Bounding_Box Union(Bounding_Box ori, Bounding_Box add);
extern "C" _declspec(dllexport) double get_surface_area(Bounding_Box bbox);
extern "C" _declspec(dllexport) bool IntersectP(Bounding_Box &bbox, Ray &ray, Eigen::Vector3d invDir, std::array<int, 3>& dirIsNeg);
#pragma once
#include "Ray.h"
#include "iostream"
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

	Bounding_Box Union(Bounding_Box add)
	{
		Bounding_Box res;
		res.x_max = std::max(this->x_max, add.x_max);
		res.x_min = std::min(this->x_min, add.x_min);
		res.y_max = std::max(this->y_max, add.y_max);
		res.y_min = std::min(this->y_min, add.y_min);
		res.z_max = std::max(this->z_max, add.z_max);
		res.z_min = std::min(this->z_min, add.z_min);
		res.surface_area = (res.x_max - res.x_min) * (res.y_max - res.y_min) + (res.y_max - res.y_min) * (res.z_max - res.z_min) + (res.z_max - res.z_min) * (res.x_max - res.x_min);

		return res;
	}

	double get_surface_area()
	{
		this->surface_area = (x_max - x_min) * (y_max - y_min) + (y_max - y_min) * (z_max - z_min) + (z_max - z_min) * (x_max - x_min);
		return surface_area;
	}

	bool IntersectP(Ray &ray, Eigen::Vector3d invDir, std::array<int, 3>& dirIsNeg)
	{
		double t_min = -DBL_MAX, t_max = DBL_MAX;
		double pmin[3] = { this->x_min, this->y_min, this->z_min };
		double pmax[3] = { this->x_max, this->y_max, this->z_max };

		for (int i = 0; i < 3; i++)
		{
			if (dirIsNeg[i] == 1)
			{
				t_min = std::max(t_min, (pmin[i] - ray.origin[i]) * invDir[i]);
				t_max = std::min(t_max, (pmax[i] - ray.origin[i]) * invDir[i]);
			}
			else
			{
				t_min = std::max(t_min, (pmax[i] - ray.origin[i]) * invDir[i]);
				t_max = std::min(t_max, (pmin[i] - ray.origin[i]) * invDir[i]);
			}
		}
		if (t_min <= t_max && t_max > 0) return true;
		return false;
	}

	void Print()
	{
		printf("bbox\n");
		//std::cout << x_min << " " << x_max << std::endl;
		//std::cout << y_min << " " << y_max << std::endl;
		//std::cout << z_min << " " << z_max << std::endl;

		printf("x: [%lf, %lf]\n", x_min, x_max);
		printf("y: [%lf, %lf]\n", y_min, y_max);
		printf("z: [%lf, %lf]\n", z_min, z_max);	
	}
};

Bounding_Box Union(Bounding_Box ori, Bounding_Box add)
{
	Bounding_Box res;
	res.x_max = std::max(ori.x_max, add.x_max);
	res.x_min = std::min(ori.x_min, add.x_min);
	res.y_max = std::max(ori.y_max, add.y_max);
	res.y_min = std::min(ori.y_min, add.y_min);
	res.z_max = std::max(ori.z_max, add.z_max);
	res.z_min = std::min(ori.z_min, add.z_min);
	res.surface_area = (res.x_max - res.x_min) * (res.y_max - res.y_min) + (res.y_max - res.y_min) * (res.z_max - res.z_min) + (res.z_max - res.z_min) * (res.x_max - res.x_min);
	return res;
}

double get_surface_area(Bounding_Box bbox)
{
	//std::cout << bbox.x_min << " " << bbox.x_max << std::endl;
	//std::cout << bbox.y_min << " " << bbox.y_max << std::endl;
	//std::cout << bbox.z_min << " " << bbox.z_max << std::endl;

	bbox.surface_area = (bbox.x_max - bbox.x_min) * (bbox.y_max - bbox.y_min) + (bbox.y_max - bbox.y_min) * (bbox.z_max - bbox.z_min) + (bbox.z_max - bbox.z_min) * (bbox.x_max - bbox.x_min);
	return bbox.surface_area;
}

bool IntersectP(Bounding_Box &bbox, Ray &ray, Eigen::Vector3d invDir, std::array<int, 3>& dirIsNeg)
{
	double t_min = -DBL_MAX, t_max = DBL_MAX;
	double pmin[3] = { bbox.x_min, bbox.y_min, bbox.z_min };
	double pmax[3] = { bbox.x_max, bbox.y_max, bbox.z_max };

	for (int i = 0; i < 3; i++)
	{
		if (dirIsNeg[i] == 1)
		{
			t_min = std::max(t_min, (pmin[i] - ray.origin[i]) * invDir[i]);
			t_max = std::min(t_max, (pmax[i] - ray.origin[i]) * invDir[i]);
		}
		else
		{
			t_min = std::max(t_min, (pmax[i] - ray.origin[i]) * invDir[i]);
			t_max = std::min(t_max, (pmin[i] - ray.origin[i]) * invDir[i]);
		}
	}
	if (t_min <= t_max && t_max > 0) return true;
	return false;
}
#pragma once
#include <queue>
#include <vector>
#include "TriangleMesh.h"
#include "Bounding_Box.h"
#include "Ray.h"
#include "Intersection.h"
#include <iostream>
#include <opencv2/opencv.hpp>

/*
double partition(double *dim_p, int n)
{
	int l = 0, r = n - 1;
	double pivot = dim_p[(l + r) / 2];

	while (l < r)
	{
		while (l < r && dim_p[r] >= pivot) r--;
		dim_p[l] = dim_p[r];
		while (l < r && dim_p[l] <= pivot) l++;
		dim_p[r] = dim_p[l];
		//std::swap(dim_p[l], dim_p[r]);
		//std::cout << l << " " << r << std::endl;
		//std::cout << dim_p[l] << " " << pivot << " " << dim_p[r] << std::endl;
		//system("pause");
	}
	dim_p[r] = pivot;
	return r;
}
int get_K_p(double *dim_p, int k, int n)
{
	printf("%d %d\n", k, n);
	int pos = partition(dim_p, n);
	// printf("%d\n", pos);
	//system("pause");
	if (pos == k) return pos;
	if (pos > k)
	{
		return get_K_p(dim_p, k, pos);
	}
	else
	{
		return get_K_p(dim_p + pos + 1, k - pos - 1, n - pos - 1) + pos + 1;
	}
	
}
*/

extern "C" _declspec(dllexport) int partition(double *dim_p, int left, int right);
extern "C" _declspec(dllexport) int get_K_p(double *dim_p, int left, int right, int k);
extern "C" _declspec(dllexport) int get_median_p(std::vector<TriangleMesh> *meshes, int p);
extern "C" _declspec(dllexport) void divide_index(std::vector<TriangleMesh> *meshes, int index, int p, std::vector<TriangleMesh> &child_L, std::vector<TriangleMesh> &child_R);
extern "C" _declspec(dllexport) void divide_mid(std::vector<TriangleMesh> *meshes, double mid, int p, std::vector<TriangleMesh> &child_L, std::vector<TriangleMesh> &child_R);

struct BVH
{
	int termination_criteria = 5;
	Bounding_Box bbox;
	BVH* child[2] = { nullptr, nullptr };
	std::vector<TriangleMesh> Object;
	
	/*
	BVH(std::vector<TriangleMesh> *meshes)
	{
		for (TriangleMesh mesh : (*meshes))
		{
			this->bbox.x_max = std::max(this->bbox.x_max, mesh.Barycentric_coordinates[0]);
			this->bbox.x_min = std::min(this->bbox.x_min, mesh.Barycentric_coordinates[0]);
			this->bbox.y_max = std::max(this->bbox.y_max, mesh.Barycentric_coordinates[1]);
			this->bbox.y_min = std::min(this->bbox.y_min, mesh.Barycentric_coordinates[1]);
			this->bbox.z_max = std::max(this->bbox.z_max, mesh.Barycentric_coordinates[2]);
			this->bbox.z_min = std::min(this->bbox.z_min, mesh.Barycentric_coordinates[2]);
		}
	}
	*/

	BVH()
	{
	}

	void Triangle_Barycentric_Render(cv::Mat *res);

	void Print(int &total_mesh, int *vis);
};
extern "C" _declspec(dllexport) void SAH(std::vector<TriangleMesh> *meshes, double &mid, int &p, int bucket_number = 50);
extern "C" _declspec(dllexport) BVH* building(BVH* bvh, std::vector<TriangleMesh> *meshes);
extern "C" _declspec(dllexport) Intersection getIntersection(Ray &ray, TriangleMesh *mesh);
extern "C" _declspec(dllexport) Intersection Intersect(Ray &ray, BVH *node);

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
int partition(double *dim_p, int left, int right)
{
	double pivot = dim_p[(left + right) / 2];
	int l = left, r = right;
	while (l < r)
	{
		while (l < r && dim_p[r] >= pivot) r--;
		dim_p[r] = dim_p[l];
		while (l < r && dim_p[l] <= pivot) l++;
		dim_p[l] = dim_p[r];
	}
	dim_p[r] = pivot;
	return r;
}
int get_K_p(double *dim_p, int left, int right, int k)
{
	int index = partition(dim_p, left, right);
	if (index == k)
		return index;
	if (index > k)
		return get_K_p(dim_p, left, index - 1, k);
	else
		return get_K_p(dim_p, index + 1, right, k);
}
int get_median_p(std::vector<TriangleMesh> *meshes, int p)
{
	int n = meshes->size();
	int k = (n + 1) / 2 - 1;
	double *dim_p = new double[n];
	int i = 0;
	for (auto mesh : (*meshes))
	{
		dim_p[i] = mesh.Barycentric_coordinates[p];
		i++;
	}
	int res = get_K_p(dim_p, 0, n - 1, k);
	delete[] dim_p;
	return res;
}

//std::vector<TriangleMesh> 
void divide(std::vector<TriangleMesh> *meshes, int index, int p, std::vector<TriangleMesh> &child_L, std::vector<TriangleMesh> &child_R)
{
	//std::vector<TriangleMesh> res[2];
	for (int i = 0; i < (*meshes).size(); i++)
	{
		if (i <= index) child_L.push_back((*meshes)[i]);
		else child_R.push_back((*meshes)[i]);
	}
	/*
	for (auto mesh : (*meshes))
	{
		if (mesh.Barycentric_coordinates[p] < mid)
			child_L.push_back(mesh);
		else
			child_R.push_back(mesh);
	}
	*/
	//return child;
}
void divide(std::vector<TriangleMesh> *meshes, double mid, int p, std::vector<TriangleMesh> &child_L, std::vector<TriangleMesh> &child_R)
{

	for (auto mesh : (*meshes))
	{
		if (mesh.Barycentric_coordinates[p] < mid)
			child_L.push_back(mesh);
		else
			child_R.push_back(mesh);
	}
	//return child;
}


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

	void Triangle_Barycentric_Render(cv::Mat *res)
	{
		if (child[0]) child[0]->Triangle_Barycentric_Render(res);
		if (child[1]) child[1]->Triangle_Barycentric_Render(res);
		for (auto mesh : Object)
		{
			double x = mesh.Barycentric_coordinates[0];
			double y = mesh.Barycentric_coordinates[1];
			(*res).cols;
			x = (x + 3) / 6 * res->cols;
			y = (y + 3) / 6 * res->rows;
			res->at<cv::Vec3b>(y, x) = cv::Vec3b(0,0, 255);
			//(*res)[x][y] = Eigen::Vector3d(0, 0, 255);
		}
	}

	void Print(int &total_mesh, int *vis)
	{
		//std::cout << &bbox << std::endl;
		//std::cout << &this->bbox << std::endl;
		//std::cout << bbox.x_min << " " << bbox.x_max << std::endl;
		//std::cout << this->bbox.x_min << " " << this->bbox.x_max << std::endl;
		// bbox.Print();
		//this->bbox.Print();
		//std::cout << child[0] << "\n" << child[1] << std::endl;
		//printf("child_number = %d\n", Object.size());
		if (child[0]) child[0]->Print(total_mesh, vis);
		if (child[1]) child[1]->Print(total_mesh, vis);
		total_mesh += Object.size();
		for (auto mesh : Object)
		{
			vis[mesh.index] += 1;
		}
		return;
	}
};
void SAH(std::vector<TriangleMesh> *meshes, double &mid, int &p, int bucket_number = 50)
{
	mid = -DBL_MAX;
	p = -1;
	double loss = DBL_MAX;
	double axis[3][2] = {DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX};
	for (auto mesh : (*meshes))
	{
		for (int k = 0; k < 3; k++)
		{
			for (int i = 0; i < 3; i++)
			{
				axis[k][0] = std::min(axis[k][0], mesh.v[i][k]);
				axis[k][1] = std::max(axis[k][1], mesh.v[i][k]);
			}
		}
	}
	double res_C = DBL_MAX;
	for (int k = 0; k < 3; k++)
	{
		double L = axis[k][0], R = axis[k][1];
		if (L == R) continue;
		std::vector<Bounding_Box> lbd(bucket_number), rbd(bucket_number);
		std::vector<int> bucket(bucket_number, 0);
		for (auto mesh : (*meshes))
		{
			double val = mesh.Barycentric_coordinates[k];
			//std::cout << val << " " << L << " " << R << std::endl;
			int bucket_index = std::min((int)((val - L) / (R - L) * bucket_number), bucket_number - 1);
			//std::cout << bucket_index << std::endl;
			bucket[bucket_index]++;
			Bounding_Box temp(mesh);
			lbd[bucket_index] = Union(lbd[bucket_index], temp);
		}
		for (int i = 0; i < bucket_number; i++) rbd[i] = lbd[i];
		for (int i = 1; i < bucket_number; i++) lbd[i] = Union(lbd[i], lbd[i - 1]);
		for (int i = bucket_number - 2; i >= 0; i--) rbd[i] = Union(rbd[i], rbd[i + 1]);
		Bounding_Box bd = lbd[bucket_number - 1];

		int L_mesh_number = 0, R_mesh_number = meshes->size(), total_mesh_number = meshes->size();
		double C = DBL_MAX, partition = -1;
		for (int i = 0; i < bucket_number - 1; i++)
		{
			L_mesh_number += bucket[i];
			R_mesh_number -= bucket[i];
			if (L_mesh_number && R_mesh_number)
			{
				//std::cout << get_surface_area(lbd[i]) << std::endl;
				//std::cout << get_surface_area(rbd[i]) << std::endl;
				//std::cout << get_surface_area(bd) << std::endl;
				//system("pause");
				double temp_C = 1.0 * get_surface_area(lbd[i]) / get_surface_area(bd) * L_mesh_number;
				temp_C += 1.0 * get_surface_area(rbd[i]) / get_surface_area(bd) * R_mesh_number;
				if (temp_C < C)
				{
					C = temp_C;
					partition = i;
				}
			}
		}
		if (C < res_C)
		{
			res_C = C;
			p = k;
			mid = (R - L) / bucket_number * (partition + 1) + L;
		}
	}
}
BVH* building(BVH* bvh, std::vector<TriangleMesh> *meshes)
{
	bool _SAH = false;
	//bvh = new BVH(meshes);
	bvh = new BVH();
	Bounding_Box *bbox = new Bounding_Box(meshes);
	bvh->bbox = *bbox;

	if (meshes->size() < bvh->termination_criteria)
	{
		bvh->Object = *meshes;
		return bvh;
	}
	double dif[3] = { bvh->bbox.x_max - bvh->bbox.x_min, bvh->bbox.y_max - bvh->bbox.y_min, bvh->bbox.z_max - bvh->bbox.z_min };
	int p = 0;
	if (dif[1] > dif[p]) p = 1;
	if (dif[2] > dif[p]) p = 2;
	
	//printf("%d %d\n", index, meshes->size());
	//system("pause");


	std::vector<TriangleMesh> child[2];
	if (_SAH)
	{
		double mid = 0;
		SAH(meshes, mid, p);
		divide(meshes, mid, p, child[0], child[1]);
	}
	else
	{
		int index = get_median_p(meshes, p);
		divide(meshes, index, p, child[0], child[1]);
	}
	//printf("%d %d\n", child[0].size(), child[1].size());
	bvh->child[0] = building(nullptr, &child[0]);
	bvh->child[1] = building(nullptr, &child[1]);
	return bvh;
}

inline Intersection getIntersection(Ray &ray, TriangleMesh *mesh)
{
	Intersection res;

	Eigen::Vector3d E1 = mesh->v[1] - mesh->v[0], E2 = mesh->v[2] - mesh->v[0], S = ray.origin - mesh->v[0];
	Eigen::Vector3d D = ray.direction.normalized();
	Eigen::Vector3d S1 = D.cross(E2), S2 = S.cross(E1);
	double tnear = S2.dot(E2) / S1.dot(E1);
	double u = S1.dot(S) / S1.dot(E1);
	double v = S2.dot(D) / S1.dot(E1);
	double EPSILON = 0.0000001;
	if ((u >= 0) && (v >= 0) && (1 - u - v >= 0) && tnear > EPSILON)
	{
		res.happened = true;
		res.coords = (1 - u - v) * mesh->v[0] + u * mesh->v[1] + v * mesh->v[2];
		//res.coords = ray.origin + tnear * ray.direction;
		res.normal = (1 - u - v) * mesh->vn[0].normalized() + u * mesh->vn[1].normalized() + v * mesh->vn[2].normalized();
		//res.normal = ((1 - u - v) * mesh->vn[0] + (u * mesh->vn[1] + v * mesh->vn[2]).normalized() * (u + v)).normalized();
		res.normal = res.normal.normalized();
		Eigen::Vector3d dir = res.coords - ray.origin;
		dir = dir.normalized();
		if (std::abs(dir.dot(ray.direction) - 1) > 0.0000001)
		{
			std::cout << "dir.dot(ray.direction) abnormal" << std::endl;
			std::cout << "tnear : " << tnear << std::endl;
			printf("%.20lf\n", dir.dot(ray.direction));
			std::cout << u << " " << v << std::endl;
			std::cout << "ray origin: " << ray.origin << std::endl;
			std::cout << "mesh vertex 0 : " << mesh->v[0] << std::endl;
			std::cout << "mesh vertex 1 : " << mesh->v[1] << std::endl;
			std::cout << "mesh vertex 2 : " << mesh->v[2] << std::endl;
			std::cout << dir << std::endl << ray.direction << std::endl;
			system("pause");
		}
		res.tex_coords = (1 - u - v) * mesh->vt[0] + u * mesh->vt[1] + v * mesh->vt[2];
		res.distance = (res.coords - ray.origin).norm();
		//std::cout << res.distance << std::endl << tnear * ray.direction.norm() << std::endl << ray.direction << std::endl << ray.direction.norm() << std::endl;
		//system("pause");
		res.mesh = mesh;
		if (mesh->material == nullptr)
			system("pause");
		//std::cout << res.coords << std::endl;
		//std::cout << ray.origin + ray.direction * tnear << std::endl;
		//std::cout << (1 - u - v) * mesh->v[0] + u * mesh->v[1] + v * mesh->v[2] << std::endl;
		//system("pause");

	}
	else res.happened = false;

	return res;
}

inline Intersection Intersect(Ray &ray, BVH *node)
{
	double EPSILON = 0.000001;
	Intersection res;
	//res.distance = DBL_MAX;
	std::array<int, 3> dirIsNeg;
	for (int i = 0; i < 3; i++)
		dirIsNeg[i] = int(ray.direction[i] > 0);
	if (IntersectP(node->bbox, ray, ray.inv_direction, dirIsNeg) == false)
		return res;
	if (node->child[0] == nullptr && node->child[1] == nullptr)
	{
		for (auto mesh_iter = node->Object.begin(); mesh_iter != node->Object.end(); mesh_iter++)
		if (ray.mesh != &(*mesh_iter))
		{
			Intersection temp;
			temp = getIntersection(ray, &(*mesh_iter));
			//if (ray.mesh == temp.mesh && temp.distance < EPSILON)
			//{
			//	std::cout << "same mesh" << std::endl;
			//	system("pause");
			//}
			if (temp.happened == true && ray.mesh != temp.mesh && temp.distance < res.distance)// && temp.distance > EPSILON)
			{
				//if (temp.distance < EPSILON)
				//	system("pause");
				res = temp;
				//std::cout << res.mesh->material->light << std::endl;
			}
			
		}
		return res;
	}
	Intersection hit[2];
	hit[0] = Intersect(ray, node->child[0]);
	hit[1] = Intersect(ray, node->child[1]);
	Intersection _res = hit[0].distance < hit[1].distance ? hit[0] : hit[1];
	
	if (hit[0].distance < hit[1].distance) return hit[0];
	return hit[1];
}
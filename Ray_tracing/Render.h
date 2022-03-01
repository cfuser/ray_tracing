#pragma once
#define _USE_MATH_DEFINES
#include "Eigen/Eigen"
#include "Ray.h"
#include "BVH.h"
#include <cmath>

inline double get_area(Eigen::Vector3d v[3])
{
	double area = 0;
	Eigen::Vector3d e[2] = { v[1] - v[0], v[2] - v[0] };
	//Eigen::Vector3d normal = e[0].cross(e[1]).normalized();
	area = e[0].cross(e[1]).norm() * 0.5;
	return area;
}

inline int Sample_Light(std::vector<TriangleMesh> *Light_Mesh)
{
	double total_area = 0;
	double *area = new double[Light_Mesh->size()];
	int index = 0;
	for (auto light_mesh : (*Light_Mesh))
	{
		area[index++] = get_area(light_mesh.v);
		total_area += area[index - 1];
	}
	double p = get_random_double();//(double)rand() / RAND_MAX;
	//printf("%.20lf\n", p);
	//system("pause");
	total_area *= p;
	int res = -1;
	for (int i = 0; i < index; i++)
	{
		if (total_area <= area[i])
		{
			res = i;
			break;
		}
		total_area -= area[i];
	}
	delete[] area;
	return res;
}

/*
Eigen::Vector3d shade(Intersection p, Eigen::Vector3d wo, int depth, BVH *bvh_root, std::vector<TriangleMesh> *LightMeshes)
{
	Eigen::Vector3d L_dir = Eigen::Vector3d::Zero();
	
	double EPSILON = 0.01;
	{
		// Contribution from the light source.
		std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		int light_mesh_index = Sample_Light(&Light_Mesh);
		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = Light_Mesh[light_mesh_index];
		// Uniformly sample the light at x' (pdf_light = 1 / A);
		// Shoot a ray from p to x';
		
		double pos[3] = { 0, 0, 0 };
		pos[0] = (double)rand() / RAND_MAX;
		pos[1] = (double)rand() / RAND_MAX;
		pos[2] = 1 - pos[0] - pos[1];
		Eigen::Vector3d x_v = Eigen::Vector3d::Zero(), x_vn = Eigen::Vector3d::Zero();
		Eigen::Vector2d x_vt = Eigen::Vector2d::Zero();
		x_v = pos[0] * light_mesh.v[0] + pos[1] * light_mesh.v[1] + pos[2] * light_mesh.v[2];
		x_vn = pos[0] * light_mesh.vn[0] + pos[1] * light_mesh.vn[1] + pos[2] * light_mesh.vn[2];
		x_vt = pos[0] * light_mesh.vt[0] + pos[1] * light_mesh.vt[1] + pos[2] * light_mesh.vt[2];
		Eigen::Vector3d dir = x_v - p.coords;
		double distance = dir.norm();
		dir = dir.normalized();
		Ray ray(p.coords, dir);
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(ray, bvh_root);
		if (abs(block.distance - distance) < EPSILON)
		{
			light_pdf = 1.0 / get_area(light_mesh.v);
			double cos_theta_p = p.normal.dot(dir);
			double cos_theta_x = x_vn.dot(-dir);
			//Eigen::Vector3d f_r = light_mesh.material->light_attr->Radiance;
			//Eigen::Vector3d f_r = light_mesh.material->light_attr->Radiance / M_PI;
			Eigen::Vector3d f_r = p.mesh->material->Kd / M_PI;
			//if (p.coords.dot())
			L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * cos_theta_p * cos_theta_x / std::pow(distance, 2) / light_pdf;
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;
			Eigen::Vector3d n = p.normal;
			Eigen::Vector3d h = wo + dir;
			h = h.normalized();
			//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
		}
	}
	double psi = (double)rand() / RAND_MAX;
	double P_RR = 0.8;

	Eigen::Vector3d L_indir = Eigen::Vector3d::Zero();
	if (psi <= P_RR)
	{
		// Contribution from other reflectors.
		//Test Russian Roulette with probability P_RR;
		
		//Uniformly sample the hemisphere toward wi(pdf_hemi = 1 / 2pi);
		//Trace a ray r(p, wi);
		//if ray r hit a non - emitting object at q
		//	L_indir = shade(q, -wi) * f_r * cos(theta) / pdf_hemi / P_RR;
		
		//double phi = (double)rand() / RAND_MAX * 180, theta = (double)rand() / RAND_MAX * 180;

		double x_1 = (double)rand() / RAND_MAX, x_2 = (double)rand() / RAND_MAX;
		double z = std::fabs(1.0 - x_1);
		double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
		Eigen::Vector3d wi(r * std::cos(phi), r * std::sin(phi), z);
		Ray ray(p.coords, wi);

		double pdf_hemi = 1.0 / 2 / M_PI;
		//BVH* node = nullptr;
		Intersection q = Intersect(ray, bvh_root);
		if (q.happened == true)
		{
			//std::cout << q.distance << std::endl;
			//std::cout << q.mesh->v[0] << " " << q.mesh->v[1] << " " << q.mesh->v[2] << std::endl;
			//system("pause");
			//std::cout << q.mesh->Kd[0] << " " << q.mesh->Kd[1] << " " << q.mesh->Kd[2] << std::endl;
			if (q.happened == true && q.mesh->material->light == false)
			{
				//this is to non-emit material
				//Eigen::Vector3d f_r = q.mesh->material->light_attr->Radiance / M_PI;
				Eigen::Vector3d dir = q.coords - p.coords;
				dir = dir.normalized();
				double cos_theta = p.normal.dot(dir);
				Eigen::Vector3d shading_res = shade(q, -wi, depth + 1, bvh_root, LightMeshes);
				Eigen::Vector3d f_r = p.mesh->material->Kd / M_PI;
				L_indir += f_r.cwiseProduct(shading_res) * cos_theta / pdf_hemi / P_RR;

				Eigen::Vector3d n = p.normal;
				Eigen::Vector3d h = wo + dir;
				h = h.normalized();
				//L_indir += p.mesh->material->Ks * shading_res * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / pdf_hemi / P_RR;

			}
		}
	}
	//return Eigen::Vector3d(255, 0, 0);

	return L_dir + L_indir;
	
}
*/

inline Eigen::Vector3d getColorBilinear(cv::Mat &image, Eigen::Vector2d pos)
{
	//std::cout << pos[0] << " " << pos[1] << std::endl;
	pos[0] = pos[0] - floor(pos[0]);
	pos[1] = pos[1] - floor(pos[1]);
	pos[1] = 1 - pos[1];
	//std::cout << image.cols << " " << image.rows << std::endl;
	double u_img = pos[0] * image.cols;
	double v_img = pos[1] * image.rows;
	double u_min = std::min(double(image.cols - 1), floor(u_img));
	double u_max = std::min(double(image.cols - 1), ceil(u_img));
	double v_min = std::min(double(image.rows - 1), floor(v_img));
	double v_max = std::min(double(image.rows - 1), ceil(v_img));
	//std::cout << v_min << " " << u_min << std::endl;
	auto Q00 = image.at<cv::Vec3b>(v_min, u_min);
	auto Q01 = image.at<cv::Vec3b>(v_min, u_max);
	auto Q10 = image.at<cv::Vec3b>(v_max, u_min);
	auto Q11 = image.at<cv::Vec3b>(v_max, u_max);

	auto lc = (u_img - u_min) / (u_max - u_min), rc = 1 - lc;
	auto tc = (v_img - v_min) / (v_max - v_min), bc = 1 - tc;

	auto cTop = rc * Q00 + lc * Q01;
	auto cBot = rc * Q10 + lc * Q11;
	auto P = bc * cTop + tc * cBot;
	return Eigen::Vector3d(P[2], P[1], P[0]);

}

inline Eigen::Vector3d toWorld(Eigen::Vector3d wi, Eigen::Vector3d normal)
{
	Eigen::Vector3d B, C;
	if (std::abs(normal[0]) > std::abs(normal[1])) {
		double invLen = 1.0 / std::sqrt(normal[0] * normal[0] + normal[2] * normal[2]);
		C = Eigen::Vector3d(normal[2] * invLen, 0.0, -normal[0] *invLen);
	}
	else {
		double invLen = 1.0 / std::sqrt(normal[1] * normal[1] + normal[2] * normal[2]);
		C = Eigen::Vector3d(0.0, normal[2] * invLen, -normal[1] *invLen);
		//double invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
		//C = Eigen::Vector3d(0.0f, N.z * invLen, -N.y *invLen);
	}
	B = C.cross(normal);
	return wi[0] * B + wi[1] * C + wi[2] * normal;
}
Eigen::Vector3d shade(Ray r_in, int depth, BVH *bvh_root, std::vector<TriangleMesh> *LightMeshes)
{
	//if (depth == 1) return Eigen::Vector3d::Zero();
	Eigen::Vector3d L_dir = Eigen::Vector3d::Zero();
	Intersection intersection = Intersect(r_in, bvh_root);
	if (intersection.happened == false)
		return Eigen::Vector3d::Zero();
	if (intersection.mesh->material->light == true)
	{
		//std::cout << intersection.mesh->material->light_attr->Radiance * 0.5 / M_PI << std::endl;
		//system("pause");
		return intersection.mesh->material->light_attr->Radiance;// *0.5 / M_PI;
	}
	//else
	//	return Eigen::Vector3d::Zero();
	
	double EPSILON = 0.000001;
	{
		// Contribution from the light source.
		std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		int light_mesh_index = Sample_Light(&Light_Mesh);
		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = Light_Mesh[light_mesh_index];
		// Uniformly sample the light at x' (pdf_light = 1 / A);
		// Shoot a ray from p to x';

		//double pos[3] = { 0, 0, 0 };
		//pos[0] = (double)rand() / RAND_MAX;
		//pos[1] = (double)rand() / RAND_MAX;
		//pos[2] = (double)rand() / RAND_MAX;
		//pos[1] = pos[1] * (1 - pos[0]);
		//pos[2] = 1 - pos[0] - pos[1];
		//Eigen::Vector3d pos = { (double)rand() / RAND_MAX , (double)rand() / RAND_MAX , (double)rand() / RAND_MAX };
		Eigen::Vector3d pos = { get_random_double(), get_random_double(), get_random_double() };
		pos = pos / pos.sum();
		Eigen::Vector3d x_v = Eigen::Vector3d::Zero(), x_vn = Eigen::Vector3d::Zero();
		Eigen::Vector2d x_vt = Eigen::Vector2d::Zero();
		x_v = pos[0] * light_mesh.v[0] + pos[1] * light_mesh.v[1] + pos[2] * light_mesh.v[2];
		//x_vn = pos[0] * light_mesh.vn[0].normalized() + pos[1] * light_mesh.vn[1].normalized() + pos[2] * light_mesh.vn[2].normalized();
		x_vn = pos[0] * light_mesh.vn[0] + pos[1] * light_mesh.vn[1] + pos[2] * light_mesh.vn[2];
		//x_vn = ((pos[0] * light_mesh.vn[0] + pos[1] * light_mesh.vn[1]).normalized() * (pos[0] + pos[1]) + pos[2] * light_mesh.vn[2]).normalized();
		x_vn = x_vn.normalized();
		x_vt = pos[0] * light_mesh.vt[0] + pos[1] * light_mesh.vt[1] + pos[2] * light_mesh.vt[2];
		Eigen::Vector3d dir = x_v - intersection.coords;
		double distance = dir.norm();
		dir = dir.normalized();
		Ray ray(intersection.coords, dir); ray.mesh = intersection.mesh;
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(ray, bvh_root);
		if (abs(block.distance - distance) < EPSILON)
		{
			
			light_pdf = 1.0 / get_area(light_mesh.v);
			double cos_theta_p = intersection.normal.dot(dir);
			double cos_theta_x = x_vn.dot(-dir);
			if (false)
			if (intersection.mesh->material->name == "Ceiling")
			{
				std::cout << intersection.coords << std::endl;
				std::cout << x_v << std::endl;
				std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
				std::cout << "lighting" << std::endl;
				std::cout << light_mesh.v[0] << std::endl;
				std::cout << light_mesh.v[1] << std::endl;
				std::cout << light_mesh.v[2] << std::endl;

				std::cout << cos_theta_x << std::endl;
				std::cout << cos_theta_p << std::endl;
				system("pause");
			}
			//Eigen::Vector3d f_r = light_mesh.material->light_attr->Radiance;
			//Eigen::Vector3d f_r = light_mesh.material->light_attr->Radiance / M_PI;
			Eigen::Vector3d f_r = intersection.mesh->material->Kd;// *0.5 / M_PI;

			if (intersection.mesh->material->map_Kd_name != "")
			{
				//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
				f_r = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
				f_r = Eigen::Vector3d(std::pow(f_r[0], 2.2), std::pow(f_r[1], 2.2), std::pow(f_r[2], 2.2));
				//f_r = Eigen::Vector3d(0.8, 0.8, 0.8);
				//std::cout << f_r << std::endl;
				//system("pause");
			}
			else
				f_r = intersection.mesh->material->Kd;// *0.5 / M_PI;
			//if (intersection.normal.dot(dir) <= 0)
			if (x_vn.dot(-dir) <= 0)
				f_r = Eigen::Vector3d::Zero();
			f_r = f_r / M_PI;// f_r = f_r / 0.5;
			//if (p.coords.dot())
			//std::cout << light_mesh.material->light_attr->Radiance << std::endl;
			//std::cout << f_r << std::endl;
			//std::cout << cos_theta_p << std::endl;
			//std::cout << cos_theta_x << std::endl;
			//std::cout << distance << std::endl;
			//std::cout << light_pdf << std::endl;
			//system("pause");
			L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * cos_theta_p * cos_theta_x / std::pow(distance, 1) / light_pdf;
			//if (L_dir[0] * 20 * 255 > 200 || L_dir[1] * 20 * 255 > 200 || L_dir[2] * 20 * 255 > 200)
			//{
			//	std::cout << L_dir << std::endl;
			//	system("pause");
			//	return Eigen::Vector3d(-1.0, -1.0, -1.0);
			//}
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;
			Eigen::Vector3d n = intersection.normal;
			Eigen::Vector3d h = -r_in.direction + dir;
			h = h.normalized();
			//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
		}
	}
	double psi = get_random_double();//(double)rand() / RAND_MAX;
	double P_RR = 0.8;

	Eigen::Vector3d L_indir = Eigen::Vector3d::Zero();
	//if (false)
	if (psi <= P_RR)
	{
		// Contribution from other reflectors.
		//Test Russian Roulette with probability P_RR;

		//Uniformly sample the hemisphere toward wi(pdf_hemi = 1 / 2pi);
		//Trace a ray r(p, wi);
		//if ray r hit a non - emitting object at q
		//	L_indir = shade(q, -wi) * f_r * cos(theta) / pdf_hemi / P_RR;

		//double phi = (double)rand() / RAND_MAX * 180, theta = (double)rand() / RAND_MAX * 180;

		//double x_1 = (double)rand() / RAND_MAX, x_2 = (double)rand() / RAND_MAX;
		double x_1 = get_random_double(), x_2 = get_random_double();
		double theta = x_1 * M_PI / 2;
		//double z = 1.0 - 2 * x_1; //std::abs(1.0 - x_1);
		double z = std::cos(theta);
		double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
		//r = std::sqrt(1 - z * z);
		Eigen::Vector3d wi(r * std::cos(phi), r * std::sin(phi), z);
		wi = toWorld(wi, intersection.normal);
		Ray ray(intersection.coords, wi); ray.mesh = intersection.mesh;
		if (wi.dot(intersection.normal) < 0)
		{
			std::cout << " wi.dot(intersection.normal) < 0" << std::endl;
			system("pause");
		}
		double pdf_hemi = 1.0 * 0.5 / M_PI;
		//BVH* node = nullptr;
		Intersection q = Intersect(ray, bvh_root);
		if (q.happened == true)
		{
			//std::cout << q.distance << std::endl;
			//std::cout << q.mesh->v[0] << " " << q.mesh->v[1] << " " << q.mesh->v[2] << std::endl;
			//system("pause");
			//std::cout << q.mesh->Kd[0] << " " << q.mesh->Kd[1] << " " << q.mesh->Kd[2] << std::endl;
			if (q.happened == true && q.mesh->material->light == false)
			{
				//this is to non-emit material
				//Eigen::Vector3d f_r = q.mesh->material->light_attr->Radiance / M_PI;
				Eigen::Vector3d dir = q.coords - intersection.coords;
				dir = dir.normalized();
				if (intersection.normal.dot(dir) < 0)
				{
					//f_r = Eigen::Vector3d::Zero();
					std::cout << "intersection.normal.dot(dir) < 0" << std::endl;
					std::cout << ray.direction << std::endl << dir << std::endl;
					system("pause");
				}
				double cos_theta = intersection.normal.dot(dir);
				Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh;
				Eigen::Vector3d shading_res = shade(r_out, depth + 1, bvh_root, LightMeshes);

				Eigen::Vector3d f_r = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
				//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
				if (intersection.mesh->material->map_Kd_name != "")
				{
					//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
					f_r = getColorBilinear(intersection.mesh->material->map_Kd, q.tex_coords) / 255.0;
					f_r = Eigen::Vector3d(std::pow(f_r[0], 2.2), std::pow(f_r[1], 2.2), std::pow(f_r[2], 2.2));
					//std::cout << f_r << std::endl;
					//system("pause");
				}
				else
					f_r = intersection.mesh->material->Kd;// *0.5 / M_PI;
				
				if (intersection.normal.dot(r_out.direction) < 0)
				{
					f_r = Eigen::Vector3d::Zero();
					std::cout << "intersection.normal.dot(r_out.direction) < 0" << std::endl;
					std::cout << ray.direction << std::endl << r_out.direction << std::endl;
					system("pause");
				}
				f_r = f_r / M_PI;// f_r = f_r / 0.5;
				L_indir += f_r.cwiseProduct(shading_res) * cos_theta / pdf_hemi / P_RR;

				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d h = -r_in.direction + dir;
				h = h.normalized();
				//L_indir += p.mesh->material->Ks * shading_res * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / pdf_hemi / P_RR;

			}
		}
	}
	//return Eigen::Vector3d(255, 0, 0);

	return L_dir + L_indir;

}


Eigen::Vector3d ray_generation(Eigen::Vector3d camPos, Eigen::Vector3d dir, BVH* bvh_root, std::vector<TriangleMesh> *LightMeshes)
{
	//int N = 5;
	//std::vector<Eigen::Vector3d> pos(N, Eigen::Vector3d::Zero());
	//Uniformly choose N sample positions within the pixel;

	Eigen::Vector3d pixel_radiance = Eigen::Vector3d::Zero();
	//shoot a ray r(camPos, cam_to_sample);
	Ray ray(camPos, dir); ray.mesh = nullptr;
	//BVH *node = nullptr;
	
	//Intersection p = Intersect(ray, bvh_root);
	//if (p.happened == false) return Eigen::Vector3d::Zero();
	//pixel_radiance += shade(p, -dir, 0, bvh_root, LightMeshes);
	Eigen::Vector3d res = shade(ray, 0, bvh_root, LightMeshes);
	if (res[0] == -1.0) return Eigen::Vector3d(-1, -1, -1);
	pixel_radiance += res;
	//return Eigen::Vector3d(255, 0, 0);
	return pixel_radiance;
}

/*
Eigen::Vector3d shade(Intersection p, Eigen::Vector3d wo)
{
	//Manually specify a probability P_RR;
	double P_RR = 0.5;
	//Randomly select ksi in a uniform dist.in[0, 1];
	double ksi = (double)rand() / RAND_MAX;
	if (ksi > P_RR) return Eigen::Vector3d::Zero();

	//Randomly choose one direction wi ~pdf(w);
	//Trace a ray(p, wi);
	Ray r(p.coords, wi);
	if ray r hit the light
		return L_i * f_r * cosine / pdf(wi) / P_RR;
	else
		return shade(q, -wi) * f_r * cosine / pdf(wi) / P_RR;

}
*/


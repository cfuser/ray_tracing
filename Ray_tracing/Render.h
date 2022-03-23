#pragma once
#define _USE_MATH_DEFINES
#include "Eigen/Eigen"
#include "Ray.h"
#include "BVH.h"
#include <math.h>
#include "global.h"


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

inline int Sample_Light_Uniform(std::vector<double> *lightmesh_area, int left, int right)
{
	int index = get_random_double() * (right - left) + left;
	return index;
}
inline int Sample_Light_Area(std::vector<double> *lightmesh_area)
{
	double p = get_random_double();
	double area = p * lightmesh_area->back();
	//std::cout << lightmesh_area->back() << std::endl;
	int l = 0, r = lightmesh_area->size() - 1;
	if (area >= lightmesh_area->back())
	{
		std::cout << "area >= lightmesh_area.back()" << std::endl;
		system("pause");
		return r;
	}
	while (l < r)
	{
		int mid = (l + r) / 2;
		if ((*lightmesh_area)[mid] == p)
			break;
		else if ((*lightmesh_area)[mid] < p)
			l = mid + 1;
		else if ((*lightmesh_area)[mid] > p)
			r = mid;
	}
	return l;
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
	Eigen::Vector3d res = wi[0] * B + wi[1] * C + wi[2] * normal;
	return res.normalized();
}

inline void power_correction(double &weight_Sample_Light, double &weight_BRDF)
{
	double a = std::pow(weight_Sample_Light, 2), b = std::pow(weight_BRDF, 2), c = a + b;
	if (a == 0 && b == 0)
		return;
	weight_Sample_Light = a / c;
	weight_BRDF = b / c;
	return;
}

inline Eigen::Vector3d get_Radiance_0(Eigen::Vector3d Radiance, Eigen::Vector3d f_r, double cos_theta_x, double cos_theta_p, double distance, double distance_coefficient, double light_pdf)
{
	return Radiance.cwiseProduct(f_r) * cos_theta_x * cos_theta_p / std::pow(distance, distance_coefficient) / light_pdf;
}
inline Eigen::Vector3d get_Radiance_1(Eigen::Vector3d Radiance, Eigen::Vector3d f_r, double cos_alpha, double Ns, double pdf_hemi, double cos_theta)
{
	if (specular_coefficient == 0)
		return Radiance.cwiseProduct(f_r) * fast_power(std::max(0.0, cos_alpha), Ns) / pdf_hemi * cos_theta;
	else if (specular_coefficient == 1)
		return Radiance.cwiseProduct(f_r) * fast_power(std::max(0.0, cos_alpha), Ns) / pdf_hemi * (Ns + 2) / 2.0 / M_PI * cos_theta;
	else
	{
		std::cout << "unknown specular coefficient" << std::endl;
		system("pause");
		return Eigen::Vector3d::Zero();
	}
}
inline Eigen::Vector3d get_Radiance_2(Eigen::Vector3d Radiance, Eigen::Vector3d f_r, double cos_theta, double pdf_hemi)
{
	return Radiance.cwiseProduct(f_r) * cos_theta / pdf_hemi;
}

inline void f_r_correction(Eigen::Vector3d &f_r_Kd, Eigen::Vector3d &f_r_Ks)
{
	Eigen::Vector3d total = f_r_Kd + f_r_Ks;
	double maxm = -DBL_MAX;
	for (int _ = 0; _ < 3; _++)
	{
		total[_] = std::max(total[_], 1.0);
		maxm = std::max(total[_], maxm);
	}
	//for (int _ = 0; _ < 3; _++)
	//	total[_] = maxm;
	f_r_Kd = Eigen::Vector3d(f_r_Kd[0] / total[0], f_r_Kd[1] / total[1], f_r_Kd[2] / total[2]);
	f_r_Ks = Eigen::Vector3d(f_r_Ks[0] / total[0], f_r_Ks[1] / total[1], f_r_Ks[2] / total[2]);
	return;
}

Eigen::Vector3d shade_4(Ray r_out, int depth, BVH *bvh_root, std::vector<TriangleMesh> *LightMeshes, Scene *scene)
{
	if (depth == shading_depth) return Eigen::Vector3d::Zero();
	double psi = get_random_double();//(double)rand() / RAND_MAX;

	//std::vector<double> *lightmesh_area = &scene->lightmesh_area;
	double total_lightmesh_area = scene->total_lightmesh_area;
	
	// L_dir is sample Light, L_indir is indirectional light

	Eigen::Vector3d L_dir = Eigen::Vector3d::Zero();
	Eigen::Vector3d L_indir = Eigen::Vector3d::Zero();

	Intersection intersection = Intersect(r_out, bvh_root);
	if (intersection.happened == false)
		return Eigen::Vector3d::Zero();

	double cos_theta_o = intersection.normal.dot(-r_out.direction);

	if (intersection.mesh->material->light == true)
	{
		if (cos_theta_o > 0)
			return intersection.mesh->material->light_attr->Radiance;
		else
			return Eigen::Vector3d::Zero();
	}


	Eigen::Vector3d f_r_Kd = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
	Eigen::Vector3d f_r_Ks = Eigen::Vector3d::Zero();
	//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
	if (intersection.mesh->material->map_Kd_name != "")
	{
		//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
		f_r_Kd = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
		f_r_Kd = Eigen::Vector3d(std::pow(f_r_Kd[0], 2.2), std::pow(f_r_Kd[1], 2.2), std::pow(f_r_Kd[2], 2.2));
		//std::cout << f_r << std::endl;
		//system("pause");
	}
	else
		f_r_Kd = intersection.mesh->material->Kd;// *0.5 / M_PI;

	if (intersection.mesh->material->map_Ks_name != "")
	{
		f_r_Ks = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
		f_r_Ks = Eigen::Vector3d(std::pow(f_r_Ks[0], 2.2), std::pow(f_r_Ks[1], 2.2), std::pow(f_r_Ks[2], 2.2));
	}
	else
		f_r_Ks = intersection.mesh->material->Ks;// *0.5 / M_PI;
	
	// !!!important choice

	// normalize Kd + Ks when \exists Kd + Ks > 1
	
	//if (f_r_Kd[0] + f_r_Ks[0] > 1 || f_r_Kd[1] + f_r_Ks[1] > 1 || f_r_Kd[2] + f_r_Ks[2] > 1)
	f_r_correction(f_r_Kd, f_r_Ks);

	//if (f_r_Kd[color_type] + f_r_Ks[color_type] > 1)
	//	f_r_correction(f_r_Kd, f_r_Ks);

	//double rho_d = f_r_Kd.sum();
	//double rho_s = f_r_Ks.sum();
	//double xi = get_random_double() * (rho_d + rho_s);
	//f_r_Kd = f_r_Kd / M_PI;// f_r = f_r / 0.5;
	
	double rho_d = 0, rho_s = 0;
	rho_d = f_r_Kd[color_type];
	if (f_r_Ks[color_type] != 0)
		rho_s = f_r_Ks[color_type] * (intersection.mesh->material->Ns + 2) / (intersection.mesh->material->Ns + 1) * cos_theta_o;
	
	Eigen::Vector3d rho_d_vec = f_r_Kd;
	Eigen::Vector3d rho_s_vec = f_r_Ks * (intersection.mesh->material->Ns + 2) / (intersection.mesh->material->Ns + 1) * cos_theta_o;

	// !!!important choice

	// min(rho_s, 1) to make rho_s small
	if (rho_s > 1)
	{
		rho_s = 1;
		//std::cout << "rho_s > 1" << std::endl;
		//system("pause");
	}

	for (int _ = 0; _ < 3; _++)
	{
		if (rho_s_vec[_] > 1)
			rho_s_vec[_] = 1;
		//rho_s_vec[_] = std::max(rho_s_vec[_], 1 - rho_d_vec[_]);
		rho_s = std::max(rho_s, rho_s_vec[_]);
	}

	if (control_rho_s == 0)
		f_r_correction(rho_d_vec, rho_s_vec);

	Eigen::Vector3d rho_s_vec_contribution = rho_s_vec;
	for (int _ = 0; _ < 3; _++)
		rho_s_vec_contribution[_] = std::min(rho_s_vec_contribution[_], 1 - rho_d_vec[_]);

	double weight_Sample_Light = 0;
	double weight_BRDF = 0;
	bool weight_Sample_Light_cri = false;
	bool weight_BRDF_cri = false;
	Eigen::Vector3d Sample_Light = Eigen::Vector3d::Zero();
	Eigen::Vector3d BRDF = Eigen::Vector3d::Zero();

	double EPSILON = 0.000001;
	int sample_each_method = 1;// scene->LightIndex.size();
	if (sample_all_light == 1)
		sample_each_method = LightMeshes->size();
	if (sample_by_group == 1)
		sample_each_method = scene->LightIndex.size();
	if (sample_dir_light == 1)
	for (int i = 0; i < sample_each_method; i++)
	{
		// Contribution from the light source.
		//std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		//int light_mesh_index = Sample_Light(LightMeshes);
		int light_mesh_index;
		//light_mesh_index = Sample_Light_Area(lightmesh_area);
		if (sample_all_light == 1)
			light_mesh_index = i;
		else if (sample_by_group == 1)
			light_mesh_index = Sample_Light_Uniform(&scene->lightmesh_area, scene->LightIndex[i].first, scene->LightIndex[i].second);
		else
			light_mesh_index = Sample_Light_Uniform(&scene->lightmesh_area, 0, scene->lightmesh_area.size());

		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = (*LightMeshes)[light_mesh_index];
		// Uniformly sample the light at x' (pdf_light = 1 / A);
		// Shoot a ray from p to x';

		//double pos[3] = { 0, 0, 0 };
		//pos[0] = (double)rand() / RAND_MAX;
		//pos[1] = (double)rand() / RAND_MAX;
		//pos[2] = (double)rand() / RAND_MAX;
		//pos[1] = pos[1] * (1 - pos[0]);
		//pos[2] = 1 - pos[0] - pos[1];
		//Eigen::Vector3d pos = { (double)rand() / RAND_MAX , (double)rand() / RAND_MAX , (double)rand() / RAND_MAX };
		Eigen::Vector3d pos;// = { get_random_double(), get_random_double(), get_random_double() };
		{
			double xi_1 = get_random_double(), xi_2 = get_random_double();
			xi_1 = std::sqrt(xi_1);
			pos = Eigen::Vector3d((1 - xi_1), xi_1 * (1 - xi_2), xi_1 * xi_2);
		}
		
		//pos = pos / pos.sum();
		
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
		Ray r_in(intersection.coords, dir); r_in.mesh = intersection.mesh; r_in.Ni = intersection.mesh->material->Ni; r_in.in_material = r_out.in_material;
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(r_in, bvh_root);
		if (false)
		if (block.distance < distance && block.mesh->material->light == true)
		{
			x_v = block.coords;
			x_vn = block.normal;
			x_vt = block.tex_coords;
			distance = block.distance;
			light_mesh = *block.mesh;
			//std::cout << "new light" << std::endl;
			//system("pause");
		}
		if (abs(block.distance - distance) < EPSILON)
		{
			if (sample_all_light == 1)
				light_pdf = 1.0 / get_area(light_mesh.v);
			else if (sample_by_group == 1)
			{
				double left, right;
				if (scene->LightIndex[i].first == 0)
					left = 0;
				else
					left = scene->lightmesh_area[scene->LightIndex[i].first - 1];

				if (scene->LightIndex[i].second == 0)
					right = 0;
				else
					right = scene->lightmesh_area[scene->LightIndex[i].second - 1];

				if (right - left == 0)
				{
					std::cout << "right - left == 0" << std::endl;
					std::cout << "left : " << left << ", right : " << right << std::endl;
					system("pause");
				}
				if (group_light_pdf_choice == 0)
					light_pdf = 1 / (right - left);
				else if (group_light_pdf_choice == 1)
					light_pdf = 1.0 / get_area(light_mesh.v) / (double)(scene->LightIndex[i].second - scene->LightIndex[i].first);
			}
			else
			{
				if (one_light_pdf_choice == 0)
					light_pdf = 1.0 / scene->lightmesh_area.back();
				else if (one_light_pdf_choice == 1)
					light_pdf = 1.0 / get_area(light_mesh.v) / (double)scene->lightmesh_area.size();
				weight_Sample_Light = light_pdf;
				weight_Sample_Light_cri = true;
			}
			double cos_theta_p = intersection.normal.dot(dir);
			double cos_theta_x = x_vn.dot(-dir);
			weight_Sample_Light = get_area(light_mesh.v) * cos_theta_x / std::pow(distance, 2) * scene->lightmesh_area.size();
			weight_Sample_Light_cri = true;
			//weight_Sample_Light = weight_Sample_Light / get_area(light_mesh.v) * lightmesh_area->back();

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
			
			if (x_vn.dot(-dir) <= 0)
			{
				f_r_Kd = Eigen::Vector3d::Zero();
				f_r_Ks = Eigen::Vector3d::Zero();
			}
			//if (p.coords.dot())
			//std::cout << light_mesh.material->light_attr->Radiance << std::endl;
			//std::cout << f_r << std::endl;
			//std::cout << cos_theta_p << std::endl;
			//std::cout << cos_theta_x << std::endl;
			//std::cout << distance << std::endl;
			//std::cout << light_pdf << std::endl;
			//system("pause");

			// f_r_Kd = intersection.mesh->material->Kd;
			Eigen::Vector3d temp_ = light_mesh.material->light_attr->Radiance.cwiseProduct(f_r_Kd) * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient) / light_pdf / M_PI;
			//std::cout << i << std::endl << temp_ << std::endl;
			Sample_Light += temp_;
			//Sample_Light += get_Radiance_0(light_mesh.material->light_attr->Radiance, f_r_Kd, cos_theta_x, cos_theta_p, distance, distance_coefficient, light_pdf);

			//if (L_dir[0] * 20 * 255 > 200 || L_dir[1] * 20 * 255 > 200 || L_dir[2] * 20 * 255 > 200)
			//{
			//	std::cout << L_dir << std::endl;
			//	system("pause");
			//	return Eigen::Vector3d(-1.0, -1.0, -1.0);
			//}
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;

			if (intersection.mesh->material->Ns != 1)
			{
				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
				r = r.normalized();
				double cos_alpha = r.dot(r_in.direction);

				// cos_alpha = 1;

				//L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * 
				//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
				Eigen::Vector3d res_;// = Eigen::Vector3d::Ones();
				res_ = f_r_Ks * (intersection.mesh->material->Ns + 2) / (intersection.mesh->material->Ns + 1) * cos_theta_p;

				//if (specular_large_coefficient == 1)
				//	res_ = res_ * (intersection.mesh->material->Ns + 1) / 2.0 / M_PI;
				//res_ = res_ * fast_power(cos_alpha, intersection.mesh->material->Ns) * (intersection.mesh->material->Ns + 2) / (intersection.mesh->material->Ns + 1) * cos_theta_p;
				//res_ = res_.cwiseProduct(f_r_Ks) / P_RR;

				if (specular_large_coefficient == 1)
					res_ = res_ * (intersection.mesh->material->Ns + 1) / 2.0 / M_PI * fast_power(cos_alpha, intersection.mesh->material->Ns);
				res_ = res_.cwiseProduct(light_mesh.material->light_attr->Radiance) * cos_theta_x / std::pow(distance, distance_coefficient) / light_pdf;

				Eigen::Vector3d rho_s_vec_inv = rho_s_vec.cwiseInverse();
				//shading_res = shading_res / rho_s_vec[color_type];

				// to calculate '/ rho_s'
				//res_ = res_.cwiseProduct(rho_s_vec_inv);

				//shading_res = shading_res.cwiseProduct(rho_s_vec);
				//res_ = res_.cwiseProduct(rho_s_vec_contribution);

				//res_ = light_mesh.material->light_attr->Radiance.cwiseProduct(rho_s_vec_contribution) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / light_pdf * cos_theta_p;
				//res_ = get_Radiance_1(light_mesh.material->light_attr->Radiance, f_r_Ks, cos_alpha, intersection.mesh->material->Ns, light_pdf, cos_theta_p);
				//f_r_Ks = intersection.mesh->material->Ks;
				res_ = f_r_Ks.cwiseProduct(light_mesh.material->light_attr->Radiance) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / light_pdf * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient);
				if (control_rho_s)
					res_ = res_.cwiseProduct(rho_s_vec_inv).cwiseProduct(rho_s_vec_contribution);
				Sample_Light += res_;
			}
		}
	}
	
	if (false)
	if (sample_dir_light == 1 && sample_all_light == 0 && sample_by_group == 0 && intersection.mesh->material->Ns != 1)
	{
		Eigen::Vector3d n = intersection.normal;
		Eigen::Vector3d r_in_dir = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
		r_in_dir = r_in_dir.normalized();

		double xi_1 = get_random_double(), xi_2 = get_random_double(), temp_ = 1.0 / (intersection.mesh->material->Ns + 1);
		double cos_alpha = std::pow(xi_1, temp_), phi = 2 * M_PI * xi_2;
		double sin_alpha = std::pow(1 - std::pow(cos_alpha, 2), 0.5);
		double cos_phi = std::cos(phi), sin_phi = std::sin(phi);
		Eigen::Vector3d wi;
		wi = Eigen::Vector3d(sin_alpha * cos_phi, sin_alpha * sin_phi, cos_alpha);
		wi = toWorld(wi, r_in_dir);
		wi = wi.normalized();
		Ray r_in(intersection.coords, wi); r_in.mesh = intersection.mesh; r_in.Ni = intersection.mesh->material->Ni; r_in.in_material = r_out.in_material;
		// wi in the back of plane
		if (wi.dot(intersection.normal) <= 0)
		{
			f_r_Kd = Eigen::Vector3d::Zero();
			f_r_Ks = Eigen::Vector3d::Zero();
		}
		if (wi.dot(intersection.normal) > 0)
		{
			//Eigen::Vector3d shading_res = shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene);
			double cos_alpha = r_in_dir.dot(wi);
			double cos_theta_i = intersection.normal.dot(wi);
			double cos_theta_o = intersection.normal.dot(-r_out.direction);
			double pdf = (intersection.mesh->material->Ns + 1) / 2.0 / M_PI * fast_power(cos_alpha, intersection.mesh->material->Ns);

			Intersection q = Intersect(r_in, bvh_root);
			Eigen::Vector3d shading_res = Eigen::Vector3d::Zero();

			if (q.happened == true && q.mesh->material->light == true)
				shading_res = q.mesh->material->light_attr->Radiance;
			else
				shading_res = Eigen::Vector3d::Zero();

			if (specular_large_coefficient == 1)
				shading_res = shading_res * (intersection.mesh->material->Ns + 1) / 2.0 / M_PI * fast_power(cos_alpha, intersection.mesh->material->Ns);

			shading_res = shading_res * (intersection.mesh->material->Ns + 2) / (intersection.mesh->material->Ns + 1) * cos_theta_i;
			shading_res = shading_res.cwiseProduct(f_r_Ks) / pdf / P_RR;
			Eigen::Vector3d rho_s_vec_inv = rho_s_vec.cwiseInverse();
			//shading_res = shading_res / rho_s_vec[color_type];

			// to calculate '/ rho_s'
			if (control_rho_s)
			shading_res = shading_res.cwiseProduct(rho_s_vec_inv);

			//shading_res = shading_res.cwiseProduct(rho_s_vec);
			if (control_rho_s)
			shading_res = shading_res.cwiseProduct(rho_s_vec_contribution);

			//shading_res = shading_res * std::min(rho_s, 1 - rho_d);
			//res = res / P_RR;
			BRDF += shading_res;
			//return res;
		}
	}
	//return Sample_Light;

	// hit the back of triangle and don't refraction
	if (r_out.direction.dot(intersection.normal) > 0 && r_out.Ni == intersection.mesh->material->Ni && r_out.in_material == false)
	{
		//std::cout << "r_in.direction.dot(intersection.normal) > 0" << std::endl;
		//std::cout << r_in.direction << std::endl << intersection.normal << std::endl;
		//
		//system("pause");
		return Eigen::Vector3d::Zero();

	}

	double random_xi = get_random_double();

	//double psi = get_random_double();
	//if (psi >= P_RR)
	//	return Eigen::Vector3d::Zero();

	Eigen::Vector3d res = Eigen::Vector3d::Zero();
	
	//if (psi <= P_RR)
	{
		// non-refraction
		if (intersection.mesh->material->Ni == r_out.Ni && r_out.in_material == false)
		{
			if (psi <= P_RR)
			{
				//if (random_xi < rho_d)
				{
					// diffuse

					Eigen::Vector3d wi;
					double pdf;
					double cos_theta_i;


					//cosine distribution
					if (kd_distribution == 0)
					{
						// xi_1 is 'cos^2(xi_1)', xi_2 is 'xi_2'
						double xi_1 = get_random_double(), xi_2 = get_random_double();
						//double theta = xi_2 * M_PI / 2;
						//double z = 1.0 - 2 * x_1; //std::abs(1.0 - x_1);
						double cos_theta = std::sqrt(xi_1), sin_theta = std::sqrt(1.0 - xi_1);
						double cos_phi = std::cos(2 * M_PI * xi_2), sin_phi = std::sin(2 * M_PI * xi_2);
						//double r = std::sqrt(1.0 - xi_1 * xi_1), phi = 2 * M_PI * xi_2;
						wi = Eigen::Vector3d(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
						wi = toWorld(wi, intersection.normal);
						wi = wi.normalized();
						cos_theta_i = intersection.normal.dot(wi);
						pdf = cos_theta_i / M_PI;

					}

					// uniform distribution
					if (kd_distribution == 1)
					{
						double x_1 = get_random_double(), x_2 = get_random_double();
						double theta = x_1 * M_PI / 2;
						//double z = 1.0 - 2 * x_1; //std::abs(1.0 - x_1);
						double z = get_random_double();
						double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
						wi = Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z);
						wi = toWorld(wi, intersection.normal);
						wi = wi.normalized();
						cos_theta_i = intersection.normal.dot(wi);
						pdf = 0.5 / M_PI;
					}
					Ray r_in(intersection.coords, wi); r_in.mesh = intersection.mesh; r_in.Ni = intersection.mesh->material->Ni; r_in.in_material = r_out.in_material;
					Intersection q = Intersect(r_in, bvh_root);
					Eigen::Vector3d shading_res = Eigen::Vector3d::Zero();

					//if (false)
					if (sample_dir_light == 0 || (q.happened == true && q.mesh->material->light != true))
						shading_res = shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene);
					else
						shading_res = Eigen::Vector3d::Zero();
					shading_res = shading_res.cwiseProduct(rho_d_vec) * cos_theta_i / M_PI / pdf / P_RR;
					//shading_res = shading_res * rho_d;
					res += shading_res;
					//res = res / P_RR;
					//return res;
				}

				//if (false)
				//if (rho_s != 0)
				//if (rho_d <= random_xi && random_xi < rho_d + rho_s)
				if (specular_exists == 1)
					if (f_r_Ks[0] != 0 || f_r_Ks[1] != 0 || f_r_Ks[2] != 0)
					{
						// specular
						Eigen::Vector3d n = intersection.normal;
						Eigen::Vector3d r_in_dir = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
						r_in_dir = r_in_dir.normalized();

						double xi_1 = get_random_double(), xi_2 = get_random_double(), temp_ = 1.0 / (intersection.mesh->material->Ns + 1);
						double cos_alpha = std::pow(xi_1, temp_), phi = 2 * M_PI * xi_2;
						double sin_alpha = std::pow(1 - std::pow(cos_alpha, 2), 0.5);
						double cos_phi = std::cos(phi), sin_phi = std::sin(phi);
						Eigen::Vector3d wi;
						wi = Eigen::Vector3d(sin_alpha * cos_phi, sin_alpha * sin_phi, cos_alpha);
						wi = toWorld(wi, r_in_dir);
						wi = wi.normalized();
						Ray r_in(intersection.coords, wi); r_in.mesh = intersection.mesh; r_in.Ni = intersection.mesh->material->Ni; r_in.in_material = r_out.in_material;
						// wi in the back of plane
						if (wi.dot(intersection.normal) <= 0)
						{
							f_r_Kd = Eigen::Vector3d::Zero();
							f_r_Ks = Eigen::Vector3d::Zero();
						}
						if (wi.dot(intersection.normal) > 0)
						{
							//Eigen::Vector3d shading_res = shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene);
							double cos_alpha = r_in_dir.dot(wi);
							double cos_theta_i = intersection.normal.dot(wi);
							double cos_theta_o = intersection.normal.dot(-r_out.direction);
							double pdf = (intersection.mesh->material->Ns + 1) / 2.0 / M_PI * fast_power(cos_alpha, intersection.mesh->material->Ns);

							Intersection q = Intersect(r_in, bvh_root);
							Eigen::Vector3d shading_res = Eigen::Vector3d::Zero();

							if (sample_dir_light == 0 || (q.happened == true && q.mesh->material->light != true))
								shading_res = shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene);
							else
								shading_res = Eigen::Vector3d::Zero();

							if (specular_large_coefficient == 1)
								shading_res = shading_res * (intersection.mesh->material->Ns + 1) / 2.0 / M_PI * fast_power(cos_alpha, intersection.mesh->material->Ns);

							shading_res = shading_res * (intersection.mesh->material->Ns + 2) / (intersection.mesh->material->Ns + 1) * cos_theta_i;
							shading_res = shading_res.cwiseProduct(f_r_Ks) / pdf / P_RR;
							Eigen::Vector3d rho_s_vec_inv = rho_s_vec.cwiseInverse();
							//shading_res = shading_res / rho_s_vec[color_type];

							// to calculate '/ rho_s'
							if (control_rho_s)
							shading_res = shading_res.cwiseProduct(rho_s_vec_inv);

							//shading_res = shading_res.cwiseProduct(rho_s_vec);
							if (control_rho_s)
							shading_res = shading_res.cwiseProduct(rho_s_vec_contribution);

							//shading_res = shading_res * std::min(rho_s, 1 - rho_d);
							//res = res / P_RR;
							res += shading_res;
							//return res;
						}
					}

				if (false)
					if (rho_d + rho_s <= random_xi)
					{
						// zero distribution
						//return Eigen::Vector3d::Zero();
						res += Eigen::Vector3d::Zero();
					}
			}
		}
		else if (refraction_or_not && psi <= 0.8)
		{
			double n_i = r_out.Ni;
			double n_t = intersection.mesh->material->Ni;
			if (r_out.in_material == true)
				n_t = 1;
			Eigen::Vector3d normal = intersection.normal;
			if (r_out.direction.dot(normal) > 0)
			{
				normal = -normal;
				//std::cout << "r_in.direction.dot(normal) > 0" << std::endl;
				//system("pause");
			}
			double cos_theta_i = normal.dot(-r_out.direction);
			double sin_theta_i = std::sqrt(1 - n_i * n_i);
			double sin_theta_t = sin_theta_i * n_i / n_t;
			double R_rate = 1;
			double path_rate = get_random_double();
			Eigen::Vector3d res = Eigen::Vector3d::Zero();
			if (std::abs(sin_theta_t) < 1)
			{
				double cos_theta_t = std::sqrt(1 - sin_theta_t * sin_theta_t);
				double n_i_cos_theta_i = n_i * cos_theta_i;
				double n_t_cos_theta_t = n_t * cos_theta_t;
				double R_s = std::pow((n_i_cos_theta_i - n_t_cos_theta_t) / (n_i_cos_theta_i + n_t_cos_theta_t), 2.0);
				double n_i_cos_theta_t = n_i * cos_theta_t;
				double n_t_cos_theta_i = n_t * cos_theta_i;
				double R_p = std::pow((n_i_cos_theta_t - n_t_cos_theta_i) / (n_i_cos_theta_t + n_t_cos_theta_i), 2.0);
				double R = (R_s + R_p) / 2;
				double T = 1 - R;
				R_rate = R;
				double eta = n_t / n_i;
				Eigen::Vector3d t = r_out.direction / eta + normal * (cos_theta_i / eta - cos_theta_t);
				t = t.normalized();
				Ray r_in(intersection.coords, t); r_in.mesh = intersection.mesh; r_in.Ni = n_i; r_in.in_material = 1 - r_out.in_material;
				if (path_rate < T)
					res = res + T * shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene) / T;
				//std::cout << res << std::endl;
				//system("pause");
			}
			Eigen::Vector3d n = normal;
			Eigen::Vector3d r_in_dir = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
			r_in_dir = r_in_dir.normalized();
			Ray r_in(intersection.coords, r_in_dir); r_in.mesh = intersection.mesh; r_in.Ni = n_t; r_in.in_material = r_out.in_material;
			if (path_rate > 1 - R_rate)
				res = res + R_rate * shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene) / R_rate;
			return res / 0.8;
		}
		
	}

	L_indir = res;
	L_dir = Sample_Light;
	return L_dir + L_indir;
	/*
	if (intersection.mesh->material->light == true)
	{
		//std::cout << intersection.mesh->material->light_attr->Radiance * 0.5 / M_PI << std::endl;
		//system("pause");
		if (intersection.normal.dot(r_out.direction) < 0)
			return intersection.mesh->material->light_attr->Radiance;// *0.5 / M_PI;
		else
			return Eigen::Vector3d::Zero();
	}
	//else
	//	return Eigen::Vector3d::Zero();

	double EPSILON = 0.000001;
	if (false)
	{
		// Contribution from the light source.
		//std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		//int light_mesh_index = Sample_Light(LightMeshes);
		int light_mesh_index = Sample_Light_Area(&scene->lightmesh_area);
		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = (*LightMeshes)[light_mesh_index];
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
		Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh; r_out.Ni = intersection.mesh->material->Ni; r_out.in_material = r_in.in_material;
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(r_out, bvh_root);
		if (abs(block.distance - distance) > EPSILON && block.mesh->material->light == true)
		{
			x_v = block.coords;
			x_vn = block.normal;
			x_vt = block.tex_coords;
			distance = block.distance;
			light_mesh = *block.mesh;
			//std::cout << "new light" << std::endl;
			//system("pause");
		}
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
			L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient) / light_pdf;
			//if (L_dir[0] * 20 * 255 > 200 || L_dir[1] * 20 * 255 > 200 || L_dir[2] * 20 * 255 > 200)
			//{
			//	std::cout << L_dir << std::endl;
			//	system("pause");
			//	return Eigen::Vector3d(-1.0, -1.0, -1.0);
			//}
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;
			Eigen::Vector3d n = intersection.normal;
			Eigen::Vector3d r = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
			r = r.normalized();
			//L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * 
			//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
		}
	}

	// Eigen::Vector3d L_indir = Eigen::Vector3d::Zero();
	//if (false)
	if (psi <= P_RR)
		if (intersection.mesh->material->Ni == r_out.Ni && r_out.in_material == false)
		{
			// Contribution from other reflectors.
			//Test Russian Roulette with probability P_RR;

			//Uniformly sample the hemisphere toward wi(pdf_hemi = 1 / 2pi);
			//Trace a ray r(p, wi);
			//if ray r hit a non - emitting object at q
			//	L_indir = shade(q, -wi) * f_r * cos(theta) / pdf_hemi / P_RR;

			//double phi = (double)rand() / RAND_MAX * 180, theta = (double)rand() / RAND_MAX * 180;

			//double x_1 = (double)rand() / RAND_MAX, x_2 = (double)rand() / RAND_MAX;
			Eigen::Vector3d wi = Eigen::Vector3d::Zero();
			double x_1 = get_random_double(), x_2 = get_random_double();
			double theta = x_1 * M_PI / 2;
			//double z = 1.0 - 2 * x_1; //std::abs(1.0 - x_1);
			double z = get_random_double();
			double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
			wi = Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z);
			wi = toWorld(wi, intersection.normal);
			if (wi.dot(intersection.normal) < 0)
			{
				std::cout << "wi.dot(intersection.normal) < 0" << std::endl;
				system("pause");
			}
			if (false)
				if (r_out.direction.dot(intersection.normal) > 0)
				{
					std::cout << "r_in.direction.dot(intersection.normal) > 0" << std::endl;
					std::cout << r_out.direction << std::endl << intersection.normal << std::endl;

					system("pause");
				}
			if (false)
				if (intersection.mesh->material->Ns != 1)
				{
					Eigen::Vector3d n = intersection.normal;
					Eigen::Vector3d r_in_dir = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
					r_in_dir = r_in_dir.normalized();
					double rate = 0.99;
					double radius = intersection.mesh->material->Ns_radius;

					// radius = std::pow(1 - scene->rate, 1.0 / (intersection.mesh->material->Ns + 1));

					//radius = std::pow(1 - 1.0 / intersection.mesh->material->Ns, 1.0 / (intersection.mesh->material->Ns + 0));

					// radius = 2 * std::sin(std::acos(radius) / 2);

					//radius = std::sqrt(1 - std::pow(radius, 2));
					//radius = 0;
					double x_1 = get_random_double(), x_2 = get_random_double();
					double z = get_random_double();
					z = 2 * z - 1;
					double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
					Eigen::Vector3d wi_temp(r * std::cos(phi), r * std::sin(phi), z);
					wi = r_in_dir + wi_temp * radius;

					wi = wi.normalized();
					//std::cout << "test: " << std::endl;
					//std::cout << wi << std::endl;
					//std::cout << r_out << std::endl;
					//std::cout << wi.dot(r_out) << std::endl;
					//system("pause");
					int num = 0;
					while (intersection.normal.dot(wi) < 0)
					{
						//f_r = Eigen::Vector3d::Zero();
						//std::cout << "intersection.normal.dot(wi) < 0" << std::endl;
						//std::cout << intersection.normal << std::endl << wi << std::endl;
						//system("pause");

						//double radius = std::pow(1 - rate, 1.0 / (intersection.mesh->material->Ns + 1));
						//radius = std::pow(1 - std::pow(radius, 2), 1.0 / 2);
						double x_1 = get_random_double(), x_2 = get_random_double();
						double z = get_random_double();
						z = 2 * z - 1;
						double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
						Eigen::Vector3d wi_temp(r * std::cos(phi), r * std::sin(phi), z);
						wi = r_in_dir + wi_temp * radius;
						wi = wi.normalized();
						num += 1;
						//if (false)
						if (num == 1000)
						{
							std::cout << "num = 1000" << std::endl;
							std::cout << intersection.normal << std::endl << r_in_dir << std::endl << wi << std::endl;
							system("pause");
						}
					}
				}

			if (intersection.mesh->material->Ns != 1)
			{
				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
				r = r.normalized();
				while (true)
				{
					double xi_0 = get_random_double(), xi_1 = get_random_double(), temp_ = 1.0 / (intersection.mesh->material->Ns + 1);
					double cos_alpha = std::pow(xi_0, temp_), phi = 2 * M_PI * xi_1;
					double sin_alpha = std::pow(1 - std::pow(cos_alpha, 2), 0.5);
					double cos_phi = std::cos(phi), sin_phi = std::sin(phi);
					wi = Eigen::Vector3d(sin_alpha * cos_phi, sin_alpha * sin_phi, cos_alpha);
					wi = toWorld(wi, r);
					wi = wi.normalized();
					if (wi.dot(intersection.normal) > 0)
						break;
				}
			}
			if (wi.dot(intersection.normal) > 0)
			{
				Ray ray(intersection.coords, wi); ray.mesh = intersection.mesh; ray.Ni = intersection.mesh->material->Ni; ray.in_material = r_out.in_material;

				double pdf_hemi = 1.0 * 0.5 / M_PI;
				//BVH* node = nullptr;
				Intersection q = Intersect(ray, bvh_root);
				if (q.happened == true)
				{
					//std::cout << q.distance << std::endl;
					//std::cout << q.mesh->v[0] << " " << q.mesh->v[1] << " " << q.mesh->v[2] << std::endl;
					//system("pause");
					//std::cout << q.mesh->Kd[0] << " " << q.mesh->Kd[1] << " " << q.mesh->Kd[2] << std::endl;
					if (q.happened == true)
					{
						//this is to non-emit material
						//Eigen::Vector3d f_r = q.mesh->material->light_attr->Radiance / M_PI;
						Eigen::Vector3d dir = q.coords - intersection.coords;
						dir = dir.normalized();
						if (intersection.normal.dot(dir) < 0)
						{
							//f_r = Eigen::Vector3d::Zero();
							std::cout << "intersection.normal.dot(dir) < 0" << std::endl;
							std::cout << intersection.normal << std::endl << dir << std::endl;
							system("pause");
						}
						double cos_theta = intersection.normal.dot(dir);
						Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh; r_out.Ni = intersection.mesh->material->Ni; r_out.in_material = r_in.in_material;
						Eigen::Vector3d shading_res;
						shading_res = shade_4(r_out, depth + 1, bvh_root, LightMeshes, scene);
						Eigen::Vector3d f_r_Kd = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
						Eigen::Vector3d f_r_Ks = Eigen::Vector3d::Zero();
						//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
						if (intersection.mesh->material->map_Kd_name != "")
						{
							//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
							f_r_Kd = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
							f_r_Kd = Eigen::Vector3d(std::pow(f_r_Kd[0], 2.2), std::pow(f_r_Kd[1], 2.2), std::pow(f_r_Kd[2], 2.2));
							//std::cout << f_r << std::endl;
							//system("pause");
						}
						else
							f_r_Kd = intersection.mesh->material->Kd;// *0.5 / M_PI;
						if (intersection.mesh->material->map_Ks_name != "")
						{
							f_r_Ks = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
							f_r_Ks = Eigen::Vector3d(std::pow(f_r_Ks[0], 2.2), std::pow(f_r_Ks[1], 2.2), std::pow(f_r_Ks[2], 2.2));
						}
						else
							f_r_Ks = intersection.mesh->material->Ks;// *0.5 / M_PI;
						if (f_r_Kd[0] + f_r_Ks[0] > 1 || f_r_Kd[1] + f_r_Ks[1] > 1 || f_r_Kd[2] + f_r_Ks[2] > 1)
							f_r_correction(f_r_Kd, f_r_Ks);
						//double rho_d = f_r_Kd.sum();
						//double rho_s = f_r_Ks.sum();
						//double xi = get_random_double() * (rho_d + rho_s);
						f_r_Kd = f_r_Kd / M_PI;// f_r = f_r / 0.5;
						double type = 0;
						if (intersection.mesh->material->Kd[0] != 0 || intersection.mesh->material->Kd[1] != 0 || intersection.mesh->material->Kd[2] != 0)
							type += 1;
						if (intersection.mesh->material->Ks[0] != 0 || intersection.mesh->material->Ks[1] != 0 || intersection.mesh->material->Ks[2] != 0)
							type += 1;
						type = 1;
						Eigen::Vector3d total_energy = intersection.mesh->material->Kd + intersection.mesh->material->Ks;
						Eigen::Vector3d total_energy_inverse(1.0 / total_energy[0], 1.0 / total_energy[1], 1.0 / total_energy[2]);
						if (intersection.normal.dot(r_out.direction) < 0)
						{
							f_r_Kd = Eigen::Vector3d::Zero();
							f_r_Ks = Eigen::Vector3d::Zero();
							std::cout << "intersection.normal.dot(r_out.direction) < 0" << std::endl;
							std::cout << ray.direction << std::endl << r_out.direction << std::endl;
							system("pause");
						}

						Eigen::Vector3d res_;
						res_ = f_r_Kd.cwiseProduct(shading_res) * cos_theta / pdf_hemi / P_RR;
						res_ = get_Radiance_2(shading_res, f_r_Kd, cos_theta, pdf_hemi) / P_RR;
						L_indir += res_;
						//L_indir += res_.cwiseProduct(intersection.mesh->material->Kd).cwiseProduct(total_energy_inverse);

						Eigen::Vector3d n = intersection.normal;
						Eigen::Vector3d r = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
						r = r.normalized();

						double cos_alpha = r.dot(dir);
						//res_ = f_r.cwiseProduct(shading_res) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI / type;// *cos_theta;
						//res_ = f_r_Ks.cwiseProduct(shading_res) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR / cos_theta;
						res_ = get_Radiance_1(shading_res, f_r_Ks, std::max(0.0, cos_alpha), intersection.mesh->material->Ni, pdf_hemi, cos_theta) / P_RR;
						L_indir += res_;
						//L_indir += res_.cwiseProduct(intersection.mesh->material->Ks).cwiseProduct(total_energy_inverse);
						//L_indir += p.mesh->material->Ks * shading_res * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / pdf_hemi / P_RR;

					}
				}
			}
		}
		else
		{
			double n_i = r_out.Ni;
			double n_t = intersection.mesh->material->Ni;
			if (r_out.in_material == true)
				n_t = 1;
			Eigen::Vector3d normal = intersection.normal;
			if (r_out.direction.dot(normal) > 0)
			{
				normal = -normal;
				//std::cout << "r_in.direction.dot(normal) > 0" << std::endl;
				//system("pause");
			}
			double cos_theta_i = normal.dot(-r_out.direction);
			double sin_theta_i = std::sqrt(1 - n_i * n_i);
			double sin_theta_t = sin_theta_i * n_i / n_t;
			double R_rate = 1;
			double path_rate = get_random_double();
			Eigen::Vector3d res = Eigen::Vector3d::Zero();
			if (std::abs(sin_theta_t) < 1)
			{
				double cos_theta_t = std::sqrt(1 - sin_theta_t * sin_theta_t);
				double n_i_cos_theta_i = n_i * cos_theta_i;
				double n_t_cos_theta_t = n_t * cos_theta_t;
				double R_s = std::pow((n_i_cos_theta_i - n_t_cos_theta_t) / (n_i_cos_theta_i + n_t_cos_theta_t), 2.0);
				double n_i_cos_theta_t = n_i * cos_theta_t;
				double n_t_cos_theta_i = n_t * cos_theta_i;
				double R_p = std::pow((n_i_cos_theta_t - n_t_cos_theta_i) / (n_i_cos_theta_t + n_t_cos_theta_i), 2.0);
				double R = (R_s + R_p) / 2;
				double T = 1 - R;
				R_rate = R;
				double eta = n_t / n_i;
				Eigen::Vector3d t = r_out.direction / eta + normal * (cos_theta_i / eta - cos_theta_t);
				t = t.normalized();
				Ray r_in(intersection.coords, t); r_in.mesh = intersection.mesh; r_in.Ni = intersection.mesh->material->Ni; r_in.in_material = 1 - r_out.in_material;
				if (path_rate < T)
					res = res + T * shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene) / T;
				//std::cout << res << std::endl;
				//system("pause");
			}
			Eigen::Vector3d n = normal;
			Eigen::Vector3d r_in_dir = 2 * (-r_out.direction.dot(n)) * n - -r_out.direction;
			r_in_dir = r_in_dir.normalized();
			Ray r_in(intersection.coords, r_in_dir); r_in.mesh = intersection.mesh; r_in.Ni = intersection.mesh->material->Ni; r_in.in_material = r_out.in_material;
			if (path_rate > 1 - R_rate)
				res = res + R_rate * shade_4(r_in, depth + 1, bvh_root, LightMeshes, scene) / R_rate;
			return res / P_RR;
		}
	//return Eigen::Vector3d(255, 0, 0);
	*/
	return L_dir + L_indir;

}

Eigen::Vector3d shade_3(Ray r_in, int depth, BVH *bvh_root, std::vector<TriangleMesh> *LightMeshes, Scene *scene)
{
	if (depth == shading_depth) return Eigen::Vector3d::Zero();
	//if (depth == 15) return Eigen::Vector3d::Zero();
	double psi = get_random_double();//(double)rand() / RAND_MAX;

	//std::vector<double> *lightmesh_area = &scene->lightmesh_area;
	double total_lightmesh_area = scene->total_lightmesh_area;

	Eigen::Vector3d L_dir = Eigen::Vector3d::Zero();
	Eigen::Vector3d L_indir = Eigen::Vector3d::Zero();
	double weight_Sample_Light = 0;
	double weight_BRDF = 0;
	bool weight_Sample_Light_cri = false;
	bool weight_BRDF_cri = false;
	Eigen::Vector3d Sample_Light = Eigen::Vector3d::Zero();
	Eigen::Vector3d BRDF = Eigen::Vector3d::Zero();

	Intersection intersection = Intersect(r_in, bvh_root);
	if (intersection.happened == false)
		return Eigen::Vector3d::Zero();
	if (r_in.direction.dot(intersection.normal) > 0 && r_in.Ni == intersection.mesh->material->Ni && r_in.in_material == false)
	{
		//std::cout << "r_in.direction.dot(intersection.normal) > 0" << std::endl;
		//std::cout << r_in.direction << std::endl << intersection.normal << std::endl;
		//
		//system("pause");
		return Eigen::Vector3d::Zero();

	}
	if (intersection.mesh->material->light == true)
	{
		//std::cout << intersection.mesh->material->light_attr->Radiance * 0.5 / M_PI << std::endl;
		//system("pause");
		if (intersection.normal.dot(r_in.direction) < 0)
			return intersection.mesh->material->light_attr->Radiance;// *0.5 / M_PI;
		else
			return Eigen::Vector3d::Zero();
	}
	//else
	//	return Eigen::Vector3d::Zero();

	double EPSILON = 0.000001;
	int sample_each_method = 1;
	//if (false)
	for (int i = 0; i < sample_each_method; i++)
	{
		// Contribution from the light source.
		//std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		//int light_mesh_index = Sample_Light(LightMeshes);
		int light_mesh_index;
		//light_mesh_index = Sample_Light_Area(lightmesh_area);
		light_mesh_index = Sample_Light_Uniform(&scene->lightmesh_area, 0, scene->lightmesh_area.size());
		
		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = (*LightMeshes)[light_mesh_index];
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
		Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh; r_out.Ni = intersection.mesh->material->Ni; r_out.in_material = r_in.in_material;
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(r_out, bvh_root);
		if (false)
		if (block.distance < distance && block.mesh->material->light == true)
		{
			x_v = block.coords;
			x_vn = block.normal;
			x_vt = block.tex_coords;
			distance = block.distance;
			light_mesh = *block.mesh;
			//std::cout << "new light" << std::endl;
			//system("pause");
		}
		if (abs(block.distance - distance) < EPSILON)
		{
			light_pdf = 1.0 / get_area(light_mesh.v) / (double)scene->lightmesh_area.size();
			double cos_theta_p = intersection.normal.dot(dir);
			double cos_theta_x = x_vn.dot(-dir);
			weight_Sample_Light = get_area(light_mesh.v) * cos_theta_x / std::pow(distance, 2) * scene->lightmesh_area.size();
			weight_Sample_Light_cri = true;
			//weight_Sample_Light = weight_Sample_Light / get_area(light_mesh.v) * lightmesh_area->back();

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
			Eigen::Vector3d f_r_Kd = Eigen::Vector3d::Zero();// = intersection.mesh->material->Kd;// *0.5 / M_PI;
			Eigen::Vector3d f_r_Ks = Eigen::Vector3d::Zero();

			if (intersection.mesh->material->map_Kd_name != "")
			{
				//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
				f_r_Kd = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
				f_r_Kd = Eigen::Vector3d(std::pow(f_r_Kd[0], 2.2), std::pow(f_r_Kd[1], 2.2), std::pow(f_r_Kd[2], 2.2));
				//f_r = Eigen::Vector3d(0.8, 0.8, 0.8);
				//std::cout << f_r << std::endl;
				//system("pause");
			}
			else
				f_r_Kd = intersection.mesh->material->Kd;// *0.5 / M_PI;
			//if (intersection.normal.dot(dir) <= 0)
			
			if (intersection.mesh->material->map_Ks_name != "")
			{
				//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
				f_r_Ks = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
				f_r_Ks = Eigen::Vector3d(std::pow(f_r_Ks[0], 2.2), std::pow(f_r_Ks[1], 2.2), std::pow(f_r_Ks[2], 2.2));
				//f_r = Eigen::Vector3d(0.8, 0.8, 0.8);
				//std::cout << f_r << std::endl;
				//system("pause");
			}
			else
				f_r_Ks = intersection.mesh->material->Ks;// *0.5 / M_PI;
			if (f_r_Kd[0] + f_r_Ks[0] > 1 || f_r_Kd[1] + f_r_Ks[1] > 1 || f_r_Kd[2] + f_r_Ks[2] > 1)
				f_r_correction(f_r_Kd, f_r_Ks);
			if (x_vn.dot(-dir) <= 0)
			{
				f_r_Kd = Eigen::Vector3d::Zero();
				f_r_Ks = Eigen::Vector3d::Zero();
			}
			f_r_Kd = f_r_Kd / M_PI;// f_r = f_r / 0.5;
			//if (p.coords.dot())
			//std::cout << light_mesh.material->light_attr->Radiance << std::endl;
			//std::cout << f_r << std::endl;
			//std::cout << cos_theta_p << std::endl;
			//std::cout << cos_theta_x << std::endl;
			//std::cout << distance << std::endl;
			//std::cout << light_pdf << std::endl;
			//system("pause");
			
			Sample_Light += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r_Kd) * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient) / light_pdf;
			//Sample_Light += get_Radiance_0(light_mesh.material->light_attr->Radiance, f_r_Kd, cos_theta_x, cos_theta_p, distance, distance_coefficient, light_pdf);
			
			//if (L_dir[0] * 20 * 255 > 200 || L_dir[1] * 20 * 255 > 200 || L_dir[2] * 20 * 255 > 200)
			//{
			//	std::cout << L_dir << std::endl;
			//	system("pause");
			//	return Eigen::Vector3d(-1.0, -1.0, -1.0);
			//}
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;
			
			Eigen::Vector3d n = intersection.normal;
			Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			r = r.normalized();
			double cos_alpha = r.dot(dir);
			//L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * 
			//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
			Eigen::Vector3d res_;
			res_ = light_mesh.material->light_attr->Radiance.cwiseProduct(f_r_Ks) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / light_pdf * cos_theta_p;
			//res_ = get_Radiance_1(light_mesh.material->light_attr->Radiance, f_r_Ks, cos_alpha, intersection.mesh->material->Ns, light_pdf, cos_theta_p);
			Sample_Light += res_;
		}
	}

	//if (false)
	// reflection
	//if (psi <= P_RR)
	{
		// non-refraction
		if (intersection.mesh->material->Ni == r_in.Ni && r_in.in_material == false)
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
			double z = get_random_double();
			double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
			Eigen::Vector3d wi(r * std::cos(phi), r * std::sin(phi), z);
			wi = toWorld(wi, intersection.normal);
			if (wi.dot(intersection.normal) < 0)
			{
				std::cout << "wi.dot(intersection.normal) < 0" << std::endl;
				system("pause");
			}
			if (false)
				if (r_in.direction.dot(intersection.normal) > 0)
				{
					std::cout << "r_in.direction.dot(intersection.normal) > 0" << std::endl;
					std::cout << r_in.direction << std::endl << intersection.normal << std::endl;

					system("pause");
				}

			// sphere correction
			if (false)
			if (intersection.mesh->material->Ns != 1)
			{
				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r_out = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
				r_out = r_out.normalized();
				double rate = 0.99;
				double radius = intersection.mesh->material->Ns_radius;

				// radius = std::pow(1 - scene->rate, 1.0 / (intersection.mesh->material->Ns + 1));

				//radius = std::pow(1 - 1.0 / intersection.mesh->material->Ns, 1.0 / (intersection.mesh->material->Ns + 0));

				// radius = 2 * std::sin(std::acos(radius) / 2);

				//radius = std::sqrt(1 - std::pow(radius, 2));
				//radius = 0;
				double x_1 = get_random_double(), x_2 = get_random_double();
				double z = get_random_double();
				z = 2 * z - 1;
				double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
				Eigen::Vector3d wi_temp(r * std::cos(phi), r * std::sin(phi), z);
				wi = r_out + wi_temp * radius;

				wi = wi.normalized();
				//std::cout << "test: " << std::endl;
				//std::cout << wi << std::endl;
				//std::cout << r_out << std::endl;
				//std::cout << wi.dot(r_out) << std::endl;
				//system("pause");
				int num = 0;
				while (intersection.normal.dot(wi) < 0)
				{
					//f_r = Eigen::Vector3d::Zero();
					//std::cout << "intersection.normal.dot(wi) < 0" << std::endl;
					//std::cout << intersection.normal << std::endl << wi << std::endl;
					//system("pause");

					//double radius = std::pow(1 - rate, 1.0 / (intersection.mesh->material->Ns + 1));
					//radius = std::pow(1 - std::pow(radius, 2), 1.0 / 2);
					double x_1 = get_random_double(), x_2 = get_random_double();
					double z = get_random_double();
					z = 2 * z - 1;
					double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
					Eigen::Vector3d wi_temp(r * std::cos(phi), r * std::sin(phi), z);
					wi = r_out + wi_temp * radius;
					wi = wi.normalized();
					num += 1;
					//if (false)
					if (num == 1000)
					{
						std::cout << "num = 1000" << std::endl;
						std::cout << intersection.normal << std::endl << r_out << std::endl << wi << std::endl;
						system("pause");
					}
				}
			}

			// derivation from http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.28.6852&rep=rep1&type=pdf
			//if (false)
			if (intersection.mesh->material->Ns != 1)
			{
				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
				r = r.normalized();
				while (true)
				{
					double xi_0 = get_random_double(), xi_1 = get_random_double(), temp_ = 1.0 / (intersection.mesh->material->Ns + 1);
					double cos_alpha = std::pow(xi_0, temp_), phi = 2 * M_PI * xi_1;
					double sin_alpha = std::pow(1 - std::pow(cos_alpha, 2), 0.5);
					double cos_phi = std::cos(phi), sin_phi = std::sin(phi);
					wi = Eigen::Vector3d(sin_alpha * cos_phi, sin_alpha * sin_phi, cos_alpha);
					wi = toWorld(wi, r);
					wi = wi.normalized();
					if (wi.dot(intersection.normal) > 0)
						break;
				}
			}

			Ray ray(intersection.coords, wi); ray.mesh = intersection.mesh; ray.Ni = intersection.mesh->material->Ni; ray.in_material = r_in.in_material;

			double pdf_hemi = 1.0 * 0.5 / M_PI;
			//BVH* node = nullptr;
			Intersection q = Intersect(ray, bvh_root);
			
			if (q.happened == true)
			{
				Eigen::Vector3d dir = q.coords - intersection.coords;
				double cos_theta = intersection.normal.dot(dir);
				if (q.mesh->material->light == true && q.normal.dot(-dir) > 0)
				{
					weight_BRDF = 0.5 / M_PI;
					weight_BRDF_cri = true;
					Eigen::Vector3d f_r_Kd = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
					Eigen::Vector3d f_r_Ks = Eigen::Vector3d::Zero();
						//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
					if (intersection.mesh->material->map_Kd_name != "")
					{
						//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
						f_r_Kd = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
						f_r_Kd = Eigen::Vector3d(std::pow(f_r_Kd[0], 2.2), std::pow(f_r_Kd[1], 2.2), std::pow(f_r_Kd[2], 2.2));
						//std::cout << f_r << std::endl;
						//system("pause");
					}
					else
						f_r_Kd = intersection.mesh->material->Kd;
					if (intersection.mesh->material->map_Ks_name != "")
					{
						f_r_Ks = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
						f_r_Ks = Eigen::Vector3d(std::pow(f_r_Ks[0], 2.2), std::pow(f_r_Ks[1], 2.2), std::pow(f_r_Ks[2], 2.2));
					}
					else
						f_r_Ks = intersection.mesh->material->Ks;// *0.5 / M_PI;
					if (f_r_Kd[0] + f_r_Ks[0] > 1 || f_r_Kd[1] + f_r_Ks[1] > 1 || f_r_Kd[2] + f_r_Ks[2] > 1)
						f_r_correction(f_r_Kd, f_r_Ks);
					f_r_Kd = f_r_Kd / M_PI;
					Eigen::Vector3d res_ = f_r_Kd.cwiseProduct(q.mesh->material->light_attr->Radiance) * cos_theta / pdf_hemi;// / P_RR;
					//res_ = get_Radiance_2(q.mesh->material->light_attr->Radiance, f_r_Kd, cos_theta, pdf_hemi);
					BRDF += res_;
					
					Eigen::Vector3d n = intersection.normal;
					Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
					r = r.normalized();
					double cos_alpha = r.dot(dir);
					//res_ = f_r.cwiseProduct(shading_res) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI / type;// *cos_theta;
					res_ = f_r_Ks.cwiseProduct(q.mesh->material->light_attr->Radiance) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi * cos_theta;
					//res_ = get_Radiance_1(q.mesh->material->light_attr->Radiance, f_r_Ks, cos_alpha, intersection.mesh->material->Ns, pdf_hemi, cos_theta);
					BRDF += res_;

				}
				//std::cout << q.distance << std::endl;
				//std::cout << q.mesh->v[0] << " " << q.mesh->v[1] << " " << q.mesh->v[2] << std::endl;
				//system("pause");
				//std::cout << q.mesh->Kd[0] << " " << q.mesh->Kd[1] << " " << q.mesh->Kd[2] << std::endl;
				
				//double psi = get_random_double();
				if (q.mesh->material->light == false)
				{
					weight_BRDF = 0;
					weight_BRDF_cri = false;
				}

				if (q.mesh->material->light == false && psi <= P_RR)
				{
					//this is to non-emit material
					//Eigen::Vector3d f_r = q.mesh->material->light_attr->Radiance / M_PI;
					Eigen::Vector3d dir = q.coords - intersection.coords;
					dir = dir.normalized();
					if (intersection.normal.dot(dir) < 0)
					{
						//f_r = Eigen::Vector3d::Zero();
						std::cout << "intersection.normal.dot(dir) < 0" << std::endl;
						std::cout << intersection.normal << std::endl << dir << std::endl;
						system("pause");
					}
					double cos_theta = intersection.normal.dot(dir);
					Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh; r_out.Ni = intersection.mesh->material->Ni; r_out.in_material = r_in.in_material;
					Eigen::Vector3d shading_res;
					shading_res = shade_3(r_out, depth + 1, bvh_root, LightMeshes, scene);
					Eigen::Vector3d f_r_Kd = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
					Eigen::Vector3d f_r_Ks = Eigen::Vector3d::Zero();

					//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
					if (intersection.mesh->material->map_Kd_name != "")
					{
						//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
						f_r_Kd = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
						f_r_Kd = Eigen::Vector3d(std::pow(f_r_Kd[0], 2.2), std::pow(f_r_Kd[1], 2.2), std::pow(f_r_Kd[2], 2.2));
						//std::cout << f_r << std::endl;
						//system("pause");
					}
					else
						f_r_Kd = intersection.mesh->material->Kd;// *0.5 / M_PI;
					if (intersection.mesh->material->map_Ks_name != "")
					{
						f_r_Ks = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
						f_r_Ks = Eigen::Vector3d(std::pow(f_r_Ks[0], 2.2), std::pow(f_r_Ks[1], 2.2), std::pow(f_r_Ks[2], 2.2));
					}
					else
						f_r_Ks = intersection.mesh->material->Ks;// *0.5 / M_PI;
					if (f_r_Kd[0] + f_r_Ks[0] > 1 || f_r_Kd[1] + f_r_Ks[1] > 1 || f_r_Kd[2] + f_r_Ks[2] > 1)
						f_r_correction(f_r_Kd, f_r_Ks);
					f_r_Kd = f_r_Kd / M_PI;// f_r = f_r / 0.5;

					double type = 0;
					if (intersection.mesh->material->Kd[0] != 0 || intersection.mesh->material->Kd[1] != 0 || intersection.mesh->material->Kd[2] != 0)
						type += 1;
					if (intersection.mesh->material->Ks[0] != 0 || intersection.mesh->material->Ks[1] != 0 || intersection.mesh->material->Ks[2] != 0)
						type += 1;
					type = 1;
					Eigen::Vector3d total_energy = intersection.mesh->material->Kd + intersection.mesh->material->Ks;
					Eigen::Vector3d total_energy_inverse(1.0 / total_energy[0], 1.0 / total_energy[1], 1.0 / total_energy[2]);
					if (intersection.normal.dot(r_out.direction) < 0)
					{
						f_r_Kd = Eigen::Vector3d::Zero();
						f_r_Ks = Eigen::Vector3d::Zero();
						std::cout << "intersection.normal.dot(r_out.direction) < 0" << std::endl;
						std::cout << ray.direction << std::endl << r_out.direction << std::endl;
						system("pause");
					}
					Eigen::Vector3d res_ = f_r_Kd.cwiseProduct(shading_res) * cos_theta / pdf_hemi / P_RR;
					//res_ = get_Radiance_2(shading_res, f_r_Kd, cos_theta, pdf_hemi) / P_RR;
					L_indir += res_;
					//L_indir += res_.cwiseProduct(intersection.mesh->material->Kd).cwiseProduct(total_energy_inverse);

					Eigen::Vector3d n = intersection.normal;
					Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
					r = r.normalized();
					
					double cos_alpha = r.dot(dir);
					//res_ = f_r.cwiseProduct(shading_res) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI / type;// *cos_theta;
					res_ = f_r_Ks.cwiseProduct(shading_res) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR * cos_theta;
					//res_ = get_Radiance_1(shading_res, f_r_Ks, cos_alpha, intersection.mesh->material->Ns, pdf_hemi, cos_theta) / P_RR;
					L_indir += res_;
					//L_indir += res_.cwiseProduct(intersection.mesh->material->Ks).cwiseProduct(total_energy_inverse);
					//L_indir += p.mesh->material->Ks * shading_res * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / pdf_hemi / P_RR;

				}
			}
		}
		else // refraction
		if (psi <= P_RR)
		{
			double n_i = r_in.Ni;
			double n_t = intersection.mesh->material->Ni;
			if (r_in.in_material == true)
				n_t = 1;
			Eigen::Vector3d normal = intersection.normal;
			if (r_in.direction.dot(normal) > 0)
			{
				normal = -normal;
				//std::cout << "r_in.direction.dot(normal) > 0" << std::endl;
				//system("pause");
			}
			double cos_theta_i = normal.dot(-r_in.direction);
			double sin_theta_i = std::sqrt(1 - n_i * n_i);
			double sin_theta_t = sin_theta_i * n_i / n_t;
			double R_rate = 1;
			double path_rate = get_random_double();
			Eigen::Vector3d res = Eigen::Vector3d::Zero();
			if (std::abs(sin_theta_t) < 1)
			{
				double cos_theta_t = std::sqrt(1 - sin_theta_t * sin_theta_t);
				double n_i_cos_theta_i = n_i * cos_theta_i;
				double n_t_cos_theta_t = n_t * cos_theta_t;
				double R_s = std::pow((n_i_cos_theta_i - n_t_cos_theta_t) / (n_i_cos_theta_i + n_t_cos_theta_t), 2.0);
				double n_i_cos_theta_t = n_i * cos_theta_t;
				double n_t_cos_theta_i = n_t * cos_theta_i;
				double R_p = std::pow((n_i_cos_theta_t - n_t_cos_theta_i) / (n_i_cos_theta_t + n_t_cos_theta_i), 2.0);
				double R = (R_s + R_p) / 2;
				double T = 1 - R;
				R_rate = R;
				double eta = n_t / n_i;
				Eigen::Vector3d t = r_in.direction / eta + normal * (cos_theta_i / eta - cos_theta_t);
				t = t.normalized();
				Ray r_out(intersection.coords, t); r_out.mesh = intersection.mesh; r_out.Ni = n_t; r_out.in_material = 1 - r_in.in_material;
				if (path_rate < T)
					res = res + T * shade_3(r_out, depth + 1, bvh_root, LightMeshes, scene) / T / P_RR;
				//std::cout << res << std::endl;
				//system("pause");
			}
			Eigen::Vector3d n = normal;
			Eigen::Vector3d r_out_dir = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			r_out_dir = r_out_dir.normalized();
			Ray r_out(intersection.coords, r_out_dir); r_out.mesh = intersection.mesh; r_out.Ni = n_t; r_out.in_material = r_in.in_material;
			if (path_rate > 1 - R_rate)
				res = res + R_rate * shade_3(r_out, depth + 1, bvh_root, LightMeshes, scene) / R_rate / P_RR;

			// fraction
			L_indir = res;

			// Kd
			L_indir = L_indir + Eigen::Vector3d::Zero();
		}
	}
	//return Eigen::Vector3d(255, 0, 0);
	
	//weight_Sample_Light = 0;
	//weight_BRDF = 0;
	//std::cout << weight_Sample_Light << std::endl;
	//std::cout << weight_BRDF << std::endl;
	//printf("%.20lf\n", weight_Sample_Light);
	//printf("%.20lf\n", weight_BRDF);
	//std::cout << weight_Sample_Light_cri << std::endl;
	//std::cout << weight_BRDF_cri << std::endl;
	//system("pause");
	if (weight_Sample_Light_cri || weight_BRDF_cri)
		power_correction(weight_Sample_Light, weight_BRDF);
	//if (false)
	if ((weight_Sample_Light_cri || weight_BRDF_cri) && std::abs(weight_Sample_Light + weight_BRDF - 1) > EPSILON)
	{
		std::cout << "BRD + weight_BRDF != 1" << std::endl;
		printf("%.20lf\n", weight_Sample_Light);
		printf("%.20lf\n", weight_BRDF);
		std::cout << weight_Sample_Light_cri << std::endl;
		std::cout << weight_BRDF_cri << std::endl;
		system("pause");
	}
	L_dir = Sample_Light + BRDF;
	if (MIS_alpha == -1)
		L_dir = weight_Sample_Light * Sample_Light + weight_BRDF * BRDF;
	else
		L_dir = MIS_alpha * Sample_Light + (1 - MIS_alpha) * BRDF;
	return L_dir + L_indir;
	return L_dir * weight_Sample_Light + L_indir * weight_BRDF;

}


Eigen::Vector3d shade_2(Ray r_in, int depth, BVH *bvh_root, std::vector<TriangleMesh> *LightMeshes, std::vector<double> *lightmesh_area, double total_lightmesh_area)
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
	for (int i = 0; i < lightmesh_area->size(); i++)
	{
		// Contribution from the light source.
		
		//std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		
		
		//int light_mesh_index = Sample_Light(LightMeshes);
		//int light_mesh_index = Sample_Light(lightmesh_area);
		int light_mesh_index = i;
		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = (*LightMeshes)[light_mesh_index];
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
		Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh;
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(r_out, bvh_root);
		if (false)
		if (abs(block.distance - distance) > EPSILON && block.mesh->material->light == true)
		{
			x_v = block.coords;
			x_vn = block.normal;
			x_vt = block.tex_coords;
			distance = block.distance;
			light_mesh = *block.mesh;
			//std::cout << "new light" << std::endl;
			//system("pause");
		}
		if (abs(block.distance - distance) < EPSILON)
		{
			light_pdf = 1.0 / get_area(light_mesh.v);
			//light_pdf = 1.0 / lightmesh_area->back();
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
			L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient) / light_pdf;
			//if (L_dir[0] * 20 * 255 > 200 || L_dir[1] * 20 * 255 > 200 || L_dir[2] * 20 * 255 > 200)
			//{
			//	std::cout << L_dir << std::endl;
			//	system("pause");
			//	return Eigen::Vector3d(-1.0, -1.0, -1.0);
			//}
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;
			//Eigen::Vector3d n = intersection.normal;
			//Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			//r = r.normalized();
			//L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) *
			//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
			Eigen::Vector3d n = intersection.normal;
			Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			r = r.normalized();
			if (intersection.mesh->material->map_Ks_name != "")
			{
				f_r = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
				f_r = Eigen::Vector3d(std::pow(f_r[0], 2.2), std::pow(f_r[1], 2.2), std::pow(f_r[2], 2.2));
			}
			else
				f_r = intersection.mesh->material->Ks;// *0.5 / M_PI;
			double cos_alpha = r.dot(dir);
			L_dir += f_r.cwiseProduct(light_mesh.material->light_attr->Radiance) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / light_pdf * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient);
		}
	}
	double psi = get_random_double();//(double)rand() / RAND_MAX;
	//double P_RR = 0.8;

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
		double z = get_random_double();
		double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;

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
				Eigen::Vector3d shading_res = shade_2(r_out, depth + 1, bvh_root, LightMeshes, lightmesh_area, total_lightmesh_area);

				Eigen::Vector3d f_r = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
				//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
				if (intersection.mesh->material->map_Kd_name != "")
				{
					//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
					f_r = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
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

				//Eigen::Vector3d n = intersection.normal;
				//Eigen::Vector3d h = -r_in.direction + dir;
				//h = h.normalized();
				//L_indir += p.mesh->material->Ks * shading_res * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / pdf_hemi / P_RR;

				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
				r = r.normalized();
				if (intersection.mesh->material->map_Ks_name != "")
				{
					f_r = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
					f_r = Eigen::Vector3d(std::pow(f_r[0], 2.2), std::pow(f_r[1], 2.2), std::pow(f_r[2], 2.2));
				}
				else
					f_r = intersection.mesh->material->Ks;// *0.5 / M_PI;
				double cos_alpha = r.dot(dir);
				L_indir += f_r.cwiseProduct(shading_res) * std::pow(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI;
			}
		}
	}
	//return Eigen::Vector3d(255, 0, 0);

	return L_dir + L_indir;

}


Eigen::Vector3d shade_1(Ray r_in, int depth, BVH *bvh_root, std::vector<TriangleMesh> *LightMeshes, Scene *scene)
{
	//if (depth == 15) return Eigen::Vector3d::Zero();
	double psi = get_random_double();//(double)rand() / RAND_MAX;

	//std::vector<double> *lightmesh_area = &scene->lightmesh_area;
	double total_lightmesh_area = scene->total_lightmesh_area;

	Eigen::Vector3d L_dir = Eigen::Vector3d::Zero();
	Intersection intersection = Intersect(r_in, bvh_root);
	if (intersection.happened == false)
		return Eigen::Vector3d::Zero();
	if (r_in.direction.dot(intersection.normal) > 0 && r_in.Ni == intersection.mesh->material->Ni && r_in.in_material == false)
	{
		//std::cout << "r_in.direction.dot(intersection.normal) > 0" << std::endl;
		//std::cout << r_in.direction << std::endl << intersection.normal << std::endl;
		//
		//system("pause");
		return Eigen::Vector3d::Zero();

	}
	if (intersection.mesh->material->light == true)
	{
		//std::cout << intersection.mesh->material->light_attr->Radiance * 0.5 / M_PI << std::endl;
		//system("pause");
		if (intersection.normal.dot(r_in.direction) < 0)
			return intersection.mesh->material->light_attr->Radiance;// *0.5 / M_PI;
		else
			return Eigen::Vector3d::Zero();
	}
	//else
	//	return Eigen::Vector3d::Zero();
	
	double EPSILON = 0.000001;
	if (false)
	{
		// Contribution from the light source.
		//std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		//int light_mesh_index = Sample_Light(LightMeshes);
		int light_mesh_index = Sample_Light_Area(&scene->lightmesh_area);
		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = (*LightMeshes)[light_mesh_index];
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
		Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh; r_out.Ni = intersection.mesh->material->Ni; r_out.in_material = r_in.in_material;
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(r_out, bvh_root);
		if (abs(block.distance - distance) > EPSILON && block.mesh->material->light == true)
		{
			x_v = block.coords;
			x_vn = block.normal;
			x_vt = block.tex_coords;
			distance = block.distance;
			light_mesh = *block.mesh;
			//std::cout << "new light" << std::endl;
			//system("pause");
		}
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
			L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient) / light_pdf;
			//if (L_dir[0] * 20 * 255 > 200 || L_dir[1] * 20 * 255 > 200 || L_dir[2] * 20 * 255 > 200)
			//{
			//	std::cout << L_dir << std::endl;
			//	system("pause");
			//	return Eigen::Vector3d(-1.0, -1.0, -1.0);
			//}
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;
			Eigen::Vector3d n = intersection.normal;
			Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			r = r.normalized();
			//L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * 
			//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
		}
	}

	Eigen::Vector3d L_indir = Eigen::Vector3d::Zero();
	//if (false)
	if (psi <= P_RR)
		if (intersection.mesh->material->Ni == r_in.Ni && r_in.in_material == false)
		{
			// Contribution from other reflectors.
			//Test Russian Roulette with probability P_RR;

			//Uniformly sample the hemisphere toward wi(pdf_hemi = 1 / 2pi);
			//Trace a ray r(p, wi);
			//if ray r hit a non - emitting object at q
			//	L_indir = shade(q, -wi) * f_r * cos(theta) / pdf_hemi / P_RR;

			//double phi = (double)rand() / RAND_MAX * 180, theta = (double)rand() / RAND_MAX * 180;

			//double x_1 = (double)rand() / RAND_MAX, x_2 = (double)rand() / RAND_MAX;
			Eigen::Vector3d wi = Eigen::Vector3d::Zero();
			double x_1 = get_random_double(), x_2 = get_random_double();
			double theta = x_1 * M_PI / 2;
			//double z = 1.0 - 2 * x_1; //std::abs(1.0 - x_1);
			double z = get_random_double();
			double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
			wi = Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z);
			wi = toWorld(wi, intersection.normal);
			if (wi.dot(intersection.normal) < 0)
			{
				std::cout << "wi.dot(intersection.normal) < 0" << std::endl;
				system("pause");
			}
			if (false)
			if (r_in.direction.dot(intersection.normal) > 0)
			{
				std::cout << "r_in.direction.dot(intersection.normal) > 0" << std::endl;
				std::cout << r_in.direction << std::endl << intersection.normal << std::endl;

				system("pause");
			}
			if (false)
			if (intersection.mesh->material->Ns != 1)
			{
				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r_out = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
				r_out = r_out.normalized();
				double rate = 0.99;
				double radius = intersection.mesh->material->Ns_radius;
				
				// radius = std::pow(1 - scene->rate, 1.0 / (intersection.mesh->material->Ns + 1));
				
				//radius = std::pow(1 - 1.0 / intersection.mesh->material->Ns, 1.0 / (intersection.mesh->material->Ns + 0));
				
				// radius = 2 * std::sin(std::acos(radius) / 2);
				
				//radius = std::sqrt(1 - std::pow(radius, 2));
				//radius = 0;
				double x_1 = get_random_double(), x_2 = get_random_double();
				double z = get_random_double();
				z = 2 * z - 1;
				double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
				Eigen::Vector3d wi_temp(r * std::cos(phi), r * std::sin(phi), z);
				wi = r_out + wi_temp * radius;
				
				wi = wi.normalized();
				//std::cout << "test: " << std::endl;
				//std::cout << wi << std::endl;
				//std::cout << r_out << std::endl;
				//std::cout << wi.dot(r_out) << std::endl;
				//system("pause");
				int num = 0;
				while (intersection.normal.dot(wi) < 0)
				{
					//f_r = Eigen::Vector3d::Zero();
					//std::cout << "intersection.normal.dot(wi) < 0" << std::endl;
					//std::cout << intersection.normal << std::endl << wi << std::endl;
					//system("pause");
				
					//double radius = std::pow(1 - rate, 1.0 / (intersection.mesh->material->Ns + 1));
					//radius = std::pow(1 - std::pow(radius, 2), 1.0 / 2);
					double x_1 = get_random_double(), x_2 = get_random_double();
					double z = get_random_double();
					z = 2 * z - 1;
					double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
					Eigen::Vector3d wi_temp(r * std::cos(phi), r * std::sin(phi), z);
					wi = r_out + wi_temp * radius;
					wi = wi.normalized();
					num += 1;
					//if (false)
					if (num == 1000)
					{
						std::cout << "num = 1000" << std::endl;
						std::cout << intersection.normal << std::endl << r_out << std::endl << wi << std::endl;
						system("pause");
					}
				}
			}
			
			if (intersection.mesh->material->Ns != 1)
			{
				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
				r = r.normalized();
				while (true)
				{
					double xi_0 = get_random_double(), xi_1 = get_random_double(), temp_ = 1.0 / (intersection.mesh->material->Ns + 1);
					double cos_alpha = std::pow(xi_0, temp_), phi = 2 * M_PI * xi_1;
					double sin_alpha = std::pow(1 - std::pow(cos_alpha, 2), 0.5);
					double cos_phi = std::cos(phi), sin_phi = std::sin(phi);
					wi = Eigen::Vector3d(sin_alpha * cos_phi, sin_alpha * sin_phi, cos_alpha);
					wi = toWorld(wi, r);
					wi = wi.normalized();
					if (wi.dot(intersection.normal) > 0)
						break;
				}
			}
			if (wi.dot(intersection.normal) > 0)
			{
				Ray ray(intersection.coords, wi); ray.mesh = intersection.mesh; ray.Ni = intersection.mesh->material->Ni; ray.in_material = r_in.in_material;

				double pdf_hemi = 1.0 * 0.5 / M_PI;
				//BVH* node = nullptr;
				Intersection q = Intersect(ray, bvh_root);
				if (q.happened == true)
				{
					//std::cout << q.distance << std::endl;
					//std::cout << q.mesh->v[0] << " " << q.mesh->v[1] << " " << q.mesh->v[2] << std::endl;
					//system("pause");
					//std::cout << q.mesh->Kd[0] << " " << q.mesh->Kd[1] << " " << q.mesh->Kd[2] << std::endl;
					if (q.happened == true)
					{
						//this is to non-emit material
						//Eigen::Vector3d f_r = q.mesh->material->light_attr->Radiance / M_PI;
						Eigen::Vector3d dir = q.coords - intersection.coords;
						dir = dir.normalized();
						if (intersection.normal.dot(dir) < 0)
						{
							//f_r = Eigen::Vector3d::Zero();
							std::cout << "intersection.normal.dot(dir) < 0" << std::endl;
							std::cout << intersection.normal << std::endl << dir << std::endl;
							system("pause");
						}
						double cos_theta = intersection.normal.dot(dir);
						Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh; r_out.Ni = intersection.mesh->material->Ni; r_out.in_material = r_in.in_material;
						Eigen::Vector3d shading_res;
						shading_res = shade_1(r_out, depth + 1, bvh_root, LightMeshes, scene);
						Eigen::Vector3d f_r_Kd = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
						Eigen::Vector3d f_r_Ks = Eigen::Vector3d::Zero();
						//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
						if (intersection.mesh->material->map_Kd_name != "")
						{
							//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
							f_r_Kd = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
							f_r_Kd = Eigen::Vector3d(std::pow(f_r_Kd[0], 2.2), std::pow(f_r_Kd[1], 2.2), std::pow(f_r_Kd[2], 2.2));
							//std::cout << f_r << std::endl;
							//system("pause");
						}
						else
							f_r_Kd = intersection.mesh->material->Kd;// *0.5 / M_PI;
						if (intersection.mesh->material->map_Ks_name != "")
						{
							f_r_Ks = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
							f_r_Ks = Eigen::Vector3d(std::pow(f_r_Ks[0], 2.2), std::pow(f_r_Ks[1], 2.2), std::pow(f_r_Ks[2], 2.2));
						}
						else
							f_r_Ks = intersection.mesh->material->Ks;// *0.5 / M_PI;
						if (f_r_Kd[0] + f_r_Ks[0] > 1 || f_r_Kd[1] + f_r_Ks[1] > 1 || f_r_Kd[2] + f_r_Ks[2] > 1)
							f_r_correction(f_r_Kd, f_r_Ks);
						//double rho_d = f_r_Kd.sum();
						//double rho_s = f_r_Ks.sum();
						//double xi = get_random_double() * (rho_d + rho_s);
						f_r_Kd = f_r_Kd / M_PI;// f_r = f_r / 0.5;
						double type = 0;
						if (intersection.mesh->material->Kd[0] != 0 || intersection.mesh->material->Kd[1] != 0 || intersection.mesh->material->Kd[2] != 0)
							type += 1;
						if (intersection.mesh->material->Ks[0] != 0 || intersection.mesh->material->Ks[1] != 0 || intersection.mesh->material->Ks[2] != 0)
							type += 1;
						type = 1;
						Eigen::Vector3d total_energy = intersection.mesh->material->Kd + intersection.mesh->material->Ks;
						Eigen::Vector3d total_energy_inverse(1.0 / total_energy[0], 1.0 / total_energy[1], 1.0 / total_energy[2]);
						if (intersection.normal.dot(r_out.direction) < 0)
						{
							f_r_Kd = Eigen::Vector3d::Zero();
							f_r_Ks = Eigen::Vector3d::Zero();
							std::cout << "intersection.normal.dot(r_out.direction) < 0" << std::endl;
							std::cout << ray.direction << std::endl << r_out.direction << std::endl;
							system("pause");
						}

						Eigen::Vector3d res_;
						res_ = f_r_Kd.cwiseProduct(shading_res) * cos_theta / pdf_hemi / P_RR;
						res_ = get_Radiance_2(shading_res, f_r_Kd, cos_theta, pdf_hemi) / P_RR;
						L_indir += res_;
						//L_indir += res_.cwiseProduct(intersection.mesh->material->Kd).cwiseProduct(total_energy_inverse);

						Eigen::Vector3d n = intersection.normal;
						Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
						r = r.normalized();

						double cos_alpha = r.dot(dir);
						//res_ = f_r.cwiseProduct(shading_res) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI / type;// *cos_theta;
						//res_ = f_r_Ks.cwiseProduct(shading_res) * fast_power(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR / cos_theta;
						res_ = get_Radiance_1(shading_res, f_r_Ks, std::max(0.0, cos_alpha), intersection.mesh->material->Ni, pdf_hemi, cos_theta) / P_RR;
						L_indir += res_;
						//L_indir += res_.cwiseProduct(intersection.mesh->material->Ks).cwiseProduct(total_energy_inverse);
						//L_indir += p.mesh->material->Ks * shading_res * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / pdf_hemi / P_RR;

					}
				}
			}
		}
		else
		{
			double n_i = r_in.Ni;
			double n_t = intersection.mesh->material->Ni;
			if (r_in.in_material == true)
				n_t = 1;
			Eigen::Vector3d normal = intersection.normal;
			if (r_in.direction.dot(normal) > 0)
			{
				normal = -normal;
				//std::cout << "r_in.direction.dot(normal) > 0" << std::endl;
				//system("pause");
			}
			double cos_theta_i = normal.dot(-r_in.direction);
			double sin_theta_i = std::sqrt(1 - n_i * n_i);
			double sin_theta_t = sin_theta_i * n_i / n_t;
			double R_rate = 1;
			double path_rate = get_random_double();
			Eigen::Vector3d res = Eigen::Vector3d::Zero();
			if (std::abs(sin_theta_t) < 1)
			{
				double cos_theta_t = std::sqrt(1 - sin_theta_t * sin_theta_t);
				double n_i_cos_theta_i = n_i * cos_theta_i;
				double n_t_cos_theta_t = n_t * cos_theta_t;
				double R_s = std::pow((n_i_cos_theta_i - n_t_cos_theta_t) / (n_i_cos_theta_i + n_t_cos_theta_t), 2.0);
				double n_i_cos_theta_t = n_i * cos_theta_t;
				double n_t_cos_theta_i = n_t * cos_theta_i;
				double R_p = std::pow((n_i_cos_theta_t - n_t_cos_theta_i) / (n_i_cos_theta_t + n_t_cos_theta_i), 2.0);
				double R = (R_s + R_p) / 2;
				double T = 1 - R;
				R_rate = R;
				double eta = n_t / n_i;
				Eigen::Vector3d t = r_in.direction / eta + normal * (cos_theta_i / eta - cos_theta_t);
				t = t.normalized();
				Ray r_out(intersection.coords, t); r_out.mesh = intersection.mesh; r_out.Ni = n_t; r_out.in_material = 1 - r_in.in_material;
				if (path_rate < T)
					res = res + T * shade_1(r_out, depth + 1, bvh_root, LightMeshes, scene) / T;
				//std::cout << res << std::endl;
				//system("pause");
			}
			Eigen::Vector3d n = normal;
			Eigen::Vector3d r_out_dir = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			r_out_dir = r_out_dir.normalized();
			Ray r_out(intersection.coords, r_out_dir); r_out.mesh = intersection.mesh; r_out.Ni = n_t; r_out.in_material = r_in.in_material;
			if (path_rate > 1 - R_rate)
				res = res + R_rate * shade_1(r_out, depth + 1, bvh_root, LightMeshes, scene) / R_rate;
			return res / P_RR;
		}
	//return Eigen::Vector3d(255, 0, 0);

	return L_dir + L_indir;

}



Eigen::Vector3d shade_0(Ray r_in, int depth, BVH *bvh_root, std::vector<TriangleMesh> *LightMeshes, std::vector<double> *lightmesh_area, double total_lightmesh_area)
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
		//std::vector<TriangleMesh> Light_Mesh = (*LightMeshes);
		double light_pdf = 0;
		//int light_mesh_index = Sample_Light(LightMeshes);
		int light_mesh_index = Sample_Light_Area(lightmesh_area);
		//std::cout << light_mesh_index << std::endl;
		TriangleMesh light_mesh = (*LightMeshes)[light_mesh_index];
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
		Ray r_out(intersection.coords, dir); r_out.mesh = intersection.mesh;
		//if the ray is not blocked in the middle
		//	L_dir = L_i * f_r * cos(theta) * cos(theta') / |x' - p | ^ 2 / pdf_light;
		//BVH* node = nullptr;
		Intersection block = Intersect(r_out, bvh_root);
		if (abs(block.distance - distance) > EPSILON && block.mesh->material->light == true)
		{
			x_v = block.coords;
			x_vn = block.normal;
			x_vt = block.tex_coords;
			distance = block.distance;
			light_mesh = *block.mesh;
			//std::cout << "new light" << std::endl;
			//system("pause");
		}
		if (abs(block.distance - distance) < EPSILON)
		{
			//light_pdf = 1.0 / get_area(light_mesh.v);
			light_pdf = 1.0 / lightmesh_area->back();
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
			L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) * cos_theta_p * cos_theta_x / std::pow(distance, distance_coefficient) / light_pdf;
			//if (L_dir[0] * 20 * 255 > 200 || L_dir[1] * 20 * 255 > 200 || L_dir[2] * 20 * 255 > 200)
			//{
			//	std::cout << L_dir << std::endl;
			//	system("pause");
			//	return Eigen::Vector3d(-1.0, -1.0, -1.0);
			//}
			//L_dir += p.mesh->material->Kd.cwiseProduct(light_mesh.material->light_attr->Radiance);
			//L_dir += p.mesh->material->Kd * light_mesh.material->light_attr.Radiance / M_PI * f_r * cos_theta_p * cos_theta_x / distance / light_pdf;
			//Eigen::Vector3d n = intersection.normal;
			//Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			//r = r.normalized();
			//L_dir += light_mesh.material->light_attr->Radiance.cwiseProduct(f_r) *
			//L_dir += p.mesh->material->Ks * light_mesh.material->light_attr.Radiance / M_PI * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / distance / light_pdf;
			Eigen::Vector3d n = intersection.normal;
			Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
			r = r.normalized();
			if (intersection.mesh->material->map_Ks_name != "")
			{
				f_r = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
				f_r = Eigen::Vector3d(std::pow(f_r[0], 2.2), std::pow(f_r[1], 2.2), std::pow(f_r[2], 2.2));
			}
			else
				f_r = intersection.mesh->material->Ks;// *0.5 / M_PI;
			double cos_alpha = r.dot(dir);
			L_dir += f_r.cwiseProduct(light_mesh.material->light_attr->Radiance) * std::pow(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / light_pdf / P_RR * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI;

		}
	}
	double psi = get_random_double();//(double)rand() / RAND_MAX;
	//double P_RR = 0.8;

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
		double z = get_random_double();
		double r = std::sqrt(1.0 - z * z), phi = 2 * M_PI * x_2;
		
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
				Eigen::Vector3d shading_res = shade_0(r_out, depth + 1, bvh_root, LightMeshes, lightmesh_area, total_lightmesh_area);

				Eigen::Vector3d f_r = Eigen::Vector3d::Zero();// intersection.mesh->material->Kd;// *0.5 / M_PI;
				//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
				if (intersection.mesh->material->map_Kd_name != "")
				{
					//std::cout << intersection.mesh->material->map_Kd_name << std::endl;
					f_r = getColorBilinear(intersection.mesh->material->map_Kd, intersection.tex_coords) / 255.0;
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

				//Eigen::Vector3d n = intersection.normal;
				//Eigen::Vector3d h = -r_in.direction + dir;
				//h = h.normalized();
				//L_indir += p.mesh->material->Ks * shading_res * std::pow(std::max(0.0, n.dot(h)), p.mesh->material->Ns) / pdf_hemi / P_RR;
				Eigen::Vector3d n = intersection.normal;
				Eigen::Vector3d r = 2 * (-r_in.direction.dot(n)) * n - -r_in.direction;
				r = r.normalized();
				if (intersection.mesh->material->map_Ks_name != "")
				{
					f_r = getColorBilinear(intersection.mesh->material->map_Ks, intersection.tex_coords) / 255.0;
					f_r = Eigen::Vector3d(std::pow(f_r[0], 2.2), std::pow(f_r[1], 2.2), std::pow(f_r[2], 2.2));
				}
				else
					f_r = intersection.mesh->material->Ks;// *0.5 / M_PI;
				double cos_alpha = r.dot(dir);
				L_indir += f_r.cwiseProduct(shading_res) * std::pow(std::max(0.0, cos_alpha), intersection.mesh->material->Ns) / pdf_hemi / P_RR * (intersection.mesh->material->Ns + 2) / 2.0 / M_PI;

			}
		}
	}
	//return Eigen::Vector3d(255, 0, 0);

	return L_dir + L_indir;

}


Eigen::Vector3d ray_generation(Scene *scene, Eigen::Vector3d dir, BVH* bvh_root, MeshAttributes* mesh_attributes, int render_choice)
{
	//int N = 5;
	//std::vector<Eigen::Vector3d> pos(N, Eigen::Vector3d::Zero());
	//Uniformly choose N sample positions within the pixel;

	Eigen::Vector3d pixel_radiance = Eigen::Vector3d::Zero();
	//shoot a ray r(camPos, cam_to_sample);
	Ray ray(scene->eye_value, dir); ray.mesh = nullptr; ray.Ni = 1.0; ray.in_material = false;
	//BVH *node = nullptr;
	
	//Intersection p = Intersect(ray, bvh_root);
	//if (p.happened == false) return Eigen::Vector3d::Zero();
	//pixel_radiance += shade(p, -dir, 0, bvh_root, LightMeshes);
	//scene->lightmesh_area;
	//scene->total_lightmesh_area;
	Eigen::Vector3d res;
	if (render_choice == 0)
		res = shade_0(ray, 0, bvh_root, &(mesh_attributes->LightMeshes), &scene->lightmesh_area, scene->total_lightmesh_area);
	if (render_choice == 1)
		res = shade_1(ray, 0, bvh_root, &(mesh_attributes->LightMeshes), scene);
	if (render_choice == 2)
		res = shade_2(ray, 0, bvh_root, &(mesh_attributes->LightMeshes), &scene->lightmesh_area, scene->total_lightmesh_area);
	if (render_choice == 3)
		res = shade_3(ray, 0, bvh_root, &(mesh_attributes->LightMeshes), scene);
	if (render_choice == 4)
	{
		//return Eigen::Vector3d::Zero();
		if (false)
		for (int i = 0; i < 3; i++)
		{
			color_type = i;
			res[i] = shade_4(ray, 0, bvh_root, &(mesh_attributes->LightMeshes), scene)[i];
		}
		color_type = 0;
		res = shade_4(ray, 0, bvh_root, &(mesh_attributes->LightMeshes), scene);
		//res = shade_4(ray, 0, bvh_root, &(mesh_attributes->LightMeshes), scene);
	}
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


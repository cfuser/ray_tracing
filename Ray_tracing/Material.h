#pragma once
#include "Eigen/Eigen"
#include <fstream>
#include "Light.h"
#include "Scene.h"
#include <opencv2/opencv.hpp>
#include <math.h>

struct Material
{
	std::string name;
	Eigen::Vector3d Kd;
	std::string map_Kd_name;
	Eigen::Vector3d Ks;
	std::string map_Ks_name;
	double Ns;
	double Ns_radius;
	double Ni;
	bool light;
	Light* light_attr;
	cv::Mat map_Kd;
	cv::Mat map_Ks;
	Material()
	{
		name = "";
		Kd = Eigen::Vector3d::Zero();
		map_Kd_name = "";
		Ks = Eigen::Vector3d::Zero();
		map_Ks_name = "";
		Ns = 0;
		Ni = 0;
		light = false;
		light_attr = nullptr;
		//light_attr.Radiance = Eigen::Vector3d::Zero();

	}
};

struct Materials
{
	//std::vector<std::string> Light_name;
	std::vector<Material> materials;
	//std::vector<Material> light_materials;
	void LoadFile(std::string file_name, Scene* scene, std::string path);
};
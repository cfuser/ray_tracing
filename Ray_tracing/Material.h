#pragma once
#include "Eigen/Eigen"
#include <fstream>
#include "Light.h"
#include "Scene.h"
#include <opencv2/opencv.hpp>
#include <math.h>

struct Material
{
	std::string name; // name of material
	Eigen::Vector3d Kd; // Kd of material
	std::string map_Kd_name; // Kd map of material
	Eigen::Vector3d Ks; // Ks of material
	std::string map_Ks_name; // Ks map of material
	double Ns; // Ns of material
	double Ns_radius; // Ns radius of material
	double Ni; // Ni of material
	bool light; // light mesh or not, 1 for true, 0 for false
	Light* light_attr; // light mesh attribute
	cv::Mat map_Kd; // map Kd image
	cv::Mat map_Ks; // map Ks image
	Material() // initialize
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
	std::vector<Material> materials; // store all material
	//std::vector<Material> light_materials;
	void LoadFile(std::string file_name, Scene* scene, std::string path); // load materials
};
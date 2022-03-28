#pragma once
#include <Eigen/Eigen>
#include "Light.h"
#include <iostream>
#include <fstream>

extern "C" _declspec(dllexport) void line_(std::string line, std::string &name, std::vector<double> &attr);

struct Scene
{
	Eigen::Vector3d eye_value; // camera and scene defintion
	Eigen::Vector3d lookat_value;
	Eigen::Vector3d up_value;
	double fovy_value;
	int width_value;
	int height_value;
	std::vector<Light> lights; // light list
	Eigen::Vector3d x_;
	Eigen::Vector3d y_;
	Eigen::Vector3d z_;
	std::vector<double> lightmesh_area; // area of light mesh
	double total_lightmesh_area = 0;
	double rate;
	std::vector<std::pair<int, int>> LightIndex; // first and last index of light in triangle mesh
	Scene() // initialize
	{
		//eye_value = Eigen::Vector3d(0, 1, 6.8);
		//lookat_value = Eigen::Vector3d(0, 1, 5.8);
		//up_value = Eigen::Vector3d(0, 1, 0);
		//fovy_value = 19.5;
		//width_value = 1024;
		//height_value = 1024;
		//lights.push_back(Light("Light", Eigen::Vector3d(17, 12, 4)));
		
		//eye_value = Eigen::Vector3d(0, 2.0, 15.0);
		//lookat_value = Eigen::Vector3d(0, 1.69521, 14.0476);
		//up_value = Eigen::Vector3d(0, 0.952421, -0.304787);
		//fovy_value = 27.3909;
		//width_value = 1200;
		//height_value = 900;
		//lights.push_back(Light("Light1", Eigen::Vector3d(901.803, 901.803, 901.803)));
		//lights.push_back(Light("Light2", Eigen::Vector3d(100.000, 100.000, 100.000)));
		//lights.push_back(Light("Light3", Eigen::Vector3d(11.1111, 11.1111, 11.1111)));
		//lights.push_back(Light("Light4", Eigen::Vector3d(1.23457, 1.23457, 1.23457)));
		//lights.push_back(Light("Light5", Eigen::Vector3d(800.000, 800.000, 800.800)));

		//eye_value = Eigen::Vector3d(3.456, 1.212, 3.299);
		//lookat_value = Eigen::Vector3d(2.699, 1.195, 2.645);
		//up_value = Eigen::Vector3d(-0.013, 1.000, -0.011);
		//fovy_value = 39.4305;
		//width_value = 1280;
		//height_value = 720;
		//lights.push_back(Light("Light", Eigen::Vector3d(16.4648, 16.4648, 16.4648)));

		//z_ = lookat_value - eye_value;
		//y_ = up_value;
		//x_ = -y_.cross(z_);
	}
	void LoadFile(std::string name) // load file
	{
		std::vector<std::string> type_name;
		std::vector<std::vector<double>> type_attr;
		std::ifstream ifile(name);
		std::string line;
		//line.resize(100);
		//ifile >> line;
		getline(ifile, line);
		getline(ifile, line);
		for (int i = 0; i < 6; i++)
		{
			std::vector<double> attr;
			getline(ifile, line);
			std::cout << line << std::endl;
			int type_begin = line.find("<");
			int type_end = line.find(" ", type_begin + 1);
			//std::cout << line.substr(type_begin + 1, type_end - type_begin - 1) << " ";
			type_name.push_back(line.substr(type_begin + 1, type_end - type_begin - 1));
			int value_begin = line.find("\"", type_begin + 1);
			int value_end = line.find("\"", value_begin + 1);
			//std::cout << value_begin << " " << value_end << std::endl;
			while (value_begin < value_end)
			{
				int one_value_end = line.find(",", value_begin + 1);
				if (one_value_end == -1) one_value_end = value_end;
				double one_value = stod(line.substr(value_begin + 1, one_value_end - value_begin - 1));
				attr.push_back(one_value);
				//std::cout << one_value << " ";
				
				value_begin = one_value_end;
			}
			type_attr.push_back(attr);
			//std::cout << std::endl;
		}
		getline(ifile, line);
		while (getline(ifile, line))
		if (line != "")
		{
			std::vector<double> attr;
			std::cout << line << std::endl;
			int type_begin = line.find("\"");
			int type_end = line.find("\"", type_begin + 1);
			//std::cout << line.substr(type_begin + 1, type_end - type_begin - 1) << " ";
			type_name.push_back(line.substr(type_begin + 1, type_end - type_begin - 1));
			int value_begin = line.find("\"", type_end + 1);
			int value_end = line.find("\"", value_begin + 1);
			//std::cout << value_begin << " " << value_end << std::endl;
			while (value_begin < value_end)
			{
				int one_value_end = line.find(",", value_begin + 1);
				if (one_value_end == -1) one_value_end = value_end;
				double one_value = stod(line.substr(value_begin + 1, one_value_end - value_begin - 1));
				attr.push_back(one_value);
				//std::cout << one_value << " ";
				value_begin = one_value_end;
			}
			type_attr.push_back(attr);
			//std::cout << std::endl;
		}
		eye_value = Eigen::Vector3d(type_attr[0][0], type_attr[0][1], type_attr[0][2]);
		lookat_value = Eigen::Vector3d(type_attr[1][0], type_attr[1][1], type_attr[1][2]);
		up_value = Eigen::Vector3d(type_attr[2][0], type_attr[2][1], type_attr[2][2]);
		fovy_value = type_attr[3][0];
		width_value = type_attr[4][0];
		height_value = type_attr[5][0];
		for (int i = 6; i < type_attr.size(); i++)
		{
			Eigen::Vector3d color = Eigen::Vector3d(type_attr[i][0], type_attr[i][1], type_attr[i][2]);
			lights.push_back(Light(type_name[i], color));
		}
		z_ = lookat_value - eye_value;
		y_ = up_value;
		x_ = -y_.cross(z_);
	}
};
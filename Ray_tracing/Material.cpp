#include "Material.h"
void Materials::LoadFile(std::string file_name, Scene* scene, std::string path)
{
	materials.clear();
	FILE *fp = fopen(file_name.c_str(), "r");
	std::ifstream ifile(file_name);
	//std::cout << fp << std::endl;
	//std::cout << file_name.c_str() << std::endl;
	std::string tag;
	tag.resize(50);
	char ch;
	while (ifile >> tag)
	{
		if (tag == "newmtl")
		{
			std::string ch;
			Material temp_material;

			ifile >> temp_material.name;

			ifile >> ch >> temp_material.Kd[0] >> temp_material.Kd[1] >> temp_material.Kd[2];
			ifile >> ch >> temp_material.Ks[0] >> temp_material.Ks[1] >> temp_material.Ks[2];
			ifile >> ch >> temp_material.Ns;
			ifile >> ch >> temp_material.Ni;
			temp_material.Ns_radius = std::pow(1 - scene->rate, 1.0 / (temp_material.Ns + 1));
			temp_material.Ns_radius = 2 * std::sin(std::acos(temp_material.Ns_radius) / 2);
			for (auto light_iter = scene->lights.begin(); light_iter != scene->lights.end(); light_iter++)
			{
				if (temp_material.name == light_iter->name)
				{
					temp_material.light = true;
					temp_material.light_attr = &(*light_iter);
					break;
				}
			}
			materials.push_back(temp_material);
		}
		else if (tag == "map_Kd")
		{
			ifile >> materials.back().map_Kd_name;
			std::string map_Kd_file = path + materials.back().map_Kd_name;
			materials.back().map_Kd = cv::imread(map_Kd_file);
			//std::cout << map_Kd_file << std::endl;
			//std::cout << materials.back().map_Kd.empty() << std::endl;
			//std::cout << materials.back().map_Kd.rows << std::endl;
			//std::cout << materials.back().map_Kd.cols << std::endl;
			//std::cout << materials.back().map_Kd.at<uchar>(0, 0) << std::endl;
			//cv::imshow("map_Kd", materials.back().map_Kd);
			//cv::waitKey(0);
			//system("pause");
		}
		else if (tag == "map_Ks")
		{
			ifile >> materials.back().map_Ks_name;
			std::string map_Ks_file = path + materials.back().map_Ks_name;
			materials.back().map_Ks = cv::imread(map_Ks_file);
		}
	}
}
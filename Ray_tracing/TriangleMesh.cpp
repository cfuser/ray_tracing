#include "stdafx.h"
#include "TriangleMesh.h"
#include "Material.h"
#include "Scene.h"
#include "global.h"

Material* getMaterial(std::string mtl_name, Materials* materials)
{
	//To deal
	//shall return the point of materials, rather than auto material
	/*
	for (auto material : (*materials).materials)
	{
		if (material.name == mtl_name)
			return &material;
	}
	*/
	for (auto iter = materials->materials.begin(); iter != materials->materials.end(); iter++)
	{
		if (iter->name == mtl_name)
			return &(*iter);
	}
	std::cout << "don't find corresponding material" << std::endl;
	return nullptr;
}
void MeshAttributes::LoadFile(std::string file_name, Materials *materials, Scene *scene)
{
	FILE *fp = fopen(file_name.c_str(), "r");
	std::ifstream ifile(file_name);
	//std::cout << fp << std::endl;
	//std::cout << file_name.c_str() << std::endl;

	Material *material = nullptr;
	std::string tag;
	tag.resize(20);
	char ch;
	while (ifile >> tag)
	{
		//std::cout << tag << std::endl;
		if (tag == std::string("v"))
		{
			//std::cout << "v" << std::endl;
			Eigen::Vector3d temp;
			// fscanf(fp, "%lf %lf %lf", &temp[0], &temp[1], &temp[2]);
			ifile >> temp[0] >> temp[1] >> temp[2];
			v.push_back(temp);
		}
		else if (tag == "vn")
		{
			//std::cout << "vn" << std::endl;
			Eigen::Vector3d temp;
			//fscanf(fp, "%lf %lf %lf", &temp[0], &temp[1], &temp[2]);
			ifile >> temp[0] >> temp[1] >> temp[2];
			vn.push_back(temp);
		}
		else if (tag == "vt")
		{
			//std::cout << "vt" << std::endl;
			Eigen::Vector2d temp;
			//fscanf(fp, "%lf %lf", &temp[0], &temp[1]);
			ifile >> temp[0] >> temp[1];

			/*
			if (temp[0] < 0 || temp[1] < 0)
			{
				std::cout << "in load\n" << temp << std::endl;
				system("pause");
			}
			*/
			vt.push_back(temp);
		}
		else if (tag == "f")
		{
			//std::cout << "f" << std::endl;
			Mesh temp;
			//fscanf(fp, "%d/%d/%d", &temp.v[0], &temp.vt[0], &temp.vn[0]);
			//fscanf(fp, "%d/%d/%d", &temp.v[1], &temp.vt[1], &temp.vn[1]);
			//fscanf(fp, "%d/%d/%d", &temp.v[2], &temp.vt[2], &temp.vn[2]);
			// temp.order[0] = 0; temp.order[1] = 1; temp.order[2] = 2;
			ifile >> temp.v_index[0] >> ch >> temp.vt_index[0] >> ch >> temp.vn_index[0];
			ifile >> temp.v_index[1] >> ch >> temp.vt_index[1] >> ch >> temp.vn_index[1];
			ifile >> temp.v_index[2] >> ch >> temp.vt_index[2] >> ch >> temp.vn_index[2];
			for (int i = 0; i < 3; i++)
			{
				//temp.vn[i] = temp.vn[i].normalized();
				temp.v_index[i] -= 1;
				temp.vt_index[i] -= 1;
				temp.vn_index[i] -= 1;

				temp.v[i] = v[temp.v_index[i]];
				temp.vt[i] = vt[temp.vt_index[i]];
				temp.vn[i] = vn[temp.vn_index[i]];
				temp.vn[i] = temp.vn[i].normalized();
			}

			Meshes.push_back(temp);

			TriangleMesh _temp;
			for (int i = 0; i < 3; i++)
			{
				_temp.v[i] = temp.v[i];
				_temp.vt[i] = temp.vt[i];
				_temp.vn[i] = temp.vn[i];
			}
			_temp.Barycentric_coordinates = (_temp.v[0] + _temp.v[1] + _temp.v[2]) / 3;
			_temp.MtlNameIndex = MtlName.size() - 1;
			
			//_temp.material = getMaterial(MtlName.back(), materials);
			_temp.material = material;

			_temp.index = TriangleMeshes.size();
			if (_temp.material == nullptr)
				system("pause");
			if (_temp.material->light == true)
			{
				LightMeshes.push_back(_temp);
				//lightmesh_area.push_back(get_area(_temp.v));
			}
			TriangleMeshes.push_back(_temp);
			/*
			for (auto light : (scene->lights))
			{
				if (light.name == MtlName.back())
				{
					LightMeshes.push_back(_temp);
					break;
				}
			}
			*/
			Barycentric_coordinates.push_back(_temp.Barycentric_coordinates);
		}
		else if (tag == "usemtl")
		{
			std::string name;
			name.resize(20);
			ifile >> name;
			MtlName.push_back(name);
			material = getMaterial(MtlName.back(), materials);
			MaterialIndex.push_back(TriangleMeshes.size());
			//std::cout << MaterialIndex.size() << std::endl;
		}
	}
}
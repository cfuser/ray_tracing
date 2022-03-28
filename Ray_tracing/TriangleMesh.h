#pragma once
#include <vector>
#include "Eigen/Eigen"
#include "Material.h"
#include "Scene.h"

struct Point
{
	Eigen::Vector3d v; // coordinate of point
	Eigen::Vector3d vn; // normal of point
	Eigen::Vector2d vt; //texture coordinate of point
};
struct Mesh
{
	int v_index[3]; // coordinate index of point
	int vn_index[3]; // normal index of point
	int vt_index[3]; //texture coordinate of point
	// int order[3];
	Eigen::Vector3d v[3]; // unimportant
	Eigen::Vector3d vn[3]; // unimportant
	Eigen::Vector2d vt[3]; // unimportant
};

struct TriangleMesh
{
	Eigen::Vector3d v[3]; // coordinate of mesh point
	Eigen::Vector3d vn[3]; // normal of mesh point
	Eigen::Vector2d vt[3]; // texture of mesh point
	Eigen::Vector3d Barycentric_coordinates; // unimportant
	Material* material; // point to material
	int MtlNameIndex; // material index
	int index; // index of triangle mesh
	//Eigen::Vector3d Kd;
	//Eigen::Vector3d Ks;
	//Eigen::Vector3d Ns;
	//Eigen::Vector3d Ni;
};

struct MeshAttributes
{
	//std::vector<Point> Points;
	std::vector<Eigen::Vector3d> v; // coordinate of point list
	std::vector<Eigen::Vector3d> vn; // normal of point list
	std::vector<Eigen::Vector2d> vt; // texture of point list
	std::vector<Mesh> Meshes; // mesh list
	std::vector<TriangleMesh> TriangleMeshes; // triangle mesh list
	std::vector<Eigen::Vector3d> Barycentric_coordinates; //unimportant
	std::vector<TriangleMesh> LightMeshes; // light triangle mesh list
	std::vector<std::string> MtlName; // name of material list
	std::vector<int> MaterialIndex; // first index of material
	//std::vector<double> lightmesh_area;
	//double total_lightmesh_area = 0;
	//double *area = nullptr;
	void LoadFile(std::string file_name, Materials* materials, Scene* scene); // load file
};

Material* getMaterial(std::string mtl_name, Materials* materials); // get material by name
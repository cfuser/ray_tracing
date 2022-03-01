#pragma once
#include <vector>
#include "Eigen/Eigen"
#include "Material.h"
#include "Scene.h"

struct Point
{
	Eigen::Vector3d v;
	Eigen::Vector3d vn;
	Eigen::Vector2d vt;
};
struct Mesh
{
	int v_index[3];
	int vn_index[3];
	int vt_index[3];
	// int order[3];
	Eigen::Vector3d v[3];
	Eigen::Vector3d vn[3];
	Eigen::Vector2d vt[3];
};

struct TriangleMesh
{
	Eigen::Vector3d v[3];
	Eigen::Vector3d vn[3];
	Eigen::Vector2d vt[3];
	Eigen::Vector3d Barycentric_coordinates;
	Material* material;
	int MtlNameIndex;
	int index;
	Eigen::Vector3d Kd;
	Eigen::Vector3d Ks;
	Eigen::Vector3d Ns;
	Eigen::Vector3d Ni;
};

struct MeshAttributes
{
	//std::vector<Point> Points;
	std::vector<Eigen::Vector3d> v;
	std::vector<Eigen::Vector3d> vn;
	std::vector<Eigen::Vector2d> vt;
	std::vector<Mesh> Meshes;
	std::vector<TriangleMesh> TriangleMeshes;
	std::vector<Eigen::Vector3d> Barycentric_coordinates;
	std::vector<TriangleMesh> LightMeshes;
	std::vector<std::string> MtlName;
	void LoadFile(std::string file_name, Materials* materials, Scene* scene);
};

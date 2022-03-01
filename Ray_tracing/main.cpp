#include "global.h"
#include "BVH.h"
#include "iostream"
#include <opencv2/opencv.hpp>
#include "Scene.h"
#include "Render.h"
#include <time.h>

//using namespace System::Xml;

inline void UpdateProgress(double progress)
{
	int barWidth = 70;

	std::cout << "[";
	int pos = barWidth * progress;
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "=";
		else if (i == pos) std::cout << ">";
		else std::cout << " ";
	}
	std::cout << "] " << int(progress * 100.0) << " %\r";
	std::cout.flush();
};

int main()
{
	//printf("%.20lf\n", get_random_double());
	std::srand((unsigned)time(NULL));
	Scene scene;
	std::cout << "scene width and height: " << std::endl;
	std::cin >> scene.width_value >> scene.height_value;
	std::string path = "D:/subject/graduate/computer graphics/example-scenes-cg21/cornell-box/";
	//std::cout << "path : " << std::endl;
	//std::cin >> path;
	double multi_radiance = 1;
	std::cout << "multi radiance: " << std::endl;
	std::cin >> multi_radiance;
	int spp = 64;
	std::cout << "input spp: " << std::endl;
	std::cin >> spp;

	std::string materials_filename = "cornell-box.mtl";
	Materials materials;
	
	//time_t t = time(0);
	//tm *ltm = localtime(&t);
	//sprintf(loc_date, "%d%02d%02d", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday);
	//std::cout << "location time: " << loc_date << std::endl;

	{
		time_t t = time(0);
		char tmp[32] = { NULL };
		strftime(tmp, sizeof(tmp), "start time: %Y-%m-%d %H:%M:%S", localtime(&t));
		std::cout << tmp << std::endl;
	}

	auto start = std::chrono::system_clock::now();
	materials.LoadFile(path + materials_filename, &scene, path);
	std::string mesh_attributes_filename = "cornell-box.obj";
	MeshAttributes mesh_attributes;
	mesh_attributes.LoadFile(path + mesh_attributes_filename, &materials, &scene);
	std::cout << mesh_attributes.TriangleMeshes.size() << std::endl;
	/*
	Mesh last_mesh = mesh_attributes.Meshes[mesh_attributes.Meshes.size() - 1];
	std::cout << last_mesh.v_index[0] << " " << last_mesh.v_index[1] << " " << last_mesh.v_index[2] << std::endl;
	last_mesh = mesh_attributes.Meshes[0];
	std::cout << last_mesh.v_index[0] << " " << last_mesh.v_index[1] << " " << last_mesh.v_index[2] << std::endl;
	*/
	BVH* bvh_root = building(nullptr, &(mesh_attributes.TriangleMeshes));
	//system("pause");
	int total_mesh = 0;
	cv::Mat res(scene.height_value, scene.width_value, CV_8UC3);
	//int *vis = new int[mesh_attributes.TriangleMeshes.size()];
	//for (int i = 0; i < mesh_attributes.TriangleMeshes)
	//std::fill(vis, vis + mesh_attributes.TriangleMeshes.size(), 0);
	//int len = mesh_attributes.TriangleMeshes.size();
	//std::cout << "len : " << len << std::endl;
	//for (auto mesh : mesh_attributes.TriangleMeshes)
	//{
	//	vis[mesh.index] = 1;
	//}
	//for (int i = 0; i < len; i++)
	//	if (vis[i] != 1)
	//		std::cout << i << std::endl;
	//std::cout << "BVH" << std::endl;
	//bvh_root->Print(total_mesh, vis);
	//std::cout << mesh_attributes.TriangleMeshes[len - 1].material->light << std::endl;
	//std::cout << mesh_attributes.TriangleMeshes[len - 2].material->light << std::endl;
	//std::cout << mesh_attributes.TriangleMeshes[len - 1].material->light_attr->name << std::endl;
	//std::cout << mesh_attributes.TriangleMeshes[len - 2].material->light_attr->name << std::endl;
	//std::cout << mesh_attributes.TriangleMeshes[len - 1].material->light_attr->Radiance << std::endl;
	//std::cout << mesh_attributes.TriangleMeshes[len - 2].material->light_attr->Radiance << std::endl;
	//
	//for (int i = 0; i < len; i++)
	//	if (vis[i] != 2)
	//		std::cout << i << std::endl;
	//std::cout << "Tra" << std::endl;

	//std::cout << "15056" << vis[15056] << std::endl;
	//std::cout << total_mesh << std::endl;
	/*
	bvh_root->Triangle_Barycentric_Render(&res);
	cv::imshow("Triangle_Barycentric_Render", res);
	*/

	double scale = std::tan(scene.fovy_value * 0.5 / 180 * M_PI);
	//std::cout << std::tan(45 / 180.0 * M_PI) << std::endl;
	
	int multithread = true;
	if (multithread == true)
	{
		const int x_block = 8;
		const int y_block = 8;
		std::thread th[x_block * y_block];
		int strideX = scene.width_value / x_block + 1;
		int strideY = scene.height_value / y_block + 1;
		std::mutex mutex_ins;
		int process = 0;
		auto castRayMultiThread = [&](int min_x, int min_y, int max_x, int max_y) {
			for (uint32_t j = min_y; j < max_y; ++j) {
				int m = j * scene.width_value + min_x;
				for (uint32_t i = min_x; i < max_x; ++i) {
					// generate primary ray direction
					Eigen::Vector3d res_color = Eigen::Vector3d::Zero();
					for (int k = 0; k < spp; k++)
					{
						//std::cout << "k : " << k << std::endl;
						//if (i == 6 && j == 7 && k == 7)
						//{
						//	std::cout << "debug" << std::endl; //printf("debug\n");
						//}
						Ray ray();

						//double x = 2 * ((i + (double)rand() / RAND_MAX) / scene.width_value) - 1, y = 2 * ((j + (double)rand() / RAND_MAX) / scene.height_value) - 1;
						double x = 2 * ((i + get_random_double()) / scene.width_value) - 1, y = 2 * ((j + get_random_double()) / scene.height_value) - 1;
						//double x = 2 * ((i + 0.5) / scene.width_value) - 1, y = 2 * ((j + 0.5) / scene.height_value) - 1;
						x = x * scene.width_value / scene.height_value * scale;
						y = y * scale;
						Eigen::Vector3d dir(x, y, -1);

						dir = dir.normalized();
						res_color += ray_generation(scene.eye_value, dir, bvh_root, &mesh_attributes.LightMeshes) / spp;
					}
					res_color = Eigen::Vector3d(std::pow(res_color[0], 1 / 2.2), std::pow(res_color[1], 1 / 2.2), std::pow(res_color[2], 1 /2.2)); 
					res_color *= 255 * multi_radiance;
					//std::cout << res_color << std::endl;
					//system("pause");
					res_color[0] = res_color[0] > 255 ? 255 : res_color[0];
					res_color[1] = res_color[1] > 255 ? 255 : res_color[1];
					res_color[2] = res_color[2] > 255 ? 255 : res_color[2];

					res.at<cv::Vec3b>(scene.height_value - j - 1, i) = cv::Vec3b(res_color[2], res_color[1], res_color[0]);
					m++;

				}
				{
					std::lock_guard<std::mutex> lock(mutex_ins);
					++process;
					UpdateProgress((double)process / scene.height_value / y_block);
				}

			}
		};
		int id = 0;
		for (int i = 0; i < y_block; i++) {//height
			for (int j = 0; j < x_block; j++) {//width
				th[id] = std::thread(castRayMultiThread, j* strideX, i* strideY, std::min((j + 1)* strideY, scene.width_value), std::min((i + 1) * strideY, scene.height_value));
				id++;
			}
		}
		for (int i = 0; i < x_block * y_block; i++) th[i].join();
	}
	if (multithread == false)
	for (int i = 0 / 2; i < scene.width_value; i++)
	{
		for (int j = 0 / 2; j < scene.height_value; j++)
		{
			//std::cout << i << " " << j << std::endl;
			Eigen::Vector3d res_color = Eigen::Vector3d::Zero();

			for (int k = 0; k < spp; k++)
			{
				//std::cout << "k : " << k << std::endl;
				//if (i == 6 && j == 7 && k == 7)
				//{
				//	std::cout << "debug" << std::endl; //printf("debug\n");
				//}
				Ray ray();

				double x = 2 * ((i + 0.5) / scene.width_value) - 1, y = 2 * ((j + 0.5) / scene.height_value) - 1;
				x = x * scene.width_value / scene.height_value * scale;
				y = y * scale;
				Eigen::Vector3d dir(x, y, -1);

				dir = dir.normalized();
				Eigen::Vector3d res = ray_generation(scene.eye_value, dir, bvh_root, &mesh_attributes.LightMeshes) / spp;
				if (res[0] * spp == -1.0)
				{
					std::cout << i << " " << j << " " << k << std::endl;
					system("pause");
				}
				res_color += res;
			}
			//res_color.pow(2.2);
			//Eigen::pow(res_color, );
			res_color = Eigen::Vector3d(std::pow(res_color[0], 1 / 2.2), std::pow(res_color[1], 1 / 2.2), std::pow(res_color[2], 1 / 2.2));
			res_color *= 255 * multi_radiance;
			//std::cout << res_color << std::endl;
			//system("pause");
			res_color[0] = res_color[0] > 255 ? 255 : res_color[0];
			res_color[1] = res_color[1] > 255 ? 255 : res_color[1];
			res_color[2] = res_color[2] > 255 ? 255 : res_color[2];

			res.at<cv::Vec3b>(scene.height_value - j - 1, i) = cv::Vec3b(res_color[2], res_color[1], res_color[0]);
		}
		UpdateProgress(1.0 * i / scene.width_value);
	}
	auto stop = std::chrono::system_clock::now();
	{
		time_t t = time(0);
		char tmp[32] = { NULL };
		strftime(tmp, sizeof(tmp), "end time: %Y-%m-%d %H:%M:%S", localtime(&t));
		std::cout << tmp << std::endl;
	}
	UpdateProgress(1.0);

	std::cout << "Render complete: \n";
	std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";
	cv::namedWindow("res", cv::WINDOW_NORMAL);
	cv::imshow("res", res);
	cv::imwrite("res.png", res);
	cv::waitKey(0);
	cv::destroyAllWindows();
	system("pause");
	return 0;
}
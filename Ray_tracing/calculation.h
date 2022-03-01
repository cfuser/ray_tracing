#pragma once
#include <cmath>
struct Vector3d
{
	double x, y, z;
	Vector3d() : x(0), y(0), z(0) {}
	//Vector3d(float xx) : x(xx), y(xx), z(xx) {}
	Vector3d(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
	Vector3d operator * (const float &r) const { return Vector3d(x * r, y * r, z * r); }
	Vector3d operator / (const float &r) const { return Vector3d(x / r, y / r, z / r); }

	float norm() { return std::sqrt(x * x + y * y + z * z); }
	Vector3d normalized() {
		double n = std::sqrt(x * x + y * y + z * z);
		return Vector3d(x / n, y / n, z / n);
	}

	Vector3d operator * (const Vector3d &v) const { return Vector3d(x * v.x, y * v.y, z * v.z); }
	Vector3d operator - (const Vector3d &v) const { return Vector3d(x - v.x, y - v.y, z - v.z); }
	Vector3d operator + (const Vector3d &v) const { return Vector3d(x + v.x, y + v.y, z + v.z); }
	Vector3d operator - () const { return Vector3d(-x, -y, -z); }
	Vector3d& operator += (const Vector3d &v) { x += v.x, y += v.y, z += v.z; return *this; }
	Vector3d cross(Vector3d ano)
	{
		Vector3d res;
		res.x = y * ano.z - z * ano.y;
		res.y = z * ano.x - x * ano.z;
		res.z = x * ano.y - y * ano.x;
		return res;
	}
	double dot(Vector3d ano)
	{
		double res = 0;
		res = x * ano.x + y * ano.y + z * ano.z;
		return res;
	}
};
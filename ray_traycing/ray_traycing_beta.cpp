#define RAY_TRAYCER_BETA_
#ifdef RAY_TRAYCER_BETA_1
#define _USE_MATH_DEFINES

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <random>
#include "geometry.h"

using namespace std;


const float eps = 1e-8;

float clamp(const float& inf, const float& sup, const float& value)
{
	return max(inf, min(sup, value));
}


class Parameters
{
public:
	int width;
	int height;
	float FOV;
	Matrix44f camToWorld;
	Vec3f cam_origin;
	Vec3f cam_direction;

	Parameters(int width, int height, float FOV, Matrix44f camToWorld, Vec3f cam_origin, Vec3f cam_direction)
	{
		this->width = width;
		this->height = height;
		this->FOV = FOV;
		this->camToWorld = camToWorld;
		this->cam_origin = cam_origin;
		this->cam_direction = cam_direction;
	}
};


class Triangle
{
public:
	Vec3f v0, v1, v2;
	Vec3f color_v0, color_v1, color_v2;

	Triangle(Vec3f v0, Vec3f v1, Vec3f v2, Vec3f color_v0, Vec3f color_v1, Vec3f color_v2)
	{
		this->v0 = v0;
		this->v1 = v1;
		this->v2 = v2;
		this->color_v0 = color_v0;
		this->color_v1 = color_v1;
		this->color_v2 = color_v2;
	}

	bool rayTriangleIntersectMT(const Vec3f& origin, const Vec3f& direction, const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, float& t, float& u, float& v)
	{
		Vec3f v0v1 = v1 - v0;
		Vec3f v0v2 = v2 - v0;
		Vec3f Pvec = direction.crossProduct(v0v2);
		float det = v0v1.dotProduct(Pvec);
		
#define CULLING_1
#ifdef CULLING_1
		if (det < eps) return false;
#else
		if (fabs(det) < eps) return false;
#endif

		float invDet = 1 / det;
		Vec3f Tvec = origin - v0;
		Vec3f Qvec = Tvec.crossProduct(v0v1);

		u = Tvec.dotProduct(Pvec) * invDet;
		if (u < 0 || u > 1) return false;

		v = direction.dotProduct(Qvec) * invDet;
		if (v < 0 || u + v > 1) return false;

		t = v0v2.dotProduct(Qvec) * invDet;

		return true;
	}


	bool rayTriangleIntersectGeometric(const Vec3f& origin, const Vec3f& direction, const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, float& t, float& u, float& v)
	{
		Vec3f v0v1 = v1 - v0;
		Vec3f v0v2 = v2 - v0;
		Vec3f Normal = v0v1.crossProduct(v0v2);
		float denom = Normal.dotProduct(Normal);

		float NormaldotRayDirection = Normal.dotProduct(direction);
		if (fabs(NormaldotRayDirection) < eps) return false;

		float D = Normal.dotProduct(v0);

		t = (-Normal.dotProduct(origin) + D) / NormaldotRayDirection;
		if (t < 0) return false;

		Vec3f hitPoint = origin + t * direction;
		Vec3f C;

		Vec3f edge0 = v1 - v0;
		Vec3f edgeVP0 = hitPoint - v0;
		C = edge0.crossProduct(edgeVP0);
		if (Normal.dotProduct(C) < 0) return false;

		Vec3f edge1 = v2 - v1;
		Vec3f edgeVP1 = hitPoint - v1;
		C = edge1.crossProduct(edgeVP1);
		if ((u = Normal.dotProduct(C)) < 0)  return false;

		Vec3f edge2 = v0 - v2;
		Vec3f edgeVP2 = hitPoint - v2;
		C = edge2.crossProduct(edgeVP2);
		if ((v = Normal.dotProduct(C)) < 0) return false;

		u /= denom;
		v /= denom;

		return true;
	}
};


Matrix44f LookAt(const Vec3f& origin, const Vec3f& target, const Vec3f& up_temp = Vec3f(0, 1, 0))
{
	Vec3f up_temp_normalize = Vec3f(up_temp).normalize();
	Vec3f forward = Vec3f(origin - target).normalize();
	Vec3f right = up_temp_normalize.crossProduct(forward);
	Vec3f up = forward.crossProduct(right);

	Matrix44f camToWorld;

	camToWorld[0][0] = right.x;
	camToWorld[0][1] = right.y;
	camToWorld[0][2] = right.z;
	camToWorld[1][0] = up.x;
	camToWorld[1][1] = up.y;
	camToWorld[1][2] = up.z;
	camToWorld[2][0] = forward.x;
	camToWorld[2][1] = forward.y;
	camToWorld[2][2] = forward.z;
	camToWorld[3][0] = origin.x;
	camToWorld[3][1] = origin.y;
	camToWorld[3][2] = origin.z;

	return camToWorld;
}


bool trace(const Vec3f& origin, const Vec3f& direction, const vector<unique_ptr<Triangle>>& triangles, float& t_min, float& u_final, float& v_final, const Triangle*& hitTrian)
{
	vector<unique_ptr<Triangle>>::const_iterator iter;
	t_min = INFINITY;
	u_final = INFINITY;
	v_final = INFINITY;
	for (iter = triangles.begin(); iter != triangles.end(); ++iter)
	{
		float t = INFINITY, u = INFINITY, v = INFINITY;
		bool isIntersect;

		isIntersect = (*iter)->rayTriangleIntersectMT(origin, direction, (*iter)->v0, (*iter)->v1, (*iter)->v2, t, u ,v);
		//isIntersect = (*iter)->rayTriangleIntersectGeometric(origin, direction, (*iter)->v0, (*iter)->v1, (*iter)->v2, t, u, v);

		if (isIntersect && (t < t_min))
		{
			hitTrian = (iter)->get();
			t_min = t;
			u_final = u;
			v_final = v;
		}
	}

	return (hitTrian != nullptr);
}



Vec3f castRay(const Vec3f& origin, const Vec3f& direction, const vector<unique_ptr<Triangle>>& triangles)
{
	Vec3f hitColor(0.2f, 0.7f, 0.8f);
	const Triangle* hitTrian = nullptr;
	float t, u, v;

	if (trace(origin, direction, triangles, t, u, v, hitTrian))
	{
		Vec3f hitPoint = origin + t * direction;
		hitColor = (1 - u - v) * hitTrian->color_v0 + u * hitTrian->color_v1 + v * hitTrian->color_v2;
		//hitColor = Vec3f(u, v, 1 - u - v);
	}

	return hitColor;
}


void render(const Parameters& parameters, const vector<unique_ptr<Triangle>>& triangles)
{
	Vec3f* frameBuffer = new Vec3f[parameters.width * parameters.height];
	Vec3f* pixel_curr = frameBuffer;

	float imageAspectRatio = parameters.width / (float)parameters.height;
	float scale = (float)tan(parameters.FOV * 0.5f * (float)M_PI / 180);

	for (int j = 0; j < parameters.height; ++j)
	{
		for (int i = 0; i < parameters.width; ++i)
		{
			float x = (2 * (i + 0.5f) / (float)parameters.width - 1) * imageAspectRatio * scale;
			float y = (1 - 2 * (j + 0.5f) / (float)parameters.height) * scale;

			Vec3f origin;
			parameters.camToWorld.multVecMatrix(parameters.cam_origin, origin);
			Vec3f direction(x, y, -1);
			parameters.camToWorld.multDirMatrix(Vec3f(x, y, -1), direction);
			direction.normalize();

			*(pixel_curr++) = castRay(origin, direction, triangles);
		}
	}

	ofstream ofs("ray_traycing_beta_out.ppm", ios::out | ios::binary);
	ofs << "P6\n" << parameters.width << " " << parameters.height << "\n255\n";
	for (int i = 0; i < parameters.width * parameters.height; ++i)
	{
		unsigned char r = (unsigned char)(255 * clamp(0, 1, frameBuffer[i].x));
		unsigned char g = (unsigned char)(255 * clamp(0, 1, frameBuffer[i].y));
		unsigned char b = (unsigned char)(255 * clamp(0, 1, frameBuffer[i].z));
		ofs << r << g << b;
	}
	ofs.close();

	delete[] frameBuffer;
}



int main()
{
	Matrix44f camToWorld;
	Vec3f cam_origin(6, 0, 5), cam_target(0, 2, 0);
	camToWorld = LookAt(cam_origin, cam_target);

	Parameters parameters(800, 800, 60, camToWorld, cam_origin, cam_target);

	Vec3f v1_0(-1, -1, -5), v1_1(1, -1, -7), v1_2(0, 1, -5);
	Vec3f color_v1_0(1, 0, 0), color_v1_1(0, 1, 0), color_v1_2(0, 0, 1);

	Vec3f v2_0(-3, -1, 0), v2_1(3, 0, -10), v2_2(-1, 5, -5);
	Vec3f color_v2_0(0.6, 0.4, 0.1), color_v2_1(0.1, 0.5, 0.3), color_v2_2(0.1, 0.3, 0.7);
	
	vector<unique_ptr<Triangle>> triangles;
	triangles.push_back(unique_ptr<Triangle>(new Triangle(v1_0, v1_1, v1_2, color_v1_0, color_v1_1, color_v1_2)));
	triangles.push_back(unique_ptr<Triangle>(new Triangle(v2_0, v2_1, v2_2, color_v2_0, color_v2_1, color_v2_2)));

	render(parameters, triangles);
}


#endif

#pragma once

#define _USE_MATH_DEFINES

#include <functional>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <random>
#include "../headers/geometry.h"
#include "terrain_noise_classes.h"

using namespace std;

bool rayTriangleIntersectMT(const Vec3f& origin, const Vec3f& direction, const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, float& t, float& u, float& v);

class Ray
{
public:
	Vec3f orig, dir, invdir;
	int sign[3];

	Ray(const Vec3f& orig, const Vec3f& dir)
	{
		this->orig = orig;
		this->dir = dir;
		invdir = 1 / dir;
		sign[0] = (invdir.x < 0);
		sign[1] = (invdir.y < 0);
		sign[2] = (invdir.z < 0);
	}
};


class AABBox
{
public:
	Vec3f bounds[2];

	AABBox(const Vec3f& b0, const Vec3f& b1)
	{
		bounds[0] = b0;
		bounds[1] = b1;
	}

	bool intersect(const Ray& r, float& t) const
	{
		float tmin, tmax, tymin, tymax, tzmin, tzmax;

		tmin = (bounds[r.sign[0]].x - r.orig.x) * r.invdir.x;
		tmax = (bounds[1 - r.sign[0]].x - r.orig.x) * r.invdir.x;
		tymin = (bounds[r.sign[1]].y - r.orig.y) * r.invdir.y;
		tymax = (bounds[1 - r.sign[1]].y - r.orig.y) * r.invdir.y;

		if ((tmin > tymax) || (tymin > tmax))
			return false;

		if (tymin > tmin)
			tmin = tymin;
		if (tymax < tmax)
			tmax = tymax;

		tzmin = (bounds[r.sign[2]].z - r.orig.z) * r.invdir.z;
		tzmax = (bounds[1 - r.sign[2]].z - r.orig.z) * r.invdir.z;

		if ((tmin > tzmax) || (tzmin > tmax))
			return false;

		if (tzmin > tmin)
			tmin = tzmin;
		if (tzmax < tmax)
			tmax = tzmax;

		t = tmin;

		if (t < 0) {
			t = tmax;
			if (t < 0) return false;
		}

		return true;
	}
};


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


class Object
{
public:
	AABBox* bbox = nullptr;
	Object() {}
	virtual ~Object() {}
	virtual bool intersect(const Ray&, float&, int&, float&, float&) const = 0;
	virtual void getSurfaceProperties(const Vec3f&, const Vec3f&, const int&, const float&, const float&, Vec3f&, Vec2f&) const = 0;
	virtual void vMinMax(Vec3f&, Vec3f&) const = 0;
};


class TriangleMesh : public Object
{
public:
	int numTris;
	unique_ptr<Vec3f[]> P;
	unique_ptr<int[]> trisIndex;
	unique_ptr<Vec3f[]> N;
	unique_ptr<Vec2f[]> texCoordinates;

	TriangleMesh
	(
		const int nfaces,
		const unique_ptr<int[]>& faceIndex,
		const unique_ptr<int[]>& vertsIndex,
		const unique_ptr<Vec3f[]>& verts,
		unique_ptr<Vec3f[]>& normals,
		unique_ptr<Vec2f[]>& st
	)
	{
		this->numTris = 0;

		int k = 0, maxVertIndex = 0;

		for (int i = 0; i < nfaces; ++i)
		{
			numTris += faceIndex[i] - 2;
			for (int j = 0; j < faceIndex[i]; ++j)
				if (vertsIndex[(long long)k + j] > maxVertIndex)
					maxVertIndex = vertsIndex[(long long)k + j];
			k += faceIndex[i];
		}
		maxVertIndex += 1;

		P = unique_ptr<Vec3f[]>(new Vec3f[maxVertIndex]);
		for (int i = 0; i < maxVertIndex; ++i)
			P[i] = verts[i];

		trisIndex = unique_ptr<int[]>(new int[numTris * 3LL]);
		N = unique_ptr<Vec3f[]>(new Vec3f[numTris * 3]);
		unique_ptr<Vec3f[]> N_temp = unique_ptr<Vec3f[]>(new Vec3f[maxVertIndex]);
		texCoordinates = unique_ptr<Vec2f[]>(new Vec2f[numTris * 3]);
		int l = 0;

		for (int i = 0, k = 0; i < nfaces; ++i)
		{
			for (int j = 0; j < faceIndex[i] - 2; ++j)
			{
				trisIndex[l] = vertsIndex[k];
				trisIndex[l + 1LL] = vertsIndex[(long long)k + j + 1LL];
				trisIndex[l + 2LL] = vertsIndex[(long long)k + j + 2LL];

				texCoordinates[l] = st[k];
				texCoordinates[l + 1LL] = st[(long long)k + j + 1LL];
				texCoordinates[l + 2LL] = st[(long long)k + j + 2LL];
				l += 3;
			}
			k += faceIndex[i];
		}
		
		for (int i = 0; i < numTris; ++i)
		{
			const Vec3f v0 = verts[trisIndex[i * 3LL]];
			const Vec3f v1 = verts[trisIndex[i * 3LL + 1LL]];
			const Vec3f v2 = verts[trisIndex[i * 3LL + 2LL]];

			Vec3f hitN = (v1 - v0).crossProduct(v2 - v0);
			hitN.normalize();

			N_temp[trisIndex[i * 3LL]] += hitN;
			N_temp[trisIndex[i * 3LL + 1LL]] += hitN;
			N_temp[trisIndex[i * 3LL + 2LL]] += hitN;
		}

		for (int i = 0; i < maxVertIndex; ++i)
			N_temp[i].normalize();
		
		for (int i = 0, j = 0; i < numTris; ++i, j += 3)
		{
			N[i * 3LL] = N_temp[trisIndex[j]];
			N[i * 3LL + 1LL] = N_temp[trisIndex[j + 1LL]];
			N[i * 3LL + 2LL] = N_temp[trisIndex[j + 2LL]];
		}
	}

	bool intersect(const Ray& ray, float& tNear, int& triIndex, float& u, float& v) const
	{
		int j = 0;
		bool isect = false;
		for (int i = 0; i < numTris; ++i)
		{
			const Vec3f& v0 = P[trisIndex[j]];
			const Vec3f& v1 = P[trisIndex[j + 1LL]];
			const Vec3f& v2 = P[trisIndex[j + 2LL]];
			float t = INFINITY, u_tmp, v_tmp;
			if (rayTriangleIntersectMT(ray.orig, ray.dir, v0, v1, v2, t, u_tmp, v_tmp) && t < tNear)
			{
				tNear = t;
				u = u_tmp;
				v = v_tmp;
				triIndex = i;
				isect = true;
			}
			j += 3;
		}
		return isect;
	}

	void getSurfaceProperties
	(
		const Vec3f& hitPoint,
		const Vec3f& viewDirection,
		const int& triIndex,
		const float& u, const float& v,
		Vec3f& hitNormal,
		Vec2f& hitTextureCoordinates
	) const
	{
		/*
		const Vec3f& v0 = P[trisIndex[triIndex * 3LL]];
		const Vec3f& v1 = P[trisIndex[triIndex * 3LL + 1LL]];
		const Vec3f& v2 = P[trisIndex[triIndex * 3LL + 2LL]];
		hitNormal = (v1 - v0).crossProduct(v2 - v0);
		hitNormal.normalize();
		*/
		
		const Vec2f& st0 = texCoordinates[triIndex * 3LL];
		const Vec2f& st1 = texCoordinates[triIndex * 3LL + 1LL];
		const Vec2f& st2 = texCoordinates[triIndex * 3LL + 2LL];
		hitTextureCoordinates = (1 - u - v) * st0 + u * st1 + v * st2;
		
		const Vec3f& n0 = N[triIndex * 3LL];
		const Vec3f& n1 = N[triIndex * 3LL + 1LL];
		const Vec3f& n2 = N[triIndex * 3LL + 2LL];
		hitNormal = (1 - u - v) * n0 + u * n1 + v * n2;
	}

	void vMinMax(Vec3f& vmin, Vec3f& vmax) const
	{
		vmin = INFINITY;
		vmax = -INFINITY;

		for (int i = 0; i < this->numTris; ++i)
		{
			const Vec3f& v0 = this->P[this->trisIndex[i * 3LL]];
			const Vec3f& v1 = this->P[this->trisIndex[i * 3LL + 1LL]];
			const Vec3f& v2 = this->P[this->trisIndex[i * 3LL + 2LL]];

			float xmin = min(v0.x, min(v1.x, v2.x));
			float ymin = min(v0.y, min(v1.y, v2.y));
			float zmin = min(v0.z, min(v1.z, v2.z));
			float xmax = max(v0.x, max(v1.x, v2.x));
			float ymax = max(v0.y, max(v1.y, v2.y));
			float zmax = max(v0.z, max(v1.z, v2.z));

			if (xmin < vmin.x) vmin.x = xmin;
			if (ymin < vmin.y) vmin.y = ymin;
			if (zmin < vmin.z) vmin.z = zmin;
			if (xmax > vmax.x) vmax.x = xmax;
			if (ymax > vmax.y) vmax.y = ymax;
			if (zmax > vmax.z) vmax.z = zmax;
		}
	}
};

Matrix44f LookAt(const Vec3f& origin, const Vec3f& target, const Vec3f& up_temp = Vec3f(0, 1, 0));

void render(const Parameters& parameters, const vector<unique_ptr<Object>>& objects, const string& FileName);

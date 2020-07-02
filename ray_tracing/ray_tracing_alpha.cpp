#define RAY_TRACER_ALPHA_
#ifdef RAY_TRACER_ALPHA_1
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
#include "../headers/geometry.h"

using namespace std;


float clamp(const float& inf, const float& sup, const float& value)
{
	return max(inf, min(sup, value));
}

Vec3f mix(const Vec3f& a, const Vec3f& b, const float& mixValue)
{
	return a * (1 - mixValue) + b * mixValue;
}


bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
	float discr = b * b - 4 * a * c;
	if (discr < 0) return false;
	else if (discr == 0) x0 = x1 = -0.5f * b / a;
	else 
	{
		float q = (b > 0) ?	float(-0.5f * ((double)b + sqrt(discr))) : float(-0.5f * ((double)b - sqrt(discr)));
		x0 = q / a;
		x1 = c / q;
	}
	return true;
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


class Object
{
public:
	Vec3f color;

	Object() : color(1, 1, 1) {}
	Object(const Vec3f& color) : color(color) {}
	
	virtual bool intersection(const Vec3f&, const Vec3f&, float&) const = 0;
	virtual void GetSurfaceData(const Vec3f&, Vec3f&, Vec2f&) const = 0;
};


class Sphere : public Object
{
public:
	float radius, radius_pow2;
	Vec3f center;

	Sphere(const Vec3f& center, const float& radius, const Vec3f& color) : Object(color)
	{
		this->radius = radius;
		this->radius_pow2 = radius * radius;
		this->center = center;
	}

	bool intersection(const Vec3f& origin, const Vec3f& dir, float& t) const
	{
		float t0, t1;

		Vec3f L = origin - center;
		float a = dir.dotProduct(dir);
		float b = 2 * dir.dotProduct(L);
		float c = L.dotProduct(L) - radius_pow2;
		if (!solveQuadratic(a, b, c, t0, t1)) return false;

		if (t0 > t1) swap(t0, t1);

		if (t0 < 0) 
		{
			t0 = t1;
			if (t0 < 0) return false;
		}

		t = t0;
		return true;
	}

	void GetSurfaceData(const Vec3f& hitPoint, Vec3f& hitNormal, Vec2f& texture) const
	{
		hitNormal = hitPoint - center;
		hitNormal.normalize();
		texture.x = (1 + atan2(hitNormal.z, hitNormal.x) / (float)M_PI) * 0.5f;
		texture.y = acosf(hitNormal.y) / (float)M_PI;
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


bool trace(const Vec3f& origin, const Vec3f& direction, const vector<unique_ptr<Object>>& objects, float& t_min, const Object*& hitObject)
{
	vector<unique_ptr<Object>>::const_iterator iter;
	t_min = INFINITY;
	for (iter = objects.begin(); iter != objects.end(); ++iter)
	{
		float t = INFINITY;
		if ((*iter)->intersection(origin, direction, t) && (t < t_min))
		{
			hitObject = (iter)->get();
			t_min = t;
		}
	}

	return (hitObject != nullptr);
}


Vec3f castRay(const Vec3f& origin, const Vec3f& direction, const vector<unique_ptr<Object>>& objects)
{
	Vec3f hitColor(0.2f, 0.7f, 0.8f);
	const Object* hitObject = nullptr;
	float t;

	if (trace(origin, direction, objects, t, hitObject))
	{
		Vec3f hitNormal, hitPoint = origin + t * direction;
		Vec2f texture;
		hitObject->GetSurfaceData(hitPoint, hitNormal, texture);

		float scale = 4;
		float pattern = (float)((fmodf(texture.x * scale, 1) > 0.5f) ^ (fmodf(texture.y * scale, 1) > 0.5f));
		hitColor = max(0.f, hitNormal.dotProduct(-direction)) * mix(hitObject->color, hitObject->color * 0.8f, pattern);
	}
	
	return hitColor;
}


void render(const Parameters& parameters, const vector<unique_ptr<Object>>& objects)
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
			Vec3f direction;
			parameters.camToWorld.multDirMatrix(Vec3f(x, y, -1), direction);
			direction.normalize();

			*(pixel_curr++) = castRay(origin, direction, objects);
		}
	}

	ofstream ofs("ray_tracing_alpha_out.ppm", ios::out | ios::binary);
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
	Vec3f cam_origin(-5, 0, 10), cam_target(0, 0, -1);
	camToWorld = LookAt(cam_origin, cam_target);

	Parameters parameters(800, 800, 60, camToWorld, cam_origin, cam_target);

	vector<unique_ptr<Object>> objects;
	objects.push_back(unique_ptr<Object>(new Sphere(Vec3f(0, 0, -40), 10, Vec3f(0.3f, 0.1f, 0.1f))));
	objects.push_back(unique_ptr<Object>(new Sphere(Vec3f(-12, 8, -30), 3, Vec3f(0.3f, 0.3f, 0.3f))));
	
	render(parameters, objects);
}


#endif

#define RAY_TRACER_FOXTROT_
#ifdef RAY_TRACER_FOXTROT_1
#define _USE_MATH_DEFINES

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

using namespace std;


const float eps = 1e-8f;

enum class RayType { PRIMARY_RAY, SHADOW_RAY };
enum class MaterialType { DIFFUSE, REFLECTION, REFLECTION_REFRACTION };

float clamp(const float& inf, const float& sup, const float& value)
{
	return max(inf, min(sup, value));
}


bool rayTriangleIntersectMT(const Vec3f& origin, const Vec3f& direction, const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, float& t, float& u, float& v)
{
	Vec3f v0v1 = v1 - v0;
	Vec3f v0v2 = v2 - v0;
	Vec3f Pvec = direction.crossProduct(v0v2);
	float det = v0v1.dotProduct(Pvec);

#define CULLING_
#ifdef CULLING_1
	if (det < eps) return false;
#else
	if (fabs(det) < eps) return false;
#endif

	float invDet = 1 / det;
	Vec3f Tvec = origin - v0;

	u = Tvec.dotProduct(Pvec) * invDet;
	if (u < 0 || u > 1) return false;

	Vec3f Qvec = Tvec.crossProduct(v0v1);
	v = direction.dotProduct(Qvec) * invDet;
	if (v < 0 || u + v > 1) return false;

	t = v0v2.dotProduct(Qvec) * invDet;

	return (t > 0) ? true : false;
}


class Light
{
public:
	Vec3f color;
	float intensity;
	Matrix44f lightToWorld;
	Light(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : lightToWorld(l2w), color(c), intensity(i) {}
	virtual ~Light() {}
	virtual void illuminate(const Vec3f&, Vec3f&, Vec3f&, float&) const = 0;
};


class DistantLight : public Light
{
	Vec3f dir;
public:
	DistantLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : Light(l2w, c, i)
	{
		l2w.multDirMatrix(Vec3f(0, 0, -1), dir);
		dir.normalize();
	}
	void illuminate(const Vec3f& P, Vec3f& lightDir, Vec3f& lightIntensity, float& distance) const
	{
		lightDir = dir;
		lightIntensity = color * intensity;
		distance = INFINITY;
	}
};


class PointLight : public Light
{
	Vec3f pos;
public:
	PointLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : Light(l2w, c, i)
	{
		l2w.multVecMatrix(Vec3f(0), pos);
	}
	void illuminate(const Vec3f& P, Vec3f& lightDir, Vec3f& lightIntensity, float& distance) const
	{
		lightDir = (P - pos);
		// ??????????????????????????????????????????????????
		float r2 = lightDir.norm();
		distance = sqrt(r2);

		lightDir.x /= distance, lightDir.y /= distance, lightDir.z /= distance;
		lightIntensity = color * intensity / (4 * (float)M_PI * r2);
	}
};


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
	Vec3f BGColor = Vec3f(0.2f, 0.7f, 0.8f);
	int maxDepth = 10;
	float bias = 0.0001f;

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
	Matrix44f objToWorld, worldToObj;
	AABBox* bbox = nullptr;
	MaterialType type = MaterialType::DIFFUSE;
	float RefractIndex = 1;
	Vec3f albedo = 0.18f;

	Object(const Matrix44f objToWorld)
	{
		this->objToWorld = objToWorld;
		this->worldToObj = objToWorld.inverse();
	}
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
	bool smoothShading = true;

	TriangleMesh
	(
		const int nfaces,
		const unique_ptr<int[]>& faceIndex,
		const unique_ptr<int[]>& vertsIndex,
		const unique_ptr<Vec3f[]>& verts,
		unique_ptr<Vec3f[]>& normals,
		unique_ptr<Vec2f[]>& st,
		const Matrix44f& objToWorld
	)
		: Object(objToWorld)
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
			objToWorld.multVecMatrix(verts[i], P[i]);

		trisIndex = unique_ptr<int[]>(new int[numTris * 3LL]);
		N = unique_ptr<Vec3f[]>(new Vec3f[numTris * 3]);
		texCoordinates = unique_ptr<Vec2f[]>(new Vec2f[numTris * 3]);
		Matrix44f transformNormals = worldToObj.transpose();
		int l = 0;

		for (int i = 0, k = 0; i < nfaces; ++i)
		{
			for (int j = 0; j < faceIndex[i] - 2; ++j)
			{
				trisIndex[l] = vertsIndex[k];
				trisIndex[l + 1LL] = vertsIndex[(long long)k + j + 1LL];
				trisIndex[l + 2LL] = vertsIndex[(long long)k + j + 2LL];
				transformNormals.multDirMatrix(normals[k], N[l]);
				transformNormals.multDirMatrix(normals[(long long)k + j + 1LL], N[l + 1LL]);
				transformNormals.multDirMatrix(normals[(long long)k + j + 2LL], N[l + 2LL]);
				N[l].normalize();
				N[l + 1LL].normalize();
				N[l + 2LL].normalize();
				texCoordinates[l] = st[k];
				texCoordinates[l + 1LL] = st[(long long)k + j + 1LL];
				texCoordinates[l + 2LL] = st[(long long)k + j + 2LL];
				l += 3;
			}
			k += faceIndex[i];
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
		if (smoothShading)
		{
			const Vec3f& n0 = N[triIndex * 3LL];
			const Vec3f& n1 = N[triIndex * 3LL + 1LL];
			const Vec3f& n2 = N[triIndex * 3LL + 2LL];
			hitNormal = (1 - u - v) * n0 + u * n1 + v * n2;
		}
		else
		{
			const Vec3f& v0 = P[trisIndex[triIndex * 3LL]];
			const Vec3f& v1 = P[trisIndex[triIndex * 3LL + 1LL]];
			const Vec3f& v2 = P[trisIndex[triIndex * 3LL + 2LL]];
			hitNormal = (v1 - v0).crossProduct(v2 - v0);
		}
		hitNormal.normalize();

		const Vec2f& st0 = texCoordinates[triIndex * 3LL];
		const Vec2f& st1 = texCoordinates[triIndex * 3LL + 1LL];
		const Vec2f& st2 = texCoordinates[triIndex * 3LL + 2LL];
		hitTextureCoordinates = (1 - u - v) * st0 + u * st1 + v * st2;
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


class IntersectionObjInfo
{
public:
	const Object* hitObj = nullptr;
	float t_min = INFINITY;
	float u, v;
	int index = 0;
};


TriangleMesh* loadPolyMeshFromFile(const char* file, const Matrix44f& objToWorld)
{
	ifstream ifs;
	try
	{
		ifs.open(file);
		if (ifs.fail()) throw;

		stringstream ss;
		ss << ifs.rdbuf();
		int numFaces;
		ss >> numFaces;
		unique_ptr<int[]> faceIndex(new int[numFaces]);
		int vertsIndexArraySize = 0;

		for (int i = 0; i < numFaces; ++i)
		{
			ss >> faceIndex[i];
			vertsIndexArraySize += faceIndex[i];
		}

		unique_ptr<int[]> vertsIndex(new int[vertsIndexArraySize]);
		int vertsArraySize = 0;

		for (int i = 0; i < vertsIndexArraySize; ++i)
		{
			ss >> vertsIndex[i];
			if (vertsIndex[i] > vertsArraySize) vertsArraySize = vertsIndex[i];
		}
		vertsArraySize += 1;

		unique_ptr<Vec3f[]> verts(new Vec3f[vertsArraySize]);
		for (int i = 0; i < vertsArraySize; ++i)
		{
			ss >> verts[i].x >> verts[i].y >> verts[i].z;
		}

		unique_ptr<Vec3f[]> normals(new Vec3f[vertsIndexArraySize]);
		for (int i = 0; i < vertsIndexArraySize; ++i)
		{
			ss >> normals[i].x >> normals[i].y >> normals[i].z;
		}

		unique_ptr<Vec2f[]> st(new Vec2f[vertsIndexArraySize]);
		for (int i = 0; i < vertsIndexArraySize; ++i)
		{
			ss >> st[i].x >> st[i].y;
		}

		return new TriangleMesh(numFaces, faceIndex, vertsIndex, verts, normals, st, objToWorld);
	}

	catch (...)
	{
		ifs.close();
	}
	ifs.close();

	return nullptr;
}


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


bool trace
(
	const Ray& ray,
	const vector<unique_ptr<Object>>& objects,
	IntersectionObjInfo& hitObjInfo,
	RayType rayType = RayType::PRIMARY_RAY
)
{
	hitObjInfo.hitObj = nullptr;
	vector<unique_ptr<Object>>::const_iterator iter;
	float t_bb;
	
	for (iter = objects.begin(); iter != objects.end(); ++iter)
	{
		AABBox* bbPtr = (*iter)->bbox;
		if ((bbPtr != nullptr) && !(bbPtr->intersect(ray, t_bb))) continue;

		float t = INFINITY, u, v;
		int index = 0;
		bool isIntersect;

		isIntersect = (*iter)->intersect(ray, t, index, u, v);

		if (isIntersect && (t < hitObjInfo.t_min))
		{
			if (rayType == RayType::SHADOW_RAY && (*iter)->type == MaterialType::REFLECTION_REFRACTION) continue;
			hitObjInfo.hitObj = (iter)->get();
			hitObjInfo.t_min = t;
			hitObjInfo.index = index;
			hitObjInfo.u = u;
			hitObjInfo.v = v;
		}
	}
	return (hitObjInfo.hitObj != nullptr);
}


Vec3f reflect(const Vec3f& I, const Vec3f& N)
{
	return I - 2 * I.dotProduct(N) * N;
}


Vec3f refract(const Vec3f& I, const Vec3f& N, const float& refractIndex)
{
	float cos_src = clamp(-1, 1, I.dotProduct(N));
	float refrIndex_src = 1, refrIndex_dst = refractIndex;
	Vec3f n = N;
	if (cos_src < 0) cos_src = -cos_src;
	else
	{
		swap(refrIndex_src, refrIndex_dst);
		n = -N;
	}
	float refrIndex = refrIndex_src / refrIndex_dst;
	float k = 1 - refrIndex * refrIndex * (1 - cos_src * cos_src);
	return k < 0 ? 0 : refrIndex * I + (refrIndex * cos_src - sqrtf(k)) * n;
}


void fresnel(const Vec3f& I, const Vec3f& N, const float& refractIndex, float& fresnelReflect)
{
	float cos_src = clamp(-1, 1, I.dotProduct(N));
	float refrIndex_src = 1, refrIndex_dst = refractIndex;
	
	if (cos_src > 0) swap(refrIndex_src, refrIndex_dst);
	float sin_dst = refrIndex_src / refrIndex_dst * sqrtf(max(0.f, 1 - cos_src * cos_src));

	if (sin_dst >= 1) fresnelReflect = 1;
	else
	{
		cos_src = fabsf(cos_src);
		float cos_dst = sqrtf(max(0.f, 1 - sin_dst * sin_dst));
		float R_1 = (refrIndex_dst * cos_src - refrIndex_src * cos_dst) / (refrIndex_dst * cos_src + refrIndex_src * cos_dst);
		float R_2 = (refrIndex_src * cos_dst - refrIndex_dst * cos_src) / (refrIndex_src * cos_dst + refrIndex_dst * cos_src);
		fresnelReflect = (R_1 * R_1 + R_2 * R_2) / 2;
	}
}


Vec3f castRay
(
	const Ray& ray, 
	const vector<unique_ptr<Object>>& objects, 
	const vector<unique_ptr<Light>>& lights, 
	const Parameters& param, 
	const int depth = 0
)
{
	if (depth > param.maxDepth) return param.BGColor;
	Vec3f hitColor(0);
	IntersectionObjInfo hitObjInfo;

	if (trace(ray, objects, hitObjInfo))
	{
		Vec3f hitPoint = ray.orig + hitObjInfo.t_min * ray.dir;
		Vec3f hitNormal;
		Vec2f hitTexCoordinates;
		hitObjInfo.hitObj->getSurfaceProperties(hitPoint, ray.dir, hitObjInfo.index, hitObjInfo.u, hitObjInfo.v, hitNormal, hitTexCoordinates);

		switch (hitObjInfo.hitObj->type)
		{
		case MaterialType::DIFFUSE:
		{
			for (int i = 0; i < lights.size(); ++i)
			{
				Vec3f lightDir, lightIntens;
				IntersectionObjInfo hitObjShadowInfo;
				lights[i]->illuminate(hitPoint, lightDir, lightIntens, hitObjShadowInfo.t_min);

				Ray shadowRay(hitPoint + hitNormal * param.bias, -lightDir);
				bool vis = !trace(shadowRay, objects, hitObjShadowInfo, RayType::SHADOW_RAY);
				
				float angle = 45 * M_PI / 180;
				float s = hitTexCoordinates.x * cos(angle) - hitTexCoordinates.y * sin(angle);
				float t = hitTexCoordinates.y * cos(angle) + hitTexCoordinates.x * sin(angle);
				float scaleS = 20.f, scaleT = 20.f;
				float pattern = (s * scaleS - (float)floor(s * scaleS) < 0.5f);

				hitColor += vis * pattern * /*hitObjInfo.hitObj->albedo * */ lightIntens * max(0.f, hitNormal.dotProduct(-lightDir));
			}
			break;
		}
		case MaterialType::REFLECTION:
		{
			Vec3f R = reflect(ray.dir, hitNormal);
			R.normalize();
			Ray reflectRay(hitPoint + hitNormal * param.bias, R);
			hitColor += 0.8f * castRay(reflectRay, objects, lights, param, depth + 1);
			break;
		}
		case MaterialType::REFLECTION_REFRACTION:
		{
			Vec3f refractionColor = 0, reflectionColor = 0;
			float fresnelReflect;
			fresnel(ray.dir, hitNormal, hitObjInfo.hitObj->RefractIndex, fresnelReflect);
			bool outside = ray.dir.dotProduct(hitNormal) < 0;
			Vec3f bias = param.bias * hitNormal;

			if (fresnelReflect < 1)
			{
				Vec3f refractionRayOrig = outside ? hitPoint - bias : hitPoint + bias;
				Vec3f refractionDirection = refract(ray.dir, hitNormal, hitObjInfo.hitObj->RefractIndex).normalize();
				Ray refractRay(refractionRayOrig, refractionDirection);
				refractionColor = castRay(refractRay, objects, lights, param, depth + 1);
			}

			Vec3f reflectionRayOrig = outside ? hitPoint + bias : hitPoint - bias;
			Vec3f reflectionDirection = reflect(ray.dir, hitNormal).normalize();
			Ray reflectRay(reflectionRayOrig, reflectionDirection);
			reflectionColor = castRay(reflectRay, objects, lights, param, depth + 1);

			hitColor += reflectionColor * fresnelReflect + refractionColor * (1 - fresnelReflect);
			break;
		}
		default:
			break;
		}
	}
	else
	{
		hitColor = param.BGColor;
	}

	return hitColor;
}


void render
(
	const Parameters& parameters, 
	const vector<unique_ptr<Object>>& objects, 
	const vector<unique_ptr<Light>>& lights
)
{
	Vec3f* frameBuffer = new Vec3f[parameters.width * parameters.height];
	Vec3f* pixel_curr = frameBuffer;

	float imageAspectRatio = parameters.width / (float)parameters.height;
	float scale = (float)tan(parameters.FOV * 0.5f * (float)M_PI / 180);


	auto timeStart = chrono::high_resolution_clock::now();
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

			Ray ray(origin, direction);
			*(pixel_curr++) = castRay(ray, objects, lights, parameters);
		}
		fprintf(stderr, "\r%3d%c", int(j / (float)parameters.height * 100), '%');
	}
	auto timeEnd = chrono::high_resolution_clock::now();
	auto passedTime = chrono::duration<double, milli>(timeEnd - timeStart).count();
	fprintf(stderr, "\rDone: %.2f (sec)\n", passedTime / 1000);


	ofstream ofs("ray_tracing_foxtrot_out.ppm", ios::out | ios::binary);
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
	Vec3f cam_origin(0, 0, 0), cam_target(0, 0, -1);
	camToWorld = LookAt(cam_origin, cam_target);

	Parameters parameters(800, 800, 60, camToWorld, cam_origin, cam_target);
	parameters.camToWorld = Matrix44f(0.999945, 0, 0.0104718, 0, 0.00104703, 0.994989, -0.0999803, 0, -0.0104193, 0.0999858, 0.994934, 0, -0.978596, 12.911879, 50.483369, 1);
	Vec3f vmin, vmax;

	vector<unique_ptr<Object>> objects;

	TriangleMesh* mesh_1 = loadPolyMeshFromFile("data/glasses.geo", Matrix44f());
	mesh_1->type = MaterialType::REFLECTION_REFRACTION;
	mesh_1->RefractIndex = 1.3;
	mesh_1->vMinMax(vmin, vmax);
	mesh_1->bbox = new AABBox(vmin, vmax);
	objects.push_back(std::unique_ptr<Object>(mesh_1));

	TriangleMesh* mesh_2 = loadPolyMeshFromFile("data/backdrop1.geo", Matrix44f());
	mesh_2->type = MaterialType::DIFFUSE;
	mesh_2->albedo = 0.18;
	mesh_2->vMinMax(vmin, vmax);
	mesh_2->bbox = new AABBox(vmin, vmax);
	objects.push_back(std::unique_ptr<Object>(mesh_2));


	Matrix44f xform;
	xform[0][0] = 10;
	xform[1][1] = 10;
	xform[2][2] = 10;
	xform[3][2] = -40;
	TriangleMesh* mesh = loadPolyMeshFromFile("data/plane.geo", xform);
	mesh->type = MaterialType::DIFFUSE;
	mesh->albedo = Vec3f(0, 0.4, 0);
	mesh->smoothShading = false;
	//objects.push_back(unique_ptr<Object>(mesh));
	

	vector<unique_ptr<Light>> lights;
	//Matrix44f l2w = Matrix44f();
	Matrix44f l2w(0.95292, 0.289503, 0.0901785, 0, -0.0960954, 0.5704, -0.815727, 0, -0.287593, 0.768656, 0.571365, 0, 0, 0, 0, 1);
	lights.push_back(unique_ptr<Light>(new DistantLight(l2w, 1, 1)));
	
	render(parameters, objects, lights);
}


#endif

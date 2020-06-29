#define TERRAIN_RENDER_
#ifdef TERRAIN_RENDER_1

#include "terrain_header.h"

using namespace std;


const float eps = 1e-8f;

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

#define CULLING_1
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

	return true;
}


Matrix44f LookAt(const Vec3f& origin, const Vec3f& target, const Vec3f& up_temp)
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
	float& t_min, float& u_final, float& v_final,
	int& hitTrianIndex, const Object*& hitObj
)
{
	vector<unique_ptr<Object>>::const_iterator iter;
	float t_bb;

	for (iter = objects.begin(); iter != objects.end(); ++iter)
	{
		AABBox* bbPtr = (*iter)->bbox;
		if ((bbPtr != nullptr) && !(bbPtr->intersect(ray, t_bb))) continue;

		float t = INFINITY, u, v;
		int TrianID;
		bool isIntersect;

		isIntersect = (*iter)->intersect(ray, t, TrianID, u, v);

		if (isIntersect && (t < t_min))
		{
			hitObj = (iter)->get();
			t_min = t;
			hitTrianIndex = TrianID;
			u_final = u;
			v_final = v;
		}
	}

	return (hitObj != nullptr);
}



Vec3f castRay(const Ray& ray, const vector<unique_ptr<Object>>& objects)
{
	Vec3f hitColor(0.2f, 0.7f, 0.8f);
	const Object* hitObj = nullptr;
	int hitTrianIndex = 0;
	float t = INFINITY, u, v;

	if (trace(ray, objects, t, u, v, hitTrianIndex, hitObj))
	{
		Vec3f hitPoint = ray.orig + t * ray.dir;
		Vec3f hitNormal;
		Vec2f hitTexCoordinates;
		hitObj->getSurfaceProperties(hitPoint, ray.dir, hitTrianIndex, u, v, hitNormal, hitTexCoordinates);

		float NdotDir = max(0.f, hitNormal.dotProduct(-ray.dir));
		hitColor = NdotDir;
	}

	return hitColor;
}


void render(const Parameters& parameters, const vector<unique_ptr<Object>>& objects, const string& FileName)
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
			*(pixel_curr++) = castRay(ray, objects);
		}
		fprintf(stderr, "\r%3d%c", int(j / (float)parameters.height * 100), '%');
	}
	auto timeEnd = chrono::high_resolution_clock::now();
	auto passedTime = chrono::duration<double, milli>(timeEnd - timeStart).count();
	fprintf(stderr, "\rDone: %.2f (sec)\n", passedTime / 1000);


	ofstream ofs(FileName, ios::out | ios::binary);
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




#endif

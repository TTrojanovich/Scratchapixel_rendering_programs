#define RASTERIZATION_
#ifdef RASTERIZATION_1
#define _USE_MATH_DEFINES

#include <fstream>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "../headers/geometry.h"
#include "../data/cow.h"

using namespace std;


static const float inchToMm = 25.4f;
enum class FitResolutionGate { kFill, kOverscan };

float min3(const float& a, const float& b, const float& c)
{
	return std::min(a, std::min(b, c));
}

float max3(const float& a, const float& b, const float& c)
{
	return std::max(a, std::max(b, c));
}

float edgeFunction(const Vec3f& a, const Vec3f& b, const Vec3f& c)
{
	return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
}

class Parameters
{
public:
	int width;
	int height;
	int WidMultHeig;
	Matrix44f worldToCamera;
	Matrix44f camToWorld;
	Vec3f cam_origin;
	Vec3f cam_direction;
	int ntris; // = 3156;
	const float nearClippingPLane = 1;
	const float farClippingPLane = 1000;
	float focalLength = 40;
	float filmApertureWidth = 0.980f;
	float filmApertureHeight = 0.735f;
	unique_ptr<Vec3f[]> vertices;
	unique_ptr<Vec2f[]> st;
	unique_ptr<int[]> nvertices;
	float l, r, b, t;

	Parameters(int width, int height, Matrix44f worldToCamera, Matrix44f camToWorld, Vec3f cam_origin, Vec3f cam_direction)
	{
		this->width = width;
		this->height = height;
		this->WidMultHeig = width * height;
		this->worldToCamera = worldToCamera;
		this->camToWorld = camToWorld;
		this->cam_origin = cam_origin;
		this->cam_direction = cam_direction;
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


void loadGeoFile
(
	const char* file,
	int& numFaces,
	unique_ptr<Vec3f[]>& verts,
	unique_ptr<Vec2f[]>& st,
	unique_ptr<int[]>& vertsIndex
)
{
	ifstream ifs;

	try 
	{
		ifs.open(file);
		if (ifs.fail()) throw;

		stringstream ss;
		ss << ifs.rdbuf();
		ss >> numFaces;
		int vertsIndexArraySize = 0;
		
		for (int i = 0; i < numFaces; ++i) 
		{
			int tmp;
			ss >> tmp; //faceIndex[i];
			vertsIndexArraySize += tmp;
		}

		vertsIndex = unique_ptr<int[]>(new int[vertsIndexArraySize]);
		
		int vertsArraySize = 0;
		for (int i = 0; i < vertsIndexArraySize; ++i) 
		{
			ss >> vertsIndex[i];
			if (vertsIndex[i] > vertsArraySize) vertsArraySize = vertsIndex[i];
		}
		vertsArraySize += 1;
		
		verts = unique_ptr<Vec3f[]>(new Vec3f[vertsArraySize]);
		for (int i = 0; i < vertsArraySize; ++i)
		{
			ss >> verts[i].x >> verts[i].y >> verts[i].z;
		}
		
		for (int i = 0; i < vertsIndexArraySize; ++i)
		{
			Vec3f normal;
			ss >> normal.x >> normal.y >> normal.z;
		}
		
		st = unique_ptr<Vec2f[]>(new Vec2f[vertsIndexArraySize]);
		for (int i = 0; i < vertsIndexArraySize; ++i)
		{
			ss >> st[i].x >> st[i].y;
		}
	}

	catch (...)
	{
		ifs.close();
	}
	
	ifs.close();
}


void computeScreenCoordinates
(
	const float& filmApertureWidth,
	const float& filmApertureHeight,
	const int& imageWidth,
	const int& imageHeight,
	const FitResolutionGate& fitFilm,
	const float& nearClippingPLane,
	const float& focalLength,
	float& top, float& bottom, float& left, float& right
)
{
	float filmAspectRatio = filmApertureWidth / filmApertureHeight;
	float deviceAspectRatio = imageWidth / (float)imageHeight;

	top = ((filmApertureHeight * inchToMm / 2) / focalLength) * nearClippingPLane;
	right = ((filmApertureWidth * inchToMm / 2) / focalLength) * nearClippingPLane;

	float xscale = 1;
	float yscale = 1;

	switch (fitFilm) 
	{
	default:
	case FitResolutionGate::kFill:
		if (filmAspectRatio > deviceAspectRatio) 
		{
			xscale = deviceAspectRatio / filmAspectRatio;
		}
		else {
			yscale = filmAspectRatio / deviceAspectRatio;
		}
		break;
	case FitResolutionGate::kOverscan:
		if (filmAspectRatio > deviceAspectRatio) 
		{
			yscale = filmAspectRatio / deviceAspectRatio;
		}
		else 
		{
			xscale = deviceAspectRatio / filmAspectRatio;
		}
		break;
	}

	right *= xscale;
	top *= yscale;

	bottom = -top;
	left = -right;
}


void convertToRaster
(
	const Vec3f& vertexWorld,
	const Matrix44f& worldToCamera,
	const float& l,
	const float& r,
	const float& t,
	const float& b,
	const float& near,
	const int& imageWidth,
	const int& imageHeight,
	Vec3f& vertexRaster
)
{
	Vec3f vertexCamera;
	
	worldToCamera.multVecMatrix(vertexWorld, vertexCamera);

	Vec2f vertexScreen;
	vertexScreen.x = near * vertexCamera.x / -vertexCamera.z;
	vertexScreen.y = near * vertexCamera.y / -vertexCamera.z;

	Vec2f vertexNDC;
	vertexNDC.x = 2 * vertexScreen.x / (r - l) - (r + l) / (r - l);
	vertexNDC.y = 2 * vertexScreen.y / (t - b) - (t + b) / (t - b);

	vertexRaster.x = (vertexNDC.x + 1) / 2 * imageWidth;
	vertexRaster.y = (1 - vertexNDC.y) / 2 * imageHeight;
	vertexRaster.z = -vertexCamera.z;
}


void render(const Parameters& param)
{
	Vec3<unsigned char>* frameBuffer = new Vec3<unsigned char>[param.WidMultHeig];
	for (int i = 0; i < param.WidMultHeig; ++i)
		frameBuffer[i] = Vec3<unsigned char>(0);

	float* depthBuffer = new float[param.WidMultHeig];
	for (int i = 0; i < param.WidMultHeig; ++i) depthBuffer[i] = param.farClippingPLane;


	for (int i = 0; i < param.ntris; ++i)
	{
		const Vec3f& v0 = vertices[nvertices[i * 3]];
		const Vec3f& v1 = vertices[nvertices[i * 3 + 1]];
		const Vec3f& v2 = vertices[nvertices[i * 3 + 2]];

		Vec3f v0Raster, v1Raster, v2Raster;
		convertToRaster(v0, param.worldToCamera, param.l, param.r, param.t, param.b, param.nearClippingPLane, param.width, param.height, v0Raster);
		convertToRaster(v1, param.worldToCamera, param.l, param.r, param.t, param.b, param.nearClippingPLane, param.width, param.height, v1Raster);
		convertToRaster(v2, param.worldToCamera, param.l, param.r, param.t, param.b, param.nearClippingPLane, param.width, param.height, v2Raster);

		v0Raster.z = 1 / v0Raster.z,
		v1Raster.z = 1 / v1Raster.z,
		v2Raster.z = 1 / v2Raster.z;

		Vec2f st0 = st[stindices[i * 3]];
		Vec2f st1 = st[stindices[i * 3 + 1]];
		Vec2f st2 = st[stindices[i * 3 + 2]];

		st0 *= v0Raster.z, st1 *= v1Raster.z, st2 *= v2Raster.z;

		float xmin = min3(v0Raster.x, v1Raster.x, v2Raster.x);
		float ymin = min3(v0Raster.y, v1Raster.y, v2Raster.y);
		float xmax = max3(v0Raster.x, v1Raster.x, v2Raster.x);
		float ymax = max3(v0Raster.y, v1Raster.y, v2Raster.y);

		if (xmin > param.width - 1 || xmax < 0 || ymin > param.height - 1 || ymax < 0) continue;

		int x0 = max(int(0), (int)(floor(xmin)));
		int x1 = min(int(param.width) - 1, (int)(floor(xmax)));
		int y0 = max(int(0), (int)(floor(ymin)));
		int y1 = min(int(param.height) - 1, (int)(floor(ymax)));

		float area = edgeFunction(v0Raster, v1Raster, v2Raster);


		for (int y = y0; y <= y1; ++y)
		{
			for (int x = x0; x <= x1; ++x)
			{
				Vec3f pixelSample_1(x + 0.25f, y + 0.25f, 0.f);
				Vec3f pixelSample_2(x + 0.75f, y + 0.25f, 0.f);
				Vec3f pixelSample_3(x + 0.25f, y + 0.75f, 0.f);
				Vec3f pixelSample_4(x + 0.75f, y + 0.75f, 0.f);

				float w0_1 = edgeFunction(v1Raster, v2Raster, pixelSample_1);
				float w0_2 = edgeFunction(v1Raster, v2Raster, pixelSample_2);
				float w0_3 = edgeFunction(v1Raster, v2Raster, pixelSample_3);
				float w0_4 = edgeFunction(v1Raster, v2Raster, pixelSample_4);

				float w1_1 = edgeFunction(v2Raster, v0Raster, pixelSample_1);
				float w1_2 = edgeFunction(v2Raster, v0Raster, pixelSample_2);
				float w1_3 = edgeFunction(v2Raster, v0Raster, pixelSample_3);
				float w1_4 = edgeFunction(v2Raster, v0Raster, pixelSample_4);

				float w2_1 = edgeFunction(v0Raster, v1Raster, pixelSample_1);
				float w2_2 = edgeFunction(v0Raster, v1Raster, pixelSample_2);
				float w2_3 = edgeFunction(v0Raster, v1Raster, pixelSample_3);
				float w2_4 = edgeFunction(v0Raster, v1Raster, pixelSample_4);

				Vec3f pixelSample(x + 0.5f, y + 0.5f, 0.f);
				float w0 = edgeFunction(v1Raster, v2Raster, pixelSample);
				float w1 = edgeFunction(v2Raster, v0Raster, pixelSample);
				float w2 = edgeFunction(v0Raster, v1Raster, pixelSample);


				if (w0 >= 0 && w1 >= 0 && w2 >= 0)
				{
					w0 /= area;
					w1 /= area;
					w2 /= area;
					float z = 1 / (v0Raster.z * w0 + v1Raster.z * w1 + v2Raster.z * w2);

					if (z < depthBuffer[y * param.width + x])
					{
						depthBuffer[y * param.width + x] = z;

						float nDotView_1 = -1, nDotView_2 = -1, nDotView_3 = -1, nDotView_4 = -1;

						if (w0_1 >= 0 && w1_1 >= 0 && w2_1 >= 0) { w0_1 /= area; w1_1 /= area; w2_1 /= area; }
						else { nDotView_1 = 1; }
						if (w0_2 >= 0 && w1_2 >= 0 && w2_2 >= 0) { w0_2 /= area; w1_2 /= area; w2_2 /= area; }
						else { nDotView_2 = 1; }
						if (w0_3 >= 0 && w1_3 >= 0 && w2_3 >= 0) { w0_3 /= area; w1_3 /= area; w2_3 /= area; }
						else { nDotView_3 = 1; }
						if (w0_4 >= 0 && w1_4 >= 0 && w2_4 >= 0) { w0_4 /= area; w1_4 /= area; w2_4 /= area; }
						else { nDotView_4 = 1; }

						Vec2f st_1 = st0 * w0_1 + st1 * w1_1 + st2 * w2_1;
						Vec2f st_2 = st0 * w0_2 + st1 * w1_2 + st2 * w2_2;
						Vec2f st_3 = st0 * w0_3 + st1 * w1_3 + st2 * w2_3;
						Vec2f st_4 = st0 * w0_4 + st1 * w1_4 + st2 * w2_4;

						float z_1 = 1 / (v0Raster.z * w0_1 + v1Raster.z * w1_1 + v2Raster.z * w2_1);
						float z_2 = 1 / (v0Raster.z * w0_2 + v1Raster.z * w1_2 + v2Raster.z * w2_2);
						float z_3 = 1 / (v0Raster.z * w0_3 + v1Raster.z * w1_3 + v2Raster.z * w2_3);
						float z_4 = 1 / (v0Raster.z * w0_4 + v1Raster.z * w1_4 + v2Raster.z * w2_4);

						st_1 *= z_1; st_2 *= z_2; st_3 *= z_3; st_4 *= z_4;

						const float M = 20.0f;
						float checker_1 = (float)((fmod(st_1.x * M, 1.0f) > 0.5f) ^ (fmod(st_1.y * M, 1.0f) < 0.5f));
						float checker_2 = (float)((fmod(st_2.x * M, 1.0f) > 0.5f) ^ (fmod(st_2.y * M, 1.0f) < 0.5f));
						float checker_3 = (float)((fmod(st_3.x * M, 1.0f) > 0.5f) ^ (fmod(st_3.y * M, 1.0f) < 0.5f));
						float checker_4 = (float)((fmod(st_4.x * M, 1.0f) > 0.5f) ^ (fmod(st_4.y * M, 1.0f) < 0.5f));

						float c_1 = 0.3f * (1.f - checker_1) + 0.7f * checker_1;
						float c_2 = 0.3f * (1.f - checker_2) + 0.7f * checker_2;
						float c_3 = 0.3f * (1.f - checker_3) + 0.7f * checker_3;
						float c_4 = 0.3f * (1.f - checker_4) + 0.7f * checker_4;

						if (nDotView_1 == -1) nDotView_1 = c_1;
						if (nDotView_2 == -1) nDotView_2 = c_2;
						if (nDotView_3 == -1) nDotView_3 = c_3;
						if (nDotView_4 == -1) nDotView_4 = c_4;

						float nDotView = (nDotView_1 + nDotView_2 + nDotView_3 + nDotView_4) / 4;

						frameBuffer[y * param.width + x].x = (unsigned char)(nDotView * 255.0f);
						frameBuffer[y * param.width + x].y = (unsigned char)(nDotView * 255.0f);
						frameBuffer[y * param.width + x].z = (unsigned char)(nDotView * 255.0f);
					}
				}

			}
		}
	}

	std::ofstream ofs;
	ofs.open("rasterization_out.ppm");
	ofs << "P6\n" << param.width << " " << param.height << "\n255\n";
	ofs.write((char*)frameBuffer, param.WidMultHeig * 3.0f);
	ofs.close();

	delete[] frameBuffer;
	delete[] depthBuffer;
}


int main(int argc, char** argv)
{
	Matrix44f camToWorld;
	Vec3f cam_origin(0, 0, 0), cam_target(0, 0, -1);
	Vec3f cam_origin_world(20, 15, 25), cam_target_world(0, 5, -1);
	camToWorld = LookAt(cam_origin_world, cam_target_world);
	Matrix44f worldToCamera = camToWorld.inverse();

	Parameters param(800, 800, worldToCamera, camToWorld, cam_origin, cam_target);

	loadGeoFile("data/cow.geo", param.ntris, param.vertices, param.st, param.nvertices);

	float t, b, l, r;
	computeScreenCoordinates
	(
		param.filmApertureWidth, param.filmApertureHeight,
		param.width, param.height,
		FitResolutionGate::kOverscan,
		param.nearClippingPLane,
		param.focalLength,
		param.t, param.b, param.l, param.r
	);

	render(param);

	return 0;
}


#endif

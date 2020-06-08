#define RASTERIZATION_
#ifdef RASTERIZATION_1

#define _USE_MATH_DEFINES
#include "geometry.h"
#include <fstream>
#include <chrono>
#include <algorithm>
#include <cmath>

#include "cow.h"

static const float inchToMm = 25.4f;
enum class FitResolutionGate { kFill = 0, kOverscan };



void computeScreenCoordinates
(
	const float& filmApertureWidth,
	const float& filmApertureHeight,
	const uint32_t& imageWidth,
	const uint32_t& imageHeight,
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

	// field of view (horizontal)
	float fov = 2 * 180 / (float)M_PI * atan((filmApertureWidth * inchToMm / 2) / focalLength);
	std::cerr << "Field of view " << fov << std::endl;

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
	const uint32_t& imageWidth,
	const uint32_t& imageHeight,
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

float min3(const float &a, const float &b, const float &c)
{ return std::min(a, std::min(b, c)); }

float max3(const float &a, const float &b, const float &c)
{ return std::max(a, std::max(b, c)); }

float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{ return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]); }

const uint32_t imageWidth = 640;
const uint32_t imageHeight = 480;
const Matrix44f worldToCamera = {0.707107f, -0.331295f, 0.624695f, 0.f, 0.f, 0.883452f, 0.468521f, 0.f, -0.707107f, -0.331295f, 0.624695f, 0.f, -1.63871f, -5.747777f, -40.400412f, 1.f};

const uint32_t ntris = 3156;
const float nearClippingPLane = 1;
const float farClippingPLane = 1000;
float focalLength = 40; // in mm
// 35mm Full Aperture in inches
float filmApertureWidth = 0.980f;
float filmApertureHeight = 0.735f;




int main(int argc, char** argv)
{
	Matrix44f cameraToWorld = worldToCamera.inverse();

	float t, b, l, r;

	computeScreenCoordinates
	(
		filmApertureWidth, filmApertureHeight,
		imageWidth, imageHeight,
		FitResolutionGate::kOverscan,
		nearClippingPLane,
		focalLength,
		t, b, l, r
	);


	Vec3<unsigned char>* frameBuffer = new Vec3<unsigned char>[imageWidth * imageHeight];
	for (uint32_t i = 0; i < imageWidth * imageHeight; ++i)
		frameBuffer[i] = Vec3<unsigned char>(0);

	float* depthBuffer = new float[imageWidth * imageHeight];
	for (uint32_t i = 0; i < imageWidth * imageHeight; ++i)
		depthBuffer[i] = farClippingPLane;

	auto t_start = std::chrono::high_resolution_clock::now();

	for (uint32_t i = 0; i < ntris; ++i)
	{
		const Vec3f& v0 = vertices[nvertices[i * 3]];
		const Vec3f& v1 = vertices[nvertices[i * 3 + 1]];
		const Vec3f& v2 = vertices[nvertices[i * 3 + 2]];

		Vec3f v0Raster, v1Raster, v2Raster;
		convertToRaster(v0, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v0Raster);
		convertToRaster(v1, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v1Raster);
		convertToRaster(v2, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v2Raster);

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

		// the triangle is out of screen
		if (xmin > imageWidth - 1 || xmax < 0 || ymin > imageHeight - 1 || ymax < 0) continue;

		uint32_t x0 = std::max(int32_t(0), (int32_t)(std::floor(xmin)));
		uint32_t x1 = std::min(int32_t(imageWidth) - 1, (int32_t)(std::floor(xmax)));
		uint32_t y0 = std::max(int32_t(0), (int32_t)(std::floor(ymin)));
		uint32_t y1 = std::min(int32_t(imageHeight) - 1, (int32_t)(std::floor(ymax)));

		float area = edgeFunction(v0Raster, v1Raster, v2Raster);


		for (uint32_t y = y0; y <= y1; ++y)
		{
			for (uint32_t x = x0; x <= x1; ++x)
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

					if (z < depthBuffer[y * imageWidth + x])
					{
						depthBuffer[y * imageWidth + x] = z;

						float nDotView_1 = -1, nDotView_2 = -1, nDotView_3 = -1, nDotView_4 = -1;

						if (w0_1 >= 0 && w1_1 >= 0 && w2_1 >= 0) { w0_1 /= area; w1_1 /= area; w2_1 /= area; }
						else { nDotView_1 = 1;  }
						if (w0_2 >= 0 && w1_2 >= 0 && w2_2 >= 0) { w0_2 /= area; w1_2 /= area; w2_2 /= area; }
						else { nDotView_2 = 1;  }
						if (w0_3 >= 0 && w1_3 >= 0 && w2_3 >= 0) { w0_3 /= area; w1_3 /= area; w2_3 /= area; }
						else { nDotView_3 = 1;  }
						if (w0_4 >= 0 && w1_4 >= 0 && w2_4 >= 0) { w0_4 /= area; w1_4 /= area; w2_4 /= area; }
						else { nDotView_4 = 1;  }

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

						//if (nDotView_1 + nDotView_2 + nDotView_3 + nDotView_4 == -4)
						{
							if (nDotView_1 == -1) nDotView_1 = c_1;
							if (nDotView_2 == -1) nDotView_2 = c_2;
							if (nDotView_3 == -1) nDotView_3 = c_3;
							if (nDotView_4 == -1) nDotView_4 = c_4;

							float nDotView = (nDotView_1 + nDotView_2 + nDotView_3 + nDotView_4) / 4;

							frameBuffer[y * imageWidth + x].x = (unsigned char)(nDotView * 255.0f);
							frameBuffer[y * imageWidth + x].y = (unsigned char)(nDotView * 255.0f);
							frameBuffer[y * imageWidth + x].z = (unsigned char)(nDotView * 255.0f);
						}
					}
				}
				
			}
		}
	}


	auto t_end = std::chrono::high_resolution_clock::now();
	auto passedTime = std::chrono::duration<double, std::milli>(t_end - t_start).count();
	std::cerr << "Wall passed time:  " << passedTime << " ms" << std::endl;

	std::ofstream ofs;
	ofs.open("rasterization_out.ppm");
	ofs << "P6\n" << imageWidth << " " << imageHeight << "\n255\n";
	ofs.write((char*)frameBuffer, imageWidth * imageWidth * 3);
	ofs.close();

	delete[] frameBuffer;
	delete[] depthBuffer;

	return 0;
}


#endif

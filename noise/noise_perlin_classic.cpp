#define NOISE_PERLIN_CLASSIC_
#ifdef NOISE_PERLIN_CLASSIC_1
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdio>
#include <random>
#include <functional>
#include <iostream>
#include <fstream>
#include "../headers/geometry.h"

using namespace std;

template<typename T = float>
T lerp(const T& lo, const T& hi, const T& t)
{
	return lo * (1 - t) + hi * t;
}
float smoothstep(const float& t)
{
	return t * t * (3 - 2 * t);
}
float quintic(const float& t)
{
	return t * t * t * (t * (t * 6 - 15) + 10);
}


class PerlinNoise
{
public:
	static const unsigned tableSize = 256;
	static const unsigned tableSizeMask = tableSize - 1;
	Vec3f gradients[tableSize];
	unsigned permutationTable[tableSize * 2] = { 0 };

	PerlinNoise(const unsigned seed = 2016)
	{
		mt19937 generator(seed);
		uniform_real_distribution<float> distribution;
		for (unsigned i = 0; i < tableSize; ++i)
		{
			float theta = acos(2 * distribution(generator) - 1);
			float phi = 2 * distribution(generator) * (float)M_PI;

			float x = cos(phi) * sin(theta);
			float y = sin(phi) * sin(theta);
			float z = cos(theta);
			gradients[i] = Vec3f(x, y, z);
			permutationTable[i] = i;
		}

		uniform_int_distribution<unsigned> distributionInt;
		for (unsigned i = 0; i < tableSize; ++i)
			swap(permutationTable[i], permutationTable[distributionInt(generator) & tableSizeMask]);
		for (unsigned i = 0; i < tableSize; ++i)
			permutationTable[tableSize + i] = permutationTable[i];
	}

	virtual ~PerlinNoise() {}

	float eval(const Vec3f& p) const
	{
		int xi0 = ((int)floor(p.x)) & tableSizeMask;
		int yi0 = ((int)floor(p.y)) & tableSizeMask;
		int zi0 = ((int)floor(p.z)) & tableSizeMask;

		int xi1 = (xi0 + 1) & tableSizeMask;
		int yi1 = (yi0 + 1) & tableSizeMask;
		int zi1 = (zi0 + 1) & tableSizeMask;

		float tx = p.x - ((int)floor(p.x));
		float ty = p.y - ((int)floor(p.y));
		float tz = p.z - ((int)floor(p.z));

		float u = smoothstep(tx);
		float v = smoothstep(ty);
		float w = smoothstep(tz);

		const Vec3f& c000 = gradients[hash(xi0, yi0, zi0)];
		const Vec3f& c100 = gradients[hash(xi1, yi0, zi0)];
		const Vec3f& c010 = gradients[hash(xi0, yi1, zi0)];
		const Vec3f& c110 = gradients[hash(xi1, yi1, zi0)];

		const Vec3f& c001 = gradients[hash(xi0, yi0, zi1)];
		const Vec3f& c101 = gradients[hash(xi1, yi0, zi1)];
		const Vec3f& c011 = gradients[hash(xi0, yi1, zi1)];
		const Vec3f& c111 = gradients[hash(xi1, yi1, zi1)];

		float x0 = tx, x1 = tx - 1;
		float y0 = ty, y1 = ty - 1;
		float z0 = tz, z1 = tz - 1;

		Vec3f p000 = Vec3f(x0, y0, z0);
		Vec3f p100 = Vec3f(x1, y0, z0);
		Vec3f p010 = Vec3f(x0, y1, z0);
		Vec3f p110 = Vec3f(x1, y1, z0);

		Vec3f p001 = Vec3f(x0, y0, z1);
		Vec3f p101 = Vec3f(x1, y0, z1);
		Vec3f p011 = Vec3f(x0, y1, z1);
		Vec3f p111 = Vec3f(x1, y1, z1);

		float a = lerp(c000.dotProduct(p000), c100.dotProduct(p100), u);
		float b = lerp(c010.dotProduct(p010), c110.dotProduct(p110), u);
		float c = lerp(c001.dotProduct(p001), c101.dotProduct(p101), u);
		float d = lerp(c011.dotProduct(p011), c111.dotProduct(p111), u);

		float e = lerp(a, b, v);
		float f = lerp(c, d, v);

		return lerp(e, f, w);
	}

	int hash(const int& x, const int& y, const int& z) const
	{
		return permutationTable[permutationTable[permutationTable[x] + y] + z];
	}


	};


int main()
{
	int imgW = 512, imgH = 512;
	float* noiseMap = new float[imgW * (long long)imgH]{ 0 };
	string FileName = "noise_perlin_classic.ppm";
	PerlinNoise noise;

	for (int j = 0; j < imgH; ++j)
		for (int i = 0; i < imgW; ++i)
			noiseMap[j * imgW + i] = (noise.eval(Vec3f(float(i), 0, float(j)) * (1 / 32.f)) + 1.f) * 0.5f;
	

	ofstream ofs(FileName, ios::out | ios::binary);
	ofs << "P6\n" << imgW << " " << imgH << "\n255\n";
	for (int i = 0; i < imgW * imgH; ++i)
	{
		unsigned char n = static_cast<unsigned char>(noiseMap[i] * 255);
		ofs << n << n << n;
	}
	ofs.close();
	delete[] noiseMap;
}

#endif
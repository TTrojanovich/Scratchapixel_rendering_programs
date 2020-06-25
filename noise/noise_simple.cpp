#define NOISE_SIMPLE_
#ifdef NOISE_SIMPLE_1
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


class ValueNoise
{
public:
	static const unsigned MaxTableSize = 256;
	static const unsigned MaxTableSizeMask = MaxTableSize - 1;
	float r[MaxTableSize] = { 0 };
	unsigned permutationTable[MaxTableSize * 2] = { 0 };

	ValueNoise(unsigned seed = 2020)
	{
		mt19937 gen(seed);
		uniform_real_distribution<float> distr;
		for (int i = 0; i < MaxTableSize; ++i)
		{
			r[i] = distr(gen);
			permutationTable[i] = i;
		}

		uniform_int_distribution<unsigned> distrUInt;
		for (int k = 0; k < MaxTableSize; ++k) 
		{
			unsigned i = distrUInt(gen) & MaxTableSizeMask;
			swap(permutationTable[k], permutationTable[i]);
			permutationTable[k + MaxTableSize] = permutationTable[k];
		}
	}

	float eval(const Vec2f& p)
	{
		int xi = floor(p.x);
		int yi = floor(p.y);

		float tx = p.x - xi;
		float ty = p.y - yi;

		int rx0 = xi & MaxTableSizeMask;
		int rx1 = (rx0 + 1) & MaxTableSizeMask;
		int ry0 = yi & MaxTableSizeMask;
		int ry1 = (ry0 + 1) & MaxTableSizeMask;

		const float& c00 = r[permutationTable[permutationTable[rx0] + ry0]];
		const float& c10 = r[permutationTable[permutationTable[rx1] + ry0]];
		const float& c01 = r[permutationTable[permutationTable[rx0] + ry1]];
		const float& c11 = r[permutationTable[permutationTable[rx1] + ry1]];

		float sx = smoothstep(tx);
		float sy = smoothstep(ty);

		float nx0 = lerp(c00, c10, sx);
		float nx1 = lerp(c01, c11, sx);

		return lerp(nx0, nx1, sy);
	}
};


int main()
{
	int imgW = 512, imgH = 512;
	float* noiseMap = new float[imgW * (long long)imgH]{ 0 };
	string FileName;

//#define WHITE_NOISE
//#define VALUE_NOISE
//#define FRACTAL_PATTERN_NOISE
//#define TURBULENCE_PATTERN_NOISE
//#define MARBLE_PATTERN_NOISE
#define WOOD_PATTERN_NOISE

#ifdef WHITE_NOISE
	
	FileName = "noise_white.ppm";
	unsigned seed = 2020;
	mt19937 gen(seed);
	uniform_real_distribution<float> distr;
	auto dice = bind(distr, gen);
	for (int j = 0; j < imgH; ++j) 
		for (int i = 0; i < imgW; ++i)
			noiseMap[j * imgW + i] = distr(gen);
#else
#ifdef VALUE_NOISE
	FileName = "noise_value.ppm";
	ValueNoise noise;
	float frequency = 0.05f;
	for (int j = 0; j < imgH; ++j)
		for (int i = 0; i < imgW; ++i)
			noiseMap[j * imgW + i] = noise.eval(Vec2f(i, j) * frequency);
#else
#ifdef FRACTAL_PATTERN_NOISE
	FileName = "noise_fractal.ppm";
	ValueNoise noise;
	float frequency = 0.02f;
	float frequencyMult = 1.8f;
	float amplitudeMult = 0.35f;
	unsigned numLayers = 5;
	float maxNoiseVal = 0;
	for (unsigned j = 0; j < imgH; ++j) 
	{
		for (unsigned i = 0; i < imgW; ++i) 
		{
			Vec2f pNoise = Vec2f(i, j) * frequency;
			float amplitude = 1;
			for (unsigned l = 0; l < numLayers; ++l) 
			{
				noiseMap[j * imgW + i] += noise.eval(pNoise) * amplitude;
				pNoise *= frequencyMult;
				amplitude *= amplitudeMult;
			}
			if (noiseMap[j * imgW + i] > maxNoiseVal)
				maxNoiseVal = noiseMap[j * imgW + i];
		}
	}
	for (unsigned i = 0; i < imgW * imgH; ++i) 
		noiseMap[i] /= maxNoiseVal;
#else
#ifdef TURBULENCE_PATTERN_NOISE
	FileName = "noise_turbulence.ppm";
	ValueNoise noise;
	float frequency = 0.02f;
	float frequencyMult = 1.8f;
	float amplitudeMult = 0.35f;
	unsigned numLayers = 5;
	float maxNoiseVal = 0;
	for (unsigned j = 0; j < imgH; ++j)
	{
		for (unsigned i = 0; i < imgW; ++i)
		{
			Vec2f pNoise = Vec2f(i, j) * frequency;
			float amplitude = 1;
			for (unsigned l = 0; l < numLayers; ++l)
			{
				noiseMap[j * imgW + i] += fabs(2 * noise.eval(pNoise) - 1) * amplitude;
				pNoise *= frequencyMult;
				amplitude *= amplitudeMult;
			}
			if (noiseMap[j * imgW + i] > maxNoiseVal)
				maxNoiseVal = noiseMap[j * imgW + i];
		}
}
	for (unsigned i = 0; i < imgW * imgH; ++i)
		noiseMap[i] /= maxNoiseVal;
#else
#ifdef MARBLE_PATTERN_NOISE
	FileName = "noise_marble.ppm";
	ValueNoise noise;
	float frequency = 0.02f;
	float frequencyMult = 1.8f;
	float amplitudeMult = 0.35f;
	unsigned numLayers = 5;
	float maxNoiseVal = 0;
	for (unsigned j = 0; j < imgH; ++j)
	{
		for (unsigned i = 0; i < imgW; ++i)
		{
			Vec2f pNoise = Vec2f(i, j) * frequency;
			float amplitude = 1;
			float noiseValue = 0;
			for (unsigned l = 0; l < numLayers; ++l)
			{
				noiseValue += noise.eval(pNoise) * amplitude;
				pNoise *= frequencyMult;
				amplitude *= amplitudeMult;
			}
			noiseMap[j * imgW + i] = (sin((i + (double)noiseValue * 100) * 2 * M_PI / 200.f) + 1) / 2.f;
		}
	}
#else
#ifdef WOOD_PATTERN_NOISE	
    FileName = "noise_wood.ppm";
	ValueNoise noise;
	float frequency = 0.01f;
    for (unsigned j = 0; j < imgH; ++j) 
    {
	    for (unsigned i = 0; i < imgW; ++i) 
	    {
		    float g = noise.eval(Vec2f(i, j) * frequency) * 20;
		    noiseMap[j * imgW + i] = g - (int)g;
    	}
    }
#endif
#endif
#endif
#endif
#endif
#endif

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
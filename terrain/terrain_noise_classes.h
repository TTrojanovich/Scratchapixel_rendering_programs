#pragma once

using namespace std;


template<typename T = float>
T lerp(const T& lo, const T& hi, const T& t);
float quintic(const float& t);
float quinticDeriv(const float& t);


class Noise
{
public:
	string type;
	virtual string getType() const = 0;
	virtual float eval(const Vec3f&) const = 0;
	virtual void print(const int&, const int&) const = 0;
};


class PerlinNoise : public Noise
{
public:
	static const unsigned tableSize = 256;
	static const unsigned tableSizeMask = tableSize - 1;
	Vec3f gradients[tableSize];
	unsigned permutationTable[tableSize * 2] = { 0 };

	PerlinNoise(const unsigned seed = 2020)
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

		float u = quintic(tx);
		float v = quintic(ty);
		float w = quintic(tz);

		float x0 = tx, x1 = tx - 1;
		float y0 = ty, y1 = ty - 1;
		float z0 = tz, z1 = tz - 1;

		float a = gradientDotV(hash(xi0, yi0, zi0), x0, y0, z0);
		float b = gradientDotV(hash(xi1, yi0, zi0), x1, y0, z0);
		float c = gradientDotV(hash(xi0, yi1, zi0), x0, y1, z0);
		float d = gradientDotV(hash(xi1, yi1, zi0), x1, y1, z0);
		float e = gradientDotV(hash(xi0, yi0, zi1), x0, y0, z1);
		float f = gradientDotV(hash(xi1, yi0, zi1), x1, y0, z1);
		float g = gradientDotV(hash(xi0, yi1, zi1), x0, y1, z1);
		float h = gradientDotV(hash(xi1, yi1, zi1), x1, y1, z1);

		float du = quinticDeriv(tx);
		float dv = quinticDeriv(ty);
		float dw = quinticDeriv(tz);

		float k0 = a;
		float k1 = (b - a);
		float k2 = (c - a);
		float k3 = (e - a);
		float k4 = (a + d - b - c);
		float k5 = (a + f - b - e);
		float k6 = (a + g - c - e);
		float k7 = (b + c + e + h - a - d - f - g);

		//derivs.x = du * (k1 + k4 * v + k5 * w + k7 * v * w);
		//derivs.y = dv * (k2 + k4 * u + k6 * w + k7 * v * w);
		//derivs.z = dw * (k3 + k5 * u + k6 * v + k7 * v * w);

		return k0 + k1 * u + k2 * v + k3 * w + k4 * u * v + k5 * u * w + k6 * v * w + k7 * u * v * w;
	}

	int hash(const int& x, const int& y, const int& z) const
	{
		return permutationTable[permutationTable[permutationTable[x] + y] + z];
	}

	float gradientDotV
	(
		int perm,
		float x, float y, float z
	) const
	{
		switch (perm & 15) {
		case  0: return  x + y; // (1,1,0)
		case  1: return -x + y; // (-1,1,0)
		case  2: return  x - y; // (1,-1,0)
		case  3: return -x - y; // (-1,-1,0)
		case  4: return  x + z; // (1,0,1)
		case  5: return -x + z; // (-1,0,1)
		case  6: return  x - z; // (1,0,-1)
		case  7: return -x - z; // (-1,0,-1)
		case  8: return  y + z; // (0,1,1),
		case  9: return -y + z; // (0,-1,1),
		case 10: return  y - z; // (0,1,-1),
		case 11: return -y - z; // (0,-1,-1)
		case 12: return  y + x; // (1,1,0)
		case 13: return -x + y; // (-1,1,0)
		case 14: return -y + z; // (0,-1,1)
		case 15: return -y - z; // (0,-1,-1)
		}
	}

	void print(const int& imgW, const int& imgH) const
	{
		float* noiseMap = new float[imgW * (long long)imgH]{ 0 };
		string FileName = "terrain_perlin_improved.ppm";

		for (int j = 0; j < imgH; ++j)
			for (int i = 0; i < imgW; ++i)
				noiseMap[j * imgW + i] = (eval(Vec3f((float)i, 0, (float)j) * (1 / 32.f)) + 1.f) * 0.5f;

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

	string getType() const
	{
		return "Perlin";
	}
};


class FractalNoise : public PerlinNoise
{
public:

	float eval(const Vec3f& p) const
	{
		Vec3f p_tmp = p;
		float frequency = 0.02f;
		float frequencyMult = 1.8f;
		float amplitudeMult = 0.35f;
		unsigned numLayers = 5;
		float maxNoiseVal = 0;
		float amplitude = 1;
		float noiseRes = 0;
		for (unsigned l = 0; l < numLayers; ++l)
		{
			noiseRes += (1 + PerlinNoise::eval(p_tmp)) * 0.5 * amplitude;
			p_tmp *= frequencyMult;
			amplitude *= amplitudeMult;
		}
		return noiseRes;
	}

	void print(const int& imgW, const int& imgH) const
	{
		float* noiseMap = new float[imgW * (long long)imgH]{ 0 };
		string FileName = "terrain_fractal.ppm";

		float frequency = 0.02f;
		float frequencyMult = 1.8f;
		float amplitudeMult = 0.35f;
		int numLayers = 5;
		float maxVal = 0;
		for (int j = 0; j < imgH; ++j) 
		{
			for (int i = 0; i < imgW; ++i)
			{
				float amplitude = 1;
				Vec3f p = Vec3f(i, 0, j) * frequency;
				for (int k = 0; k < numLayers; ++k) 
				{
					noiseMap[j * imgW + i] += (1 + PerlinNoise::eval(p)) * 0.5 * amplitude;
					p *= frequencyMult;
					amplitude *= amplitudeMult;
				}
				if (noiseMap[j * imgW + i] > maxVal) maxVal = noiseMap[j * imgW + i];
			}
		}

		for (int i = 0; i < imgW * imgH; ++i)
			noiseMap[i] /= maxVal;

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
	
	string getType() const
	{
		return "Fractal";
	}
};


class MarbleNoise : public PerlinNoise
{
public:

	float eval(const Vec3f& p) const
	{
		Vec3f p_tmp = p;
		float frequency = 0.02f;
		float frequencyMult = 1.8f;
		float amplitudeMult = 0.35f;
		unsigned numLayers = 5;
		float maxNoiseVal = 0;
		float amplitude = 1;
		float noiseRes = 0;
		for (unsigned l = 0; l < numLayers; ++l)
		{
			noiseRes += (1 + PerlinNoise::eval(p_tmp)) * 0.5 * amplitude;
			p_tmp *= frequencyMult;
			amplitude *= amplitudeMult;
		}
		return noiseRes;
	}

	void print(const int& imgW, const int& imgH) const
	{
		float* noiseMap = new float[imgW * (long long)imgH]{ 0 };
		string FileName = "terrain_marble.ppm";

		float frequency = 0.02f;
		float frequencyMult = 1.8f;
		float amplitudeMult = 0.35f;
		int numLayers = 5;
		float maxVal = 0;
		for (int j = 0; j < imgH; ++j)
		{
			for (int i = 0; i < imgW; ++i)
			{
				float amplitude = 1;
				Vec3f p = Vec3f(i, 0, j) * frequency;
				float noiseVal = 0;
				for (int k = 0; k < numLayers; ++k)
				{
					noiseVal += (1 + PerlinNoise::eval(p)) * 0.5 * amplitude;
					p *= frequencyMult;
					amplitude *= amplitudeMult;
				}
				noiseMap[j * imgW + i] = (sin((i + (double)noiseVal * 100) * 2 * M_PI / 200.f) + 1) / 2.f;
			}
		}

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

	string getType() const
	{
		return "Marble";
	}
};


class WoodNoise : public PerlinNoise
{
public:

	float eval(const Vec3f& p) const
	{
		Vec3f p_tmp = p;
		float frequency = 0.01f;
		
		float g = (PerlinNoise::eval(Vec3f(p_tmp * frequency)) + 1.f) * 0.5f * 20;
		return g - (int)g;
	}

	void print(const int& imgW, const int& imgH) const
	{
		float* noiseMap = new float[imgW * (long long)imgH]{ 0 };
		string FileName = "terrain_wood.ppm";

		float frequency = 0.01f;
		for (int j = 0; j < imgH; ++j)
		{
			for (int i = 0; i < imgW; ++i)
			{
				float g = (PerlinNoise::eval(Vec3f((float)i, 0, (float)j) * frequency) + 1.f) * 0.5f * 20;
				noiseMap[j * imgW + i] = g - (int)g;
			}
		}

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

	string getType() const
	{
		return "Wood";
	}
};



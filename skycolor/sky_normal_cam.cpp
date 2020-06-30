#define SKY_NORMAL_CAM_
#ifdef SKY_NORMAL_CAM_1
#define _USE_MATH_DEFINES

#include <cassert>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <random>
#include <limits> 
#include "../headers/geometry.h"

using namespace std;


class Atmosphere
{
public:
	Vec3f sunDirection;      // The sun direction (normalized)
	const float earthRadius; // In the paper this is usually Rg or Re (radius ground, eart)
	float atmosphereRadius;  // In the paper this is usually R or Ra (radius atmosphere)
	float Hr;                // Thickness of the atmosphere if density was uniform (Hr)
	float Hm;                // Same as above but for Mie scattering (Hm)

	static const Vec3f betaR;
	static const Vec3f betaM;

	Atmosphere
	(
		Vec3f sd = Vec3f(0, 1, 0),
		float er = 6360e3, float ar = 6420e3,
		float hr = 7994, float hm = 1200
	) :
		sunDirection(sd),
		earthRadius(er),
		atmosphereRadius(ar),
		Hr(hr),
		Hm(hm)
	{}

	Vec3f computeIncidentLight(const Vec3f& orig, const Vec3f& dir, float tmin, float tmax) const;
};

const Vec3f Atmosphere::betaR(3.8e-6f, 13.5e-6f, 33.1e-6f);
const Vec3f Atmosphere::betaM(21e-6f);

bool solveQuadratic(float a, float b, float c, float& x1, float& x2)
{
	if (b == 0)
	{
		if (a == 0) return false;
		x1 = 0;
		x2 = sqrtf(-c / a);
		return true;
	}
	float discr = b * b - 4 * a * c;

	if (discr < 0) return false;

	float q = (b < 0.f) ? -0.5f * (b - sqrtf(discr)) : -0.5f * (b + sqrtf(discr));
	x1 = q / a;
	x2 = c / q;

	return true;
}

bool raySphereIntersect(const Vec3f& orig, const Vec3f& dir, const float& radius, float& t0, float& t1)
{
	float A = 1; // dir.dotProduct(dir);
	float B = 2 * dir.dotProduct(orig);
	float C = orig.dotProduct(orig) - radius * radius;

	if (!solveQuadratic(A, B, C, t0, t1)) return false;

	if (t0 > t1) swap(t0, t1);

	return true;
}


Vec3f Atmosphere::computeIncidentLight(const Vec3f& orig, const Vec3f& dir, float tmin, float tmax) const
{
	float t0, t1; // camera inside the atmosphere, so generally t0 < 0, t1 > 0 ??
	if (!raySphereIntersect(orig, dir, atmosphereRadius, t0, t1) || t1 < 0) return 0;
	if (t0 > tmin&& t0 > 0) tmin = t0;
	if (t1 < tmax) tmax = t1;
	int numSamples = 16;
	int numSamplesLight = 8;
	float segmentLength = (tmax - tmin) / numSamples;
	float tCurrent = tmin;

	Vec3f sumR(0), sumM(0); // rayleigh and mie contribution
	float opticalDepthR = 0, opticalDepthM = 0;

	float mu = dir.dotProduct(sunDirection); // mu in the paper which is the cosine of the angle between the sun direction and the ray direction
	float phaseR = 3.f / (16.f * M_PI) * (1 + (double)mu * mu);
	float g = 0.76f;
	float phaseM = 3.f / (8.f * M_PI) * ((1.f - double(g) * g) * (1.f + double(mu) * mu)) / ((2.f + double(g) * g) * pow(1.f + g * g - 2.f * g * mu, 1.5f));

	for (int i = 0; i < numSamples; ++i)
	{
		Vec3f samplePosition = orig + (tCurrent + segmentLength * 0.5f) * dir;
		float height = samplePosition.length() - earthRadius;
		// compute optical depth
		float hr = exp(-height / Hr) * segmentLength;
		float hm = exp(-height / Hm) * segmentLength;
		opticalDepthR += hr;
		opticalDepthM += hm;

		// light optical depth
		float t0Light, t1Light;
		raySphereIntersect(samplePosition, sunDirection, atmosphereRadius, t0Light, t1Light);
		float segmentLengthLight = t1Light / numSamplesLight;
		float tCurrentLight = 0;
		float opticalDepthLightR = 0, opticalDepthLightM = 0;

		int j;
		for (j = 0; j < numSamplesLight; ++j)
		{
			Vec3f samplePositionLight = samplePosition + (tCurrentLight + segmentLengthLight * 0.5f) * sunDirection;
			float heightLight = samplePositionLight.length() - earthRadius;
			if (heightLight < 0) break; // if sun ray in earth shadow

			opticalDepthLightR += exp(-heightLight / Hr) * segmentLengthLight;
			opticalDepthLightM += exp(-heightLight / Hm) * segmentLengthLight;
			tCurrentLight += segmentLengthLight;
		}

		if (j == numSamplesLight) // if sun ray is NOT in shadow, all light samples pass
		{
			Vec3f tau = betaR * (opticalDepthR + opticalDepthLightR) + betaM * 1.1f * (opticalDepthM + opticalDepthLightM);
			Vec3f attenuation(exp(-tau.x), exp(-tau.y), exp(-tau.z)); // transmittance
			sumR += attenuation * hr; // Transmittance(Pc, X) * Transmittance(X, Ps)
			sumM += attenuation * hm; // for current X on view ray
		}
		tCurrent += segmentLength;
	}

	// 20 = magic number for sun intensity
	return (sumR * betaR * phaseR + sumM * betaM * phaseM) * 20;
}




void renderSkydome(const Vec3f& sunDir, const char* filename)
{
	Atmosphere atmosphere(sunDir);
	auto t0 = chrono::high_resolution_clock::now();

    // Render from a normal camera
	const int width = 640, height = 480;
	Vec3f* image = new Vec3f[width * height]; 
	Vec3f *p = image;
	memset(image, 0x0, sizeof(Vec3f) * width * height);

	float aspectRatio = width / float(height);
	float fov = 65;
	float angle = tan(fov * M_PI / 180 * 0.5f);

	int numPixelSamples = 4;
	Vec3f orig(0, atmosphere.earthRadius + 1000, 30000); // camera position

	default_random_engine generator;
	uniform_real_distribution<float> distribution(0, 1); // to generate random floats in the range [0:1]
	
	for (int y = 0; y < height; ++y) 
	{
		for (int x = 0; x < width; ++x) 
		{
			for (int m = 0; m < numPixelSamples; ++m) 
			{
				for (int n = 0; n < numPixelSamples; ++n) // compute pixel color via 16 subpixels
				{
					// instead of only middle of pixel, we get 16 subpixels ?? not sure
					float rayx = (2 * (x + (m + distribution(generator)) / numPixelSamples) / float(width) - 1) * aspectRatio * angle;
					float rayy = (1 - (y + (n + distribution(generator)) / numPixelSamples) / float(height) * 2) * angle;
					Vec3f dir(rayx, rayy, -1);
					dir.normalize();
					// Does the ray intersect the planetory body? (the intersection test is against the Earth here
					// not against the atmosphere). If the ray intersects the Earth body and that the intersection
					// is ahead of us, then the ray intersects the planet in 2 points, t0 and t1. But we
					// only want to compute the atmosphere between t=0 and t=t0 (where the ray hits
					// the Earth first). If the viewing ray doesn't hit the Earth, or course the ray
					// is then bounded to the range [0:INF]. In the method computeIncidentLight() we then
					// compute where this primary ray intersects the atmosphere and we limit the max t range 
					// of the ray to the point where it leaves the atmosphere.
					float t0, t1, tMax = INFINITY;
					// t0 > 0, not t1... i guess...
					if (raySphereIntersect(orig, dir, atmosphere.earthRadius, t0, t1) && t0 > 0) tMax = max(0.f, t0);

					// The *viewing or camera ray* is bounded to the range [0:tMax]
					*p += atmosphere.computeIncidentLight(orig, dir, 0, tMax);
				}
			}
			*p *= 1.f / (numPixelSamples * numPixelSamples); // divide by the number of subpixels
			++p;
		}
		fprintf(stderr, "\b\b\b\b%3d%c", (int)(100 * y / (width - 1)), '%');
	}
	

	cout << "\b\b\b\b" << ((chrono::duration<float>)(chrono::high_resolution_clock::now() - t0)).count() << " seconds" << endl;

	ofstream ofs(filename, ios::out | ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";
	p = image;
	for (int j = 0; j < height; ++j)
	{
		for (int i = 0; i < width; ++i, ++p)
		{
			// Apply tone mapping function
			(*p)[0] = (*p)[0] < 1.413f ? pow((*p)[0] * 0.38317f, 1.0f / 2.2f) : 1.0f - exp(-(*p)[0]);
			(*p)[1] = (*p)[1] < 1.413f ? pow((*p)[1] * 0.38317f, 1.0f / 2.2f) : 1.0f - exp(-(*p)[1]);
			(*p)[2] = (*p)[2] < 1.413f ? pow((*p)[2] * 0.38317f, 1.0f / 2.2f) : 1.0f - exp(-(*p)[2]);

			ofs << (unsigned char)(min(1.f, (*p)[0]) * 255)
				<< (unsigned char)(min(1.f, (*p)[1]) * 255)
				<< (unsigned char)(min(1.f, (*p)[2]) * 255);
		}
	}
	ofs.close();
	delete[] image;
}


int main()
{
	// Render one single image
	float angle = 7 / (float)9 * M_PI * 0.6f;
	Vec3f sunDir(0, cos(angle), -sin(angle));
	cerr << "Sun direction: " << sunDir << endl;
	renderSkydome(sunDir, "./sky_normal_cam.ppm");
}


#endif


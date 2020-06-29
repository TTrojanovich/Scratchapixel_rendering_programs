#define TERRAIN_PERLIN_IMPROVED_
#ifdef TERRAIN_PERLIN_IMPROVED_1

#include "terrain_header.h"

using namespace std;


template<typename T = float>
T lerp(const T& lo, const T& hi, const T& t)
{
	return lo * (1 - t) + hi * t;
}
float quintic(const float& t)
{
	return t * t * t * (t * (t * 6 - 15) + 10);
}
float quinticDeriv(const float& t)
{
	return 30 * t * t * (t * (t - 2) + 1);
}


class PolyMesh
{
public:
	unique_ptr<Vec3f[]> vertices;
	unique_ptr<Vec2f[]> st;
	unique_ptr<Vec3f[]> normals;
	unique_ptr<int[]> faceArray;
	unique_ptr<int[]> verticesArray;
	int numVertices;
	int numFaces;

	PolyMesh() : vertices(nullptr), st(nullptr), normals(nullptr), numVertices(0), numFaces(0) {}
	~PolyMesh() {}
};


PolyMesh* createPolyMesh
(
	const Matrix44f& objToWorld,
	const Noise& noise,
	int width = 1,
	int height = 1,
	int subdivisionWidth = 40,
	int subdivisionHeight = 40
)
{
	PolyMesh* poly = new PolyMesh;
	poly->numVertices = (subdivisionWidth + 1) * (subdivisionHeight + 1);
	poly->vertices = unique_ptr<Vec3f[]>(new Vec3f[poly->numVertices]);
	poly->normals = unique_ptr<Vec3f[]>(new Vec3f[poly->numVertices]);
	poly->st = unique_ptr<Vec2f[]>(new Vec2f[poly->numVertices]);
	float invSubdivisionWidth = 1.f / subdivisionWidth;
	float invSubdivisionHeight = 1.f / subdivisionHeight;
	for (int j = 0; j <= subdivisionHeight; ++j)
	{
		for (int i = 0; i <= subdivisionWidth; ++i)
		{
			poly->vertices[j * (subdivisionWidth + 1) + i] = Vec3f(width * ((long long)i * invSubdivisionWidth - 0.5f), 0, height * ((long long)j * invSubdivisionHeight - 0.5f));
			poly->st[j * (subdivisionWidth + 1) + i] = Vec2f(i * invSubdivisionWidth, j * invSubdivisionHeight);
		}
	}

	poly->numFaces = subdivisionWidth * subdivisionHeight;
	poly->faceArray = unique_ptr<int[]>(new int[poly->numFaces]);
	for (int i = 0; i < poly->numFaces; ++i)
		poly->faceArray[i] = 4;

	poly->verticesArray = unique_ptr<int[]>(new int[4LL * poly->numFaces]);
	for (int j = 0, k = 0; j < subdivisionHeight; ++j)
	{
		for (int i = 0; i < subdivisionWidth; ++i) 
		{
			poly->verticesArray[k] = j * (subdivisionWidth + 1) + i;
			poly->verticesArray[k + 1LL] = j * (subdivisionWidth + 1) + i + 1;
			poly->verticesArray[k + 2LL] = (j + 1) * (subdivisionWidth + 1) + i + 1;
			poly->verticesArray[k + 3LL] = (j + 1) * (subdivisionWidth + 1) + i;
			k += 4;
		}
	}

	int maxVal = 0;
	for (int i = 0; i < poly->numVertices; ++i)
	{
		Vec3f p((poly->vertices[i].x + 0.5), 0, (poly->vertices[i].z + 0.5));
		float y_temp = noise.eval(p);

		if (noise.getType() == "Marble")
			y_temp = (sin((i % 512 + (double)y_temp * 100) * 2 * M_PI / 200.f) + 1) / 2.f;
		
		poly->vertices[i].y = y_temp;
		
		if (noise.getType() == "Fractal") 
			if (y_temp > maxVal) maxVal = y_temp;
	}

	if (noise.getType() == "Fractal")
		for (int i = 0; i < poly->numVertices; ++i)
			poly->vertices[i].y /= maxVal;

	for (int i = 0; i < poly->numVertices; ++i)
	{
		Vec3f vert_temp = poly->vertices[i];
		objToWorld.multVecMatrix(vert_temp, poly->vertices[i]);
	}

	return poly;
}




int main()
{
	int imgW = 512, imgH = 512;
	Matrix44f camToWorld1;
	Vec3f cam_origin(8, 3, 5), cam_target(3, 0, 5);
	camToWorld1 = LookAt(cam_origin, cam_target);
	Parameters parameters(imgW, imgH, 60, camToWorld1, cam_origin, cam_target);
	Matrix44f objToWorld = Matrix44f((float)cos(M_PI), (float)-sin(M_PI), 0, 0, (float)sin(M_PI), (float)cos(M_PI), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
	//Matrix44f objToWorld = Matrix44f(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 3, 0, 1);
	
	//PerlinNoise noise;
	//FractalNoise noise;
	//MarbleNoise noise;
	WoodNoise noise;
	
	PolyMesh* poly = createPolyMesh(objToWorld, noise, 10, 10, 40, 40);
	
	vector<unique_ptr<Object>> objects;
	TriangleMesh* mesh = new TriangleMesh
	(
	poly->numFaces,
	poly->faceArray, poly->verticesArray,
	poly->vertices, poly->normals, poly->st
	);
	
	if (mesh != nullptr) objects.push_back(unique_ptr<Object>(mesh));

	
	string FileName;
	if (noise.getType() == "Perlin") FileName = "terrain_render_perlin_out.ppm";
	if (noise.getType() == "Fractal") FileName = "terrain_render_fractal_out.ppm";
	if (noise.getType() == "Marble") FileName = "terrain_render_marble_out.ppm";
	if (noise.getType() == "Wood") FileName = "terrain_render_wood_out.ppm";
	render(parameters, objects, FileName);

	noise.print(imgW, imgH);

	delete poly;
}

#endif
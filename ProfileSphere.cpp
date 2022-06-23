// profile.cpp : Source file for your target.
//

#include "Profile.h"
#include "SimplexNoise.h"
#include <igl/marching_cubes.h>
//#include <igl/copyleft/marching_cubes.h>
#include <igl/writeOBJ.h>

void SetSphere(MarchingCubes& mc)
{
	const int cX = mc.resX / 2;
	const int cY = mc.resY / 2;
	const int cZ = mc.resZ / 2;
	const float scale = 1.0f / min(min(mc.resX, mc.resY), mc.resZ);
	for (int i = 0; i < mc.resX; i++)
	{
		for (int j = 0; j < mc.resY; j++)
		{
			for (int k = 0; k < mc.resZ; k++)
			{
				float val = (i - cX) * (i - cX) + (j - cY) * (j - cY) + (k - cZ) * (k - cZ);
				val = sqrt(val);
				val *= scale;
				mc.setIsoValue(i, j, k, val);
			}
		}
	}
}

void SetRandom(MarchingCubes& mc, float _lo = -1.0, float _hi = 1.0)
{
	srand(1);
	for (int i = 0; i < mc.resX; i++)
	{
		for (int j = 0; j < mc.resY; j++)
		{
			for (int k = 0; k < mc.resZ; k++)
			{
				float freq = 20.0;
				//float val = _lo + static_cast<float>(rand()) / (static_cast <float> (RAND_MAX / (_hi - _lo)));
				float x = 1.0f * i / mc.resX;
				float y = 1.0f * j / mc.resY;
				float z = 1.0f * k / mc.resZ;
				float val = SimplexNoise::noise(freq * x, freq * y, freq * z);
				mc.setIsoValue(i, j, k, val);
			}
		}
	}
}

void SetSphere(Eigen::MatrixXf& points, Eigen::VectorXf& values, int resX, int resY, int resZ)
{
	points.resize(resX * resY * resZ, 3);
	values.resize(resX * resY * resZ);

	int sx = resX - 1;
	int sy = resY - 1;
	int sz = resZ - 1;

	float dx = 1.0f / sx;
	float dy = 1.0f / sy;
	float dz = 1.0f / sz;

	const int cX = resX / 2;
	const int cY = resY / 2;
	const int cZ = resZ / 2;
	const float scale = 1.0f/min(min(resX, resY), resZ);

	int ipoint = 0;
	for (int k = 0; k < resZ; k++)
	{
		for (int j = 0; j < resY; j++)
		{
			for (int i = 0; i < resX; i++)
			{
				float val = (i - cX) * (i - cX) + (j - cY) * (j - cY) + (k - cZ) * (k - cZ);
                val = sqrt(val);
				val *= scale;
				
				values[ipoint] = val;
				points.row(ipoint) = Eigen::RowVector3f(k * dz, j * dy, i * dx);

				++ipoint;
			}
		}
	}
}

void SetRandom(Eigen::MatrixXf& points, Eigen::VectorXf& values, int resX, int resY, int resZ, float _lo = -1.0, float _hi = 1.0)
{
	points.resize(resX * resY * resZ, 3);
	values.resize(resX * resY * resZ);

	int sx = resX - 1;
	int sy = resY - 1;
	int sz = resZ - 1;

	float dx = 1.0f / sx;
	float dy = 1.0f / sy;
	float dz = 1.0f / sz;

	int ipoint = 0;

	for (int k = 0; k < resZ; k++)
	{
		for (int j = 0; j < resY; j++)
		{
			for (int i = 0; i < resX; i++)
			{
				float freq = 20.0;
				float x = 1.0f * i / resX;
				float y = 1.0f * j / resY;
				float z = 1.0f * k / resZ;
				float val = SimplexNoise::noise(freq * z, freq * y, freq * x);

				values[ipoint] = val;
				points.row(ipoint) = Eigen::RowVector3f(k * dz, j * dy, i * dx);

				++ipoint;
			}
		}
	}
}

int main(int argc, char** argv)
{
	//if (argc != 6) { printf("usage: <res> <threshold> <blockX> <blockY> <blockZ>. \n"); return -1; }
	//int res = atoi(argv[1]);
	//float radius = atof(argv[2]);
	//int blockX = atoi(argv[3]);
	//int blockY = atoi(argv[4]);
	//int blockZ = atoi(argv[5]);
	//printf("resolution=%d\n", res);

	bool bUseSphere = false;

	float radius = 0.0;  // For Perlin noise use this!
	if(bUseSphere)
		radius = 0.4;
	

	int resx = 200, resy = 200, resz= 200;


	{
		// validate number of vertices
		// baseline result
		MarchingCubes mc;
		mc.setup(resx, resy, resz);
		if (bUseSphere)
			SetSphere(mc);
		else
			SetRandom(mc);
		//const float radius = 0.4;
		mc.update_level(radius);
		//mc.exportObj("Sphere");
		printf("%d\n", mc.vertices.size());
	}


	{
		Eigen::MatrixXf points;
		Eigen::VectorXf values;
		Eigen::MatrixXf V;
		Eigen::MatrixXi F;

		if (bUseSphere)
			SetSphere(points, values, resx, resy, resz);
		else
			SetRandom(points, values, resx, resy, resz);



		// result
		//igl::copyleft::marching_cubes(values, points, resx, resy, resz, radius, V, F);
		igl::marching_cubes(values, points, resx, resy, resz, radius, V, F);
		printf("%d\n", V.rows());

		//igl::writeOBJ("Sphere_igl.obj", V, F);

		//// timing
		//void (MarchingCubes:: * ptr_update)(float) = &MarchingCubes::update;
		//LARGE_INTEGER f;
		//QueryPerformanceFrequency((LARGE_INTEGER*)&f);
		//double p = queryperfcounter(mc, ptr_update, radius, f);
		//printf("Windows QueryPerformanceCounter() timing: %lf seconds. ==> %lf cycles based on FRENQUENCY.\n\n", p / f.QuadPart, p / f.QuadPart * FREQUENCY);
	}
    

	return 0;
}

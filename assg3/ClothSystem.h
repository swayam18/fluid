#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vecmath.h>
#include <vector>
#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"

class ClothSystem: public ParticleSystem
{
///ADD MORE FUNCTION AND FIELDS HERE
public:
	ClothSystem(int in_length, int  in_height);
	vector<Vector3f> evalF(vector<Vector3f> state);
	int indexOf(int x, int y);

	void toggleWind();
	void toggleBall();
	void toggleWireframe();
	void incrementTime();
	void draw();

private:
	bool ball;
	bool windt;
	bool wireframe;
	float random;
	int length;
	int height;

	static const float MASS;
	static const float RESTLEN;

	static const float STRUCT_COEFF;
	static const float SHEAR_COEFF;
	static const float FLEX_COEFF;

	static const Vector3f GRAVITY;
	static const float DRAG_COEFF;

	static const float STRUCT_LEN;
	static const float SHEAR_LEN;
	static const float FLEX_LEN;
	
	static float genRandomFloat() {return (float)rand() / (float)RAND_MAX;};
	Vector3f Differential_Helper(vector<Vector3f> state, int j, int i);
	Vector3f calcNormal(int j, int i);

};


#endif

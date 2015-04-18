#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vecmath.h>
#include <vector>
#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif

#include "particleSystem.h"

class PendulumSystem: public ParticleSystem
{
public:
	PendulumSystem(int numParticles);
	
	vector<Vector3f> evalF(vector<Vector3f> state);
	void incrementTime();
	void draw();

private:

	static const float MASS;
	static const Vector3f GRAVITY;

	static const float DRAG_COEFF;

	static const float SPRING_COEFF;
	static const float LEN;
	
	Vector3f Differential_Helper(vector<Vector3f> state, int i);

};

#endif

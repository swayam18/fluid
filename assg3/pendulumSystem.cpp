
#include "pendulumSystem.h"

const float PendulumSystem::MASS = 1.0f;
const Vector3f PendulumSystem::GRAVITY = Vector3f(0, -1.0, 0) * MASS;

const float PendulumSystem::DRAG_COEFF = 0.55f;

const float PendulumSystem::SPRING_COEFF = 10.0f;
const float PendulumSystem::LEN = 0.4f;

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)
{
	m_numParticles = numParticles;
	vector<Vector3f> pos;

	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles; i++) {
		pos.push_back(Vector3f(0,3.5-i*1.5,0));		
		pos.push_back(Vector3f(i,0,0));
	}

	setState(pos);
}


// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{

	vector<Vector3f> f;

	//1st particle has 0 forces
	f.push_back(state[1]);
	f.push_back(Vector3f(0,0,0));

	for (unsigned i=1; i < m_numParticles; i++) {
		f.push_back(state[2*i+1]);
		f.push_back(Differential_Helper(state,i));
	}

	return f;
}

// render the system (ie draw the particles)

Vector3f PendulumSystem::Differential_Helper(vector<Vector3f> state, int i)
{
	Vector3f x = state[2*i];
	Vector3f v = state[2*i+1];

	Vector3f drag = -DRAG_COEFF * v;
	
	Vector3f spring = Vector3f(0,0,0);

	float distanceprev = (x - state[2*i-2]).abs();
	spring += -SPRING_COEFF * (distanceprev - LEN) * (x-state[2*i-2]).normalized();

	if (i < m_numParticles - 1) //if not last particle
	{
		float distancenext = (x - state[2*i+2]).abs();
		spring += -SPRING_COEFF * (distancenext - LEN) * (x-state[2*i+2]).normalized();
	}

	return (GRAVITY + drag + spring) / MASS;
}

void PendulumSystem::incrementTime()
{
}

void PendulumSystem::draw()
{
	for (int i = 0; i < m_numParticles; i++) {
		Vector3f pos = getState()[2*i];
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
	}
}

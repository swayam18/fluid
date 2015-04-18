
#include "simpleSystem.h"

using namespace std;

SimpleSystem::SimpleSystem()
{
	m_numParticles = 1;
	vector<Vector3f> pos;
	pos.push_back(Vector3f(0.5, 0.5, 0.0f));
	setState(pos);
}

SimpleSystem::SimpleSystem(float x, float y, float z)
{
	m_numParticles = 1;
	vector<Vector3f> pos;
	pos.push_back(Vector3f(x, y, z));
	setState(pos);
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	for (unsigned i=0; i < state.size(); i++) {
		Vector3f x = state[i];
		f.push_back(Vector3f(-x[1], x[0], 0));	
	}
	return f;

}

// render the system (ie draw the particles)
void SimpleSystem::draw()
{
		Vector3f pos = getState()[0];//YOUR PARTICLE POSITION
	  glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
}

void SimpleSystem::incrementTime()
{
}

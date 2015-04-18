#include "ClothSystem.h"

const float ClothSystem::MASS = 0.25f;
const float ClothSystem::RESTLEN =  0.3f;

const float ClothSystem::STRUCT_COEFF = 180.0f;
const float ClothSystem::SHEAR_COEFF = 180.0f;
const float ClothSystem::FLEX_COEFF = 180.0f;

const Vector3f ClothSystem::GRAVITY = Vector3f(0,-3.0f,0) * ClothSystem::MASS;
const float ClothSystem::DRAG_COEFF = 0.35f;

const float ClothSystem::STRUCT_LEN = ClothSystem::RESTLEN;
const float ClothSystem::SHEAR_LEN = ClothSystem::RESTLEN * 1.4142f;
const float ClothSystem::FLEX_LEN = 2.0 * ClothSystem::RESTLEN;


ClothSystem::ClothSystem(int in_length, int in_height)
{
	wireframe = false;
	windt = true;
	ball = true;

	time = 0;
	random = 0.2 + 0.7*genRandomFloat();
	m_numParticles = in_length * in_height;
	length = in_length;
	height = in_height;

	vector<Vector3f> posvec;

	for (int i = 0; i < height; i++) //pos
	{
		for (int j = 0; j < length; j++)
		{
			posvec.push_back(Vector3f(-1.0 + j*RESTLEN, 3.0 , i*RESTLEN*1.0));
		}
	}

	for (int i = 0; i < height; i++) //vec
	{
		for (int j = 0; j < length; j++)
		{
			posvec.push_back(Vector3f(0,0,0));
		}
	}

	setState(posvec);
}

//Helper, also checks if x y within cloth
//	returns index if valid
//	returns -1 if not valid
int ClothSystem::indexOf(int x, int y)
{
	if (x < 0 || y < 0) { return -1; }
	if ( (x+1)*(y+1) > m_numParticles) {return -1;}
	if ( x > length-1) {return -1;}
	if (y > height-1) {return -1;}

	return (y*length + x);
	
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> ClothSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < length; j++)
		{
			f.push_back(state[m_numParticles + indexOf(j,i)]);
		}
	}

	f.push_back(Vector3f(0,0,0)); //top left hand corner

	for (int j=1; j < length - 1; j++) {
		f.push_back(Differential_Helper(state, j, 0));
	}
	f.push_back(Vector3f(0,0,0)); //top right hand corner

	for (int i=1; i < height; i++){
		for (int j = 0; j < length; j++){

			f.push_back(Differential_Helper(state,j,i));
		}
	}

	return f;
}


void ClothSystem::incrementTime()
{
	//also handles the ball collision haha
	if (ball){
		vector<Vector3f> state = getState();

		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < length; j++)
			{
				if (pow(state[indexOf(j,i)].x(),2) + pow(state[indexOf(j,i)].y() - 1.0,2) + pow(state[indexOf(j,i)].z(),2) < 1.1f)
				{
					Vector3f dir = (state[indexOf(j,i)] - Vector3f(0,1,0)).normalized();
					state[indexOf(j,i)] = Vector3f(0,1,0) + dir * 1.05;
				}
			}
		}
		setState(state);

	}

	//wind simul
	time++;
	if (time > 30)
	{
	random = -0.15 + 0.7*genRandomFloat();
	time = 0;
	}
}

Vector3f ClothSystem::Differential_Helper(vector<Vector3f> state, int j, int i)
{
	Vector3f x = state[indexOf(j,i)];
	Vector3f v = state[m_numParticles+indexOf(j,i)];

	Vector3f drag = -DRAG_COEFF * v;
	
	Vector3f struct_spring = Vector3f(0,0,0);
	Vector3f shear_spring = Vector3f(0,0,0);
	Vector3f flex_spring= Vector3f(0,0,0);

	//STRUCTURAL
	//up
	if (indexOf(j,i-1)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j,i-1)];
		struct_spring += -STRUCT_COEFF * ((prt).abs() - STRUCT_LEN) * prt.normalized();
	}
	//down
	if (indexOf(j,i+1)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j,i+1)];
		struct_spring += -STRUCT_COEFF * ((prt).abs() - STRUCT_LEN) * prt.normalized();
	}
	//left
	if (indexOf(j-1,i)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j-1,i)];
		struct_spring += -STRUCT_COEFF * ((prt).abs() - STRUCT_LEN) * prt.normalized();
	}
	//right
	if (indexOf(j+1,i)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j+1,i)];
		struct_spring += -STRUCT_COEFF * ((prt).abs() - STRUCT_LEN) * prt.normalized();
	}

	//SHEAR
	//up left
	if (indexOf(j-1,i-1)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j-1,i-1)];
		shear_spring += -SHEAR_COEFF * ((prt).abs() - SHEAR_LEN) * prt.normalized();
	}
	//up right
	if (indexOf(j+1,i-1)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j+1,i-1)];
		shear_spring += -SHEAR_COEFF * ((prt).abs() - SHEAR_LEN) * prt.normalized();
	}
	//bot left
	if (indexOf(j-1,i+1)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j-1,i+1)];
		shear_spring += -SHEAR_COEFF * ((prt).abs() - SHEAR_LEN) * prt.normalized();
	}
	//bot right
	if (indexOf(j+1,i+1)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j+1,i+1)];
		shear_spring += -SHEAR_COEFF * ((prt).abs() - SHEAR_LEN) * prt.normalized();
	}


	//FLEX
	//up
	if (indexOf(j,i-2)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j,i-2)];
		flex_spring += -FLEX_COEFF * ((prt).abs() - FLEX_LEN) * prt.normalized();
	}
	//down
	if (indexOf(j,i+2)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j,i+2)];
		flex_spring += -FLEX_COEFF * ((prt).abs() - FLEX_LEN) * prt.normalized();
	}
	//left
	if (indexOf(j-2,i)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j-2,i)];
		flex_spring += -FLEX_COEFF * ((prt).abs() - FLEX_LEN) * prt.normalized();
	}
	//right
	if (indexOf(j+2,i)!=(-1))
	{
		Vector3f prt = x - state[indexOf(j+2,i)];
		flex_spring += -FLEX_COEFF * ((prt).abs() - FLEX_LEN) * prt.normalized();
	}

	Vector3f wind = Vector3f(0,0,0);
	
	if (windt){
	wind += 0.6* Vector3f(0.0, 0.75*random , 2.4* random); 
	}

	return (GRAVITY + drag + struct_spring + shear_spring + flex_spring + wind) / MASS;
	//return Vector3f(0,0,0);
}

///TODO: render the system (ie draw the particles)
void ClothSystem::draw()
{
	if (ball)
	{
		glPushMatrix();
		glTranslatef(0, 1, 0);
		glutSolidSphere(1.0f, 100.0f, 100.0f);
		glPopMatrix();
	}

	glBegin(GL_TRIANGLES);
	GLfloat diffcol[] = {0.9,0.4,0.2,1.0};
	glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffcol);

	//FRONT OF CLOTH - orange (smooth)
	for (int i = 0; i < height-1;i++)
	{
		for (int j = 0; j<length-1; j++)
		{
			Vector3f x = getState()[indexOf(j,i)];
			Vector3f y = getState()[indexOf(j,i+1)];
			Vector3f z = getState()[indexOf(j+1,i)];
			Vector3f normal = calcNormal(j,i);
			glNormal3d(normal[0],normal[1],normal[2]);
			glVertex3d(x[0],x[1],x[2]);
			normal = calcNormal(j,i+1);
			glNormal3d(normal[0],normal[1],normal[2]);
			glVertex3d(y[0],y[1],y[2]);
			normal = calcNormal(j+1,i);
			glNormal3d(normal[0],normal[1],normal[2]);
			glVertex3d(z[0],z[1],z[2]);
		}
	}

	for (int i = 1; i < height;i++)
	{
		for (int j = 0; j<length-1; j++)
		{
			Vector3f x = getState()[indexOf(j,i)];
			Vector3f y = getState()[indexOf(j+1,i)];
			Vector3f z = getState()[indexOf(j+1,i-1)];

			Vector3f normal = calcNormal(j,i);
			glNormal3d(normal[0],normal[1],normal[2]);
			glVertex3d(x[0],x[1],x[2]);
			normal = calcNormal(j+1,i);
			glNormal3d(normal[0],normal[1],normal[2]);
			glVertex3d(y[0],y[1],y[2]);
			normal = calcNormal(j+1,i-1);
			glNormal3d(normal[0],normal[1],normal[2]);
			glVertex3d(z[0],z[1],z[2]);
		}
	}

	GLfloat diffcol2[] = {0.4,0.95,0.5,1.0};
	glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffcol2);

	//BACK OF CLOTH - green (triangles)
	for (int i = 0; i < height-1;i++)
	{
		for (int j = 0; j<length-1; j++)
		{
			Vector3f x = getState()[indexOf(j,i)];
			Vector3f y = getState()[indexOf(j,i+1)];
			Vector3f z = getState()[indexOf(j+1,i)];

			Vector3f normal = Vector3f::cross(getState()[indexOf(j,i+1)] - getState()[indexOf(j,i)], getState()[indexOf(j+1,i)] - getState()[indexOf(j,i)]).normalized();
			glNormal3d(-normal[0],-normal[1],-normal[2]);
			glVertex3d(x[0],x[1],x[2]);
			glVertex3d(z[0],z[1],z[2]);
			glVertex3d(y[0],y[1],y[2]);
			
		}
	}

	for (int i = 1; i < height;i++)
	{
		for (int j = 0; j<length-1; j++)
		{
			Vector3f x = getState()[indexOf(j,i)];
			Vector3f y = getState()[indexOf(j+1,i)];
			Vector3f z = getState()[indexOf(j+1,i-1)];

			Vector3f normal = Vector3f::cross(y - x, z - x).normalized();
			glNormal3d(-normal[0],-normal[1],-normal[2]);
			glVertex3d(x[0],x[1],x[2]);
			glVertex3d(z[0],z[1],z[2]);
			glVertex3d(y[0],y[1],y[2]);
			
		}
	}

	glEnd();

	//WIREFFRAME + balls

	if (wireframe){
		glLineWidth(2);
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < length; j++) {
				// draw lines down and right
				Vector3f x = getState()[indexOf(j,i)];
				if (i < height-1) {
					Vector3f down = getState()[indexOf(j,i+1)];
					glVertex3f(x[0], x[1], x[2]);
					glVertex3f(down[0], down[1], down[2]);
				}
				if (j < length-1) {
					Vector3f right = getState()[indexOf(j+1,i)];
					glVertex3f(x[0], x[1], x[2]);
					glVertex3f(right[0], right[1], right[2]);
				}
			}
		}
		glEnd();
		glEnable(GL_LIGHTING);
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < length; j++) {
				Vector3f pos = getState()[indexOf(j,i)];
				glPushMatrix();
				glTranslatef(pos[0], pos[1], pos[2] );
				glutSolidSphere(0.025f,10.0f,10.0f);
				glPopMatrix();
			}
		}

	}
	glPushMatrix();
	glTranslatef(-1,3, 0);
	glutSolidSphere(0.05f, 10.0f, 10.0f);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(1.75,3, 0);
	glutSolidSphere(0.05f, 10.0f, 10.0f);
	glPopMatrix();
}

//NORMAL APPROXIMATOR (as much data as possible)
Vector3f ClothSystem::calcNormal(int j, int i){
	vector<Vector3f> state = getState();
	Vector3f x = getState()[indexOf(j,i)];
	Vector3f normal = Vector3f(0,0,0);

	if (indexOf(j-1,i)!=-1 && indexOf(j,i+1)!= -1)
	{
		Vector3f prt = Vector3f::cross(state[indexOf(j-1,i)] - x, state[indexOf(j,i+1)] - x).normalized();
		normal += prt;
	}
	if (indexOf(j,i+1)!=-1 && indexOf(j+1,i)!= -1)
	{
		Vector3f prt = Vector3f::cross(state[indexOf(j,i+1)] - x, state[indexOf(j+1,i)] - x).normalized();
		normal += prt;
	}
	if (indexOf(j+1,i)!=-1 && indexOf(j,i-1)!= -1)
	{
		Vector3f prt = Vector3f::cross(state[indexOf(j+1,i)] - x, state[indexOf(j,i-1)] - x).normalized();
		normal += prt;
	}
	if (indexOf(j,i-1)!=-1 && indexOf(j-1,i)!= -1)
	{
		Vector3f prt = Vector3f::cross(state[indexOf(j,i-1)] - x, state[indexOf(j-1,i)] - x).normalized();
		normal += prt;
	}
	return normal.normalized();
}

void ClothSystem::toggleWind(){
	windt = !windt;
}

void ClothSystem::toggleBall(){
	ball = !ball;
}

void ClothSystem::toggleWireframe(){
	wireframe = !wireframe;
}
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "test.h"

///TODO: include more headers if necessary

#define TwoPi (6.28318531f)
#define Pi (3.14159265f)
#define countof(A) (sizeof(A) / sizeof(A[0]))

using namespace std;



// Globals here.
namespace
{
	static GLuint QuadVao;
	static GLuint VisualizeProgram, VisualizeVProgram;
	static Slab Velocity, Density, Pressure, Temperature;

	static Surface Divergence, Obstacles, HiresObstacles, Vorticity;
	SliderInterface* test;

	GLfloat angle = 0.0f;
	int height = 10;
	int balls = 4;
	bool ball = true;
	int w = GridWidth;
    int h = GridHeight;
	int counter = 0;
	bool SmokeMode = true;
	bool MouseMode = false;
	bool MouseDown = false;
	int py = 0;
	int px = 0;
	int dx, dy;

  // initialize your particle systems
  ///TODO: read argv here. set timestepper , step size etc
  void initSystem(int argc, char * argv[])
  {
    // seed the random number generator with the current time
    srand( time( NULL ) );

  }

  // Take a step forward for the particle shower
  ///TODO: Optional. modify this function to display various particle systems
  ///and switch between different timeSteppers
  //------------------------------------------------------------------- 

    // Declarations of functions whose implementations occur later.
   
    void drawScene(void);
	void update();
    void initRendering();
	void mouseFunc(int button, int state, int x, int y);
    void motionFunc(int x, int y);
	void PassiveMouseMotion(int x, int y);
	void keyboardFunc(unsigned char key, int x, int y);
	void drawSceneDensity(void);
	void drawScenePressure(void);
	void drawSceneVelocity(void);

	// This function is called whenever a "Normal" key press is received.
	void keyboardFunc( unsigned char key, int x, int y )
	{
		switch ( key )
		{
		case 27: // Escape key
			exit(0);
			break;

		case 't':
			counter += 1;

			if (counter > 2)
			{
				counter = 0;
			}
			break;

		case 'r':
			SmokeMode = !SmokeMode;
			break;

		case 'm':
			MouseMode = !MouseMode;

		default:
			cout << "Unhandled key press " << key << "." << endl;        
		}

	}

	void mouseFunc(int button, int state, int x, int y)
    {
        if ((state == GLUT_DOWN) && (button == GLUT_LEFT_BUTTON))
        { 
           MouseDown = true;
        }
        if ((state == GLUT_UP) && (button == GLUT_LEFT_BUTTON))
        {
			MouseDown = false;
        }
    }

    // Called when mouse is moved while button pressed.
    void motionFunc(int x, int y)
    {
		dx = x - px;
		dy = y - py;

        ImpulsePosition.X = (x)/2;
		ImpulsePosition.Y = (800-y)/2;

		px = x;
		py = y;
        //glutPostRedisplay();
    }

	void PassiveMouseMotion( int x, int y)
	{
		px = x;
		py = y;
	}
    // Called when the window is resized
    // w, h - width and height of the window in pixels.

    // Initialize OpenGL's rendering modes
	void update()
	{
		SmokeWeight = test->getSliderValue();
		glViewport(0, 0, GridWidth, GridHeight);

		Advect(Velocity.Ping, Velocity.Ping, Obstacles, Velocity.Pong, VelocityDissipation);
		SwapSurfaces(&Velocity);

		Advect(Velocity.Ping, Temperature.Ping, Obstacles, Temperature.Pong, TemperatureDissipation);
		SwapSurfaces(&Temperature);

		Advect(Velocity.Ping, Density.Ping, Obstacles, Density.Pong, DensityDissipation);
		SwapSurfaces(&Density);

		ApplyImpulse(Temperature.Ping, ImpulsePosition, ImpulseTemperature,ImpulseTemperature,ImpulseTemperature);
		
		if (!SmokeMode) 
		{
			if (MouseMode)	
			{
				if (MouseDown) {ApplyImpulse(Velocity.Ping, ImpulsePosition, (float)dx*5.50, -(float)dy*5.50,0.0);
				ApplyImpulse(Density.Ping, ImpulsePosition, ImpulseDensity,ImpulseDensity,ImpulseDensity);}
			} else 
			{
					ApplyImpulse(Density.Ping, ImpulsePosition, ImpulseDensity,ImpulseDensity,ImpulseDensity);		
					ApplyImpulse(Velocity.Ping, ImpulsePosition, 0.0,40.0,0.0);
			}
		} else ApplyImpulse(Density.Ping, ImpulsePosition, ImpulseDensity,ImpulseDensity,ImpulseDensity);

		for (int i = 1; i < 1; i++){
		//Vector2 ip2 = { ImpulsePosition.X + 10.0 * i, ImpulsePosition.Y };
		
		//ApplyImpulse(Temperature.Ping, ip2, ImpulseTemperature);
		//ApplyImpulse(Density.Ping, ip2, ImpulseDensity);

		//Vector2 ip3 = { ImpulsePosition.X - 10.0 * i, ImpulsePosition.Y };

		//ApplyImpulse(Temperature.Ping, ip3, ImpulseTemperature,ImpulseTemperature,ImpulseTemperature);
		//ApplyImpulse(Density.Ping, ip3, ImpulseDensity,ImpulseDensity,ImpulseDensity);
		}

		if (SmokeMode){
		ApplyBuoyancy(Velocity.Ping, Temperature.Ping, Density.Ping, Velocity.Pong);
		SwapSurfaces(&Velocity);
		}

		ComputeVorticity(Velocity.Ping, Obstacles, Vorticity);
		ComputeVortForce(Vorticity, Velocity.Ping, Obstacles, Velocity.Pong);
		SwapSurfaces(&Velocity);
    
		ComputeDivergence(Velocity.Ping, Obstacles, Divergence);
		ClearSurface(Pressure.Ping, 0);

		for (int i = 0; i < NumJacobiIterations; ++i) {
			Jacobi(Pressure.Ping, Divergence, Obstacles, Pressure.Pong);
			SwapSurfaces(&Pressure);
			}

		SubtractGradient(Velocity.Ping, Pressure.Ping, Obstacles, Velocity.Pong);
		SwapSurfaces(&Velocity);

		
	}


    void initRendering()
    {
		//glEnable(GL_TEXTURE_2D);

        glClearColor(0,0,0,1);

		Velocity = CreateSlab(w, h, 2);
		Density = CreateSlab(w, h, 1);
		Pressure = CreateSlab(w, h, 1);
		Temperature = CreateSlab(w, h, 1);
		Vorticity = CreateSurface(w, h, 1);
		Divergence = CreateSurface(w, h, 3);
		InitSlabOps(&SmokeWeight);
		VisualizeProgram = CreateProgram("Fluid.Vertex", 0, "Fluid.VisualizeObs");
		VisualizeVProgram = CreateProgram("Fluid.Vertex", 0, "Fluid.Visualize");

		Obstacles = CreateSurface(w, h, 3);
		float m = 6; //test->getMSliderValue();
		float p = 7; //test->getPSliderValue();
		float t = 21;//test->getTSliderValue();
		CreateObstacles(Obstacles, w, h,m,p,t);

		w = ViewportWidth * 2;
		h = ViewportHeight * 2;
		HiresObstacles = CreateSurface(w, h, 1);
		CreateObstacles(HiresObstacles, w, h, m, p , t);

		QuadVao = CreateQuad();
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		ClearSurface(Temperature.Ping, AmbientTemperature);

    }

    // This function is responsible for displaying the object.
    void drawScene(void)
    {
		switch ( counter )
		{
		case 0: // Escape key
			drawSceneDensity();
			break;

		case 1:
			drawScenePressure();
			break;
		
		case 2:
			drawSceneVelocity();
			break;

		default:
			break;     
		}
		
    }

	void drawSceneDensity(){
		glUseProgram(VisualizeProgram);
		GLint fillColor = glGetUniformLocation(VisualizeProgram, "FillColor");
		GLint scale = glGetUniformLocation(VisualizeProgram, "Scale");
		glEnable(GL_BLEND);

		// Set render target to the backbuffer:
		glViewport(0, 0, ViewportWidth, ViewportHeight);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);

		// Draw ink:
		glBindTexture(GL_TEXTURE_2D, Density.Ping.TextureHandle);
		if (SmokeMode) glUniform3f(fillColor, 1, 1, 0.9);
		else glUniform3f(fillColor, 0.8,0.8,1.0);
		glUniform2f(scale, 1.0f / ViewportWidth, 1.0f / ViewportHeight);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
		// Draw obstacles:
		glBindTexture(GL_TEXTURE_2D, HiresObstacles.TextureHandle);
		glUniform3f(fillColor, 0.125f, 0.4f, 0.75f);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
		// Disable blending:
		glDisable(GL_BLEND);
        // Dump the image to the screen.
        glutSwapBuffers();

	}

	void drawScenePressure(){
		glUseProgram(VisualizeProgram);
		GLint fillColor = glGetUniformLocation(VisualizeProgram, "FillColor");
		GLint scale = glGetUniformLocation(VisualizeProgram, "Scale");
		glEnable(GL_BLEND);

		// Set render target to the backbuffer:
		glViewport(0, 0, ViewportWidth, ViewportHeight);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);

		// Draw ink:
		glBindTexture(GL_TEXTURE_2D, Pressure.Ping.TextureHandle);
		glUniform3f(fillColor, 0.75, 0.8, 0.9);
		glUniform2f(scale, 1.0f / ViewportWidth, 1.0f / ViewportHeight);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
		// Draw obstacles:
		glBindTexture(GL_TEXTURE_2D, HiresObstacles.TextureHandle);
		glUniform3f(fillColor, 0.125f, 0.4f, 0.75f);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
		// Disable blending:
		glDisable(GL_BLEND);
        // Dump the image to the screen.
        glutSwapBuffers();

	}

	void drawSceneVelocity(){
		glUseProgram(VisualizeVProgram);
		GLint fillColor = glGetUniformLocation(VisualizeProgram, "FillColor");
		GLint scale = glGetUniformLocation(VisualizeProgram, "Scale");
		glEnable(GL_BLEND);

		// Set render target to the backbuffer:
		glViewport(0, 0, ViewportWidth, ViewportHeight);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);

		// Draw ink:
		glBindTexture(GL_TEXTURE_2D, Velocity.Ping.TextureHandle);
		glUniform3f(fillColor, 0.0, 0.0, 0.0);
		glUniform2f(scale, 1.0f / ViewportWidth, 1.0f / ViewportHeight);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
		// Draw obstacles:
		glBindTexture(GL_TEXTURE_2D, HiresObstacles.TextureHandle);
		glUniform3f(fillColor, 0.825f, 0.9f, 0.75f);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
		// Disable blending:
		glDisable(GL_BLEND);
        // Dump the image to the screen.
        glutSwapBuffers();

	}

    void timerFunc(int t)
	{
		update();
        glutPostRedisplay();
        glutTimerFunc(t, &timerFunc, t);
    }
    
}

// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
int main( int argc, char* argv[] )
{
	glutInit( &argc, argv );

    // We're going to animate it, so double buffer 
    glutInitDisplayMode( GLUT_DOUBLE );

    // Initial parameters for window position and size
    glutInitWindowPosition( 60, 60 );
    glutInitWindowSize( ViewportWidth, ViewportHeight );
    
    glutCreateWindow("Assignment 4");

	GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        cout << "GLEW Error: %s\n", glewGetErrorString(err);
    }

    // Initialize OpenGL parameters.
    initRendering();

	SliderInterface* test = new SliderInterface();
	test->make_window();

    // Setup particle system

    // Set up callback functions for key presses
    glutKeyboardFunc(keyboardFunc); // Handles "normal" ascii symbols
    //glutSpecialFunc(specialFunc);   // Handles "special" keyboard keys

    //// Set up callback functions for mouse
    glutMouseFunc(mouseFunc);
    glutMotionFunc(motionFunc);
	glutPassiveMotionFunc( PassiveMouseMotion );

    //// Set up the callback function for resizing windows
    //glutReshapeFunc( reshapeFunc );

    // Call this whenever window needs redrawing
    glutDisplayFunc( drawScene );

    // Trigger timerFunc every 20 msec
    glutTimerFunc(20, timerFunc, 20);

    // Start the main loop.  glutMainLoop never returns.
    glutMainLoop();

    return 0;	// This line is never reached.
}

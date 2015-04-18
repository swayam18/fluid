#include "main.h"
#include <math.h>
#include <stdio.h>

float thickness(float x, float t, float c) {
	float yt = 5 * t * c *(
		 0.2969 * sqrtf(x / c) +
		-0.1260 * (x / c) +
		-0.3516 * (x * x) / (c * c) +
		+0.2843 * (x * x* x) / (c * c * c) +
		-0.1015 * (x * x * x * x) / (c * c * c * c)
		);
	return yt;
}
void getAirfoilVeritcal(float* positions, const int slices, int width, int height) {
	// number of slices determines the number of trapezoids to produces...
	float t = 0.30;
	float c = 1.0;
	float delx = c / ((float)slices);
	float x = 0.0;
	float* pPositions = &positions[0];
	for (int i = 0; i < slices; i++) {
		float x0 = x;
		float x1 = x0 + delx;

		float yt0 = thickness(x0, t, c);
		float yt1 = thickness(x1, t, c);

		//triangle 1
		*pPositions++ = 0.0f;
		*pPositions++ = x0;

		*pPositions++ = -yt0;
		*pPositions++ = x0;

		*pPositions++ = 0.0f;
		*pPositions++ = x1;

		//triangle 2
		*pPositions++ = 0.0f;
		*pPositions++ = x1;

		*pPositions++ = -yt1;
		*pPositions++ = x1;

		*pPositions++ = -yt0;
		*pPositions++ = x0;

		//triangle 3
		*pPositions++ = 0.0f;
		*pPositions++ = x0;

		*pPositions++ = yt0;
		*pPositions++ = x0;

		*pPositions++ = 0.0f;
		*pPositions++ = x1;

		//triangle 4
		*pPositions++ = 0.0f;
		*pPositions++ = x1;

		*pPositions++ = yt1;
		*pPositions++ = x1;

		*pPositions++ = yt0;
		*pPositions++ = x0;

		x = x1;
	}

}

void getAirfoil(float* positions, const int slices, int width, int height) {
	// number of slices determines the number of trapezoids to produces...
	float t = 0.18;
	float c = 1.0;
	float delx = c / ((float)slices);
	float x = 0.0;
	float* pPositions = &positions[0];
	for (int i = 0; i < slices; i++) {
		float x0 = x;
		float x1 = x0 + delx;

		float yt0 = thickness(x0, t, c);
		float yt1 = thickness(x1, t, c);

		//triangle 1
		*pPositions++ = x0;
		*pPositions++ = 0.0f;

		*pPositions++ = x0;
		*pPositions++ = -yt0;

		*pPositions++ = x1;
		*pPositions++ = 0.0f;

		//triangle 2
		*pPositions++ = x1;
		*pPositions++ = 0.0f;

		*pPositions++ = x1;
		*pPositions++ = -yt1;

		*pPositions++ = x0;
		*pPositions++ = -yt0;

		//triangle 3
		*pPositions++ = x0;
		*pPositions++ = 0.0f;

		*pPositions++ = x0;
		*pPositions++ = yt0;

		*pPositions++ = x1;
		*pPositions++ = 0.0f;

		//triangle 4
		*pPositions++ = x1;
		*pPositions++ = 0.0f;

		*pPositions++ = x1;
		*pPositions++ = yt1;

		*pPositions++ = x0;
		*pPositions++ = yt0;

		x = x1;
	}
}

void getCircle(float* positions, const int slices) {

	float twopi = 6.28318531;
	float theta = 0;
	float dtheta = twopi / (float)(slices - 1);
	float* pPositions = &positions[0];
	for (int i = 0; i < slices; i++) {
		*pPositions++ = 0;
		*pPositions++ = 0;

		*pPositions++ = 0.25f * cos(theta) ;
		*pPositions++ = 0.25f * sin(theta);
		theta += dtheta;

		*pPositions++ = 0.25f * cos(theta);
		*pPositions++ = 0.25f * sin(theta);
	}
}

void CreateObstacles(Surface dest, int width, int height)
{
    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
    glViewport(0, 0, width, height);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    GLuint program = CreateProgram("Fluid.Vertex", 0, "Fluid.Fill");
    glUseProgram(program);

    const int DrawBorder = 1;
    if (DrawBorder) {
        #define T 0.9999f
        float positions[] = { -T, -T, T, -T, T,  T, -T,  T, -T, -T };
        #undef T
        GLuint vbo;
        GLsizeiptr size = sizeof(positions);
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, size, positions, GL_STATIC_DRAW);
        GLsizeiptr stride = 2 * sizeof(positions[0]);
        glEnableVertexAttribArray(PositionSlot);
        glVertexAttribPointer(PositionSlot, 2, GL_FLOAT, GL_FALSE, stride, 0);
        glDrawArrays(GL_LINE_STRIP, 0, 5);
        glDeleteBuffers(1, &vbo);
    }

    const int DrawCircle = 1;
    if (DrawCircle) {
        const int slices = 64;
        float positions[slices*2*3*4];
		getAirfoilVeritcal(positions, slices, width, height);
        GLuint vbo;
        GLsizeiptr size = sizeof(positions);
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, size, positions, GL_STATIC_DRAW);
        GLsizeiptr stride = 2 * sizeof(positions[0]);
        glEnableVertexAttribArray(PositionSlot);
        glVertexAttribPointer(PositionSlot, 2, GL_FLOAT, GL_FALSE, stride, 0);
        glDrawArrays(GL_TRIANGLES, 0, slices * 3 * 4);
        glDeleteBuffers(1, &vbo);
    }

    // Cleanup
    glDeleteProgram(program);
    glDeleteVertexArrays(1, &vao);

}

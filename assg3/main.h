#define GLEW_STATIC
#include "lib/glew/glew.h"
#ifdef _WIN32
#include "GL/freeglut.h"
#else
#include <GL/glut.h>
#endif
typedef struct Surface_ {
    GLuint FboHandle;
    GLuint TextureHandle;
    int NumComponents;
} Surface;

typedef struct Slab_ {
    Surface Ping;
    Surface Pong;
} Slab;

typedef struct Vector2_ {
    int X;
    int Y;
} Vector2;

#define CellSize (1.25f)
#define ViewportWidth (800)
#define ViewportHeight (800)
#define GridWidth (ViewportWidth / 2)
#define GridHeight (ViewportHeight / 2)
#define SplatRadius ((float) 50.0f)

static const float AmbientTemperature = 0.0f;
static const float ImpulseTemperature = 10.0f;
static const float ImpulseDensity = 1.0f;
static const int NumJacobiIterations = 40;
static float TimeStep = 0.125f;
static float SmokeBuoyancy = 1.1f;
static float SmokeWeight = 0.06f;
static float GradientScale = 1.0f / CellSize;
static float TemperatureDissipation = 0.99f;
static float VelocityDissipation = 0.995f;
static float DensityDissipation = 0.9999f;
static Vector2 ImpulsePosition = { GridWidth / 2, - (int) SplatRadius / 2 + 30};

static const int PositionSlot = 0;

GLuint CreateQuad();
GLuint CreateProgram(const char* vsKey, const char* gsKey, const char* fsKey);
Surface CreateSurface(GLsizei width, GLsizei height, int numComponents);
Slab CreateSlab(GLsizei width, GLsizei height, int numComponents);
void CreateObstacles(Surface dest, int width, int height);
void InitSlabOps();
void SwapSurfaces(Slab* slab);
void ClearSurface(Surface s, float value);
void Advect(Surface velocity, Surface source, Surface obstacles, Surface dest, float dissipation);
void Jacobi(Surface pressure, Surface divergence, Surface obstacles, Surface dest);
void SubtractGradient(Surface velocity, Surface pressure, Surface obstacles, Surface dest);
void ComputeDivergence(Surface velocity, Surface obstacles, Surface dest);
void ApplyImpulse(Surface dest, Vector2 position, float value);
void ApplyBuoyancy(Surface velocity, Surface temperature, Surface density, Surface dest);

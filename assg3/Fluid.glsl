
-- Vertex

in vec4 Position;

void main()
{
    gl_Position = Position;
}

-- Fill

out vec3 FragColor;

void main()
{
    FragColor = vec3(1, 0, 0);
}

-- Advect

out vec4 FragColor;

uniform sampler2D VelocityTexture;
uniform sampler2D SourceTexture;
uniform sampler2D Obstacles;

uniform vec2 InverseSize;
uniform float TimeStep;
uniform float Dissipation;

void main()
{
    vec2 fragCoord = gl_FragCoord.xy;

    vec2 u = texture(VelocityTexture, InverseSize * fragCoord).xy;
    vec2 coord = InverseSize * (fragCoord - TimeStep * u);

    FragColor = Dissipation * texture(SourceTexture, coord);
	
}

-- Jacobi

out vec4 FragColor;

uniform sampler2D Pressure;
uniform sampler2D Divergence;
uniform sampler2D Obstacles;

uniform float Alpha;
uniform float InverseBeta;

void main()
{
    ivec2 T = ivec2(gl_FragCoord.xy);

    // Find neighboring pressure:
    vec4 pN = texelFetchOffset(Pressure, T, 0, ivec2(0, 1));
    vec4 pS = texelFetchOffset(Pressure, T, 0, ivec2(0, -1));
    vec4 pE = texelFetchOffset(Pressure, T, 0, ivec2(1, 0));
    vec4 pW = texelFetchOffset(Pressure, T, 0, ivec2(-1, 0));
    vec4 pC = texelFetch(Pressure, T, 0);

    // Find neighboring obstacles:
    vec3 oN = texelFetchOffset(Obstacles, T, 0, ivec2(0, 1)).xyz;
    vec3 oS = texelFetchOffset(Obstacles, T, 0, ivec2(0, -1)).xyz;
    vec3 oE = texelFetchOffset(Obstacles, T, 0, ivec2(1, 0)).xyz;
    vec3 oW = texelFetchOffset(Obstacles, T, 0, ivec2(-1, 0)).xyz;

    // Use center pressure for solid cells:
    if (oN.x > 0) pN = pC;
    if (oS.x > 0) pS = pC;
    if (oE.x > 0) pE = pC;
    if (oW.x > 0) pW = pC;

    vec4 bC = texelFetch(Divergence, T, 0);

    FragColor = (pW + pE + pS + pN + Alpha * bC) * InverseBeta;
}

-- SubtractGradient

out vec2 FragColor;

uniform sampler2D Velocity;
uniform sampler2D Pressure;
uniform sampler2D Obstacles;
uniform float GradientScale;

void main()
{
    ivec2 T = ivec2(gl_FragCoord.xy);

    vec3 oC = texelFetch(Obstacles, T, 0).xyz;
    if (oC.x > 0) {
        FragColor = oC.yz;
        return;
    }

    // Find neighboring pressure:
    float pN = texelFetchOffset(Pressure, T, 0, ivec2(0, 1)).r;
    float pS = texelFetchOffset(Pressure, T, 0, ivec2(0, -1)).r;
    float pE = texelFetchOffset(Pressure, T, 0, ivec2(1, 0)).r;
    float pW = texelFetchOffset(Pressure, T, 0, ivec2(-1, 0)).r;
    float pC = texelFetch(Pressure, T, 0).r;

    // Find neighboring obstacles:
    vec3 oN = texelFetchOffset(Obstacles, T, 0, ivec2(0, 1)).xyz;
    vec3 oS = texelFetchOffset(Obstacles, T, 0, ivec2(0, -1)).xyz;
    vec3 oE = texelFetchOffset(Obstacles, T, 0, ivec2(1, 0)).xyz;
    vec3 oW = texelFetchOffset(Obstacles, T, 0, ivec2(-1, 0)).xyz;

    // Use center pressure for solid cells:
    vec2 obstV = vec2(0);
    vec2 vMask = vec2(1);

    if (oN.x > 0) { pN = pC; obstV.y = oN.z; vMask.y = 0; }
    if (oS.x > 0) { pS = pC; obstV.y = oS.z; vMask.y = 0; }
    if (oE.x > 0) { pE = pC; obstV.x = oE.y; vMask.x = 0; }
    if (oW.x > 0) { pW = pC; obstV.x = oW.y; vMask.x = 0; }

    // Enforce the free-slip boundary condition:
    vec2 oldV = texelFetch(Velocity, T, 0).xy;
    vec2 grad = vec2(pE - pW, pN - pS) * GradientScale;
    vec2 newV = oldV - grad;
    FragColor = (vMask * newV) + obstV;  
}

-- ComputeDivergence

out float FragColor;

uniform sampler2D Velocity;
uniform sampler2D Obstacles;
uniform float HalfInverseCellSize;

void main()
{
    ivec2 T = ivec2(gl_FragCoord.xy);

    // Find neighboring velocities:
    vec2 vN = texelFetchOffset(Velocity, T, 0, ivec2(0, 1)).xy;
    vec2 vS = texelFetchOffset(Velocity, T, 0, ivec2(0, -1)).xy;
    vec2 vE = texelFetchOffset(Velocity, T, 0, ivec2(1, 0)).xy;
    vec2 vW = texelFetchOffset(Velocity, T, 0, ivec2(-1, 0)).xy;

    // Find neighboring obstacles:
    vec3 oN = texelFetchOffset(Obstacles, T, 0, ivec2(0, 1)).xyz;
    vec3 oS = texelFetchOffset(Obstacles, T, 0, ivec2(0, -1)).xyz;
    vec3 oE = texelFetchOffset(Obstacles, T, 0, ivec2(1, 0)).xyz;
    vec3 oW = texelFetchOffset(Obstacles, T, 0, ivec2(-1, 0)).xyz;

    // Use obstacle velocities for solid cells:
    if (oN.x > 0) vN = oN.yz;
    if (oS.x > 0) vS = oS.yz;
    if (oE.x > 0) vE = oE.yz;
    if (oW.x > 0) vW = oW.yz;

    FragColor = HalfInverseCellSize * (vE.x - vW.x + vN.y - vS.y);
}

-- Splat

out vec4 FragColor;

uniform vec2 Point;
uniform float Radius;
uniform vec3 FillColor;

void main()
{
    float d = distance(Point, gl_FragCoord.xy);
    if (d < Radius) {
        float a = (Radius - d) * 0.5;
        a = min(a, 1.0);
        FragColor = vec4(FillColor, a);
    } else {
        FragColor = vec4(0);
    }
}

-- Buoyancy

out vec2 FragColor;
uniform sampler2D Velocity;
uniform sampler2D Temperature;
uniform sampler2D Density;
uniform float AmbientTemperature;
uniform float TimeStep;
uniform float Sigma;
uniform float Kappa;

void main()
{
    ivec2 TC = ivec2(gl_FragCoord.xy);
    float T = texelFetch(Temperature, TC, 0).r;
    vec2 V = texelFetch(Velocity, TC, 0).xy;

    FragColor = V;

    if (T > AmbientTemperature) {
        float D = texelFetch(Density, TC, 0).x;
        FragColor += (TimeStep * (T - AmbientTemperature) * Sigma - D * Kappa ) * vec2(0, 1);
    }
}

-- Visualize

out vec4 FragColor;
uniform sampler2D Sampler;
uniform vec3 FillColor;
uniform vec2 Scale;

void main()
{
	float R = sqrt(texture(Sampler, gl_FragCoord.xy * Scale).r*texture(Sampler, gl_FragCoord.xy * Scale).r + texture(Sampler, gl_FragCoord.xy * Scale).g*texture(Sampler, gl_FragCoord.xy * Scale).g);
	float Red = texture(Sampler, gl_FragCoord.xy * Scale).r;
	float Blue = texture(Sampler, gl_FragCoord.xy * Scale).g;

	vec2 tmp = vec2(Red,Blue);
	vec2 vec = normalize(tmp);

	vec2 vecR = normalize(vec2(1,0.577));
    vec2 vecG = normalize(vec2(-1,0.577));
    vec2 vecB = normalize(vec2(0,-1));   

    float r   = (1.73205-length(vec-vecR))/1.73205;
    float g   = (1.73205-length(vec-vecG))/1.73205;
    float b   = (1.73205-length(vec-vecB))/1.73205;

	vec3 col = normalize(vec3(r,b,g));

	if (FillColor.r == 0.0) FragColor = vec4(col, R*0.4);
	else FragColor = vec4(FillColor, R);
}

-- VisualizeObs

out vec4 FragColor;
uniform sampler2D Sampler;
uniform vec3 FillColor;
uniform vec2 Scale;

void main()
{
    float L = texture(Sampler, gl_FragCoord.xy * Scale).r;
    FragColor = vec4(FillColor, L);
}

-- ComputeVorticity

out float FragColor;

uniform sampler2D Velocity;
uniform sampler2D Obstacles;
uniform float HalfInverseCellSize;

void main()
{
    ivec2 T = ivec2(gl_FragCoord.xy);

    // Find neighboring velocities:
    vec2 vN = texelFetchOffset(Velocity, T, 0, ivec2(0, 1)).xy;
    vec2 vS = texelFetchOffset(Velocity, T, 0, ivec2(0, -1)).xy;
    vec2 vE = texelFetchOffset(Velocity, T, 0, ivec2(1, 0)).xy;
    vec2 vW = texelFetchOffset(Velocity, T, 0, ivec2(-1, 0)).xy;

    // Find neighboring obstacles:
    vec3 oN = texelFetchOffset(Obstacles, T, 0, ivec2(0, 1)).xyz;
    vec3 oS = texelFetchOffset(Obstacles, T, 0, ivec2(0, -1)).xyz;
    vec3 oE = texelFetchOffset(Obstacles, T, 0, ivec2(1, 0)).xyz;
    vec3 oW = texelFetchOffset(Obstacles, T, 0, ivec2(-1, 0)).xyz;

    // Use obstacle velocities for solid cells:
    if (oN.x > 0) vN = oN.yz;
    if (oS.x > 0) vS = oS.yz;
    if (oE.x > 0) vE = oE.yz;
    if (oW.x > 0) vW = oW.yz;

    FragColor = HalfInverseCellSize * ((vE.y - vW.y) - (vN.x - vS.x));
}

-- ComputeVortForce

out vec2 FragColor;

uniform sampler2D VorticityTexture;
uniform sampler2D SourceTexture;
uniform sampler2D Obstacles;

uniform float HalfInverseCellSize;
uniform float TimeStep;
uniform vec2 dxscale; //Vorticity Confinement

void main()
{
    ivec2 T = ivec2(gl_FragCoord.xy);

	   // Find neighboring velocities:
    float vN = texelFetchOffset(VorticityTexture, T, 0, ivec2(0, 1)).r;
    float vS = texelFetchOffset(VorticityTexture, T, 0, ivec2(0, -1)).r;
    float vE = texelFetchOffset(VorticityTexture, T, 0, ivec2(1, 0)).r;
    float vW = texelFetchOffset(VorticityTexture, T, 0, ivec2(-1, 0)).r;
	float vC = texelFetch(VorticityTexture, T, 0).r;

	vec2 force = HalfInverseCellSize * vec2(abs(vN) - abs(vS), abs(vE) - abs(vW));

	//safe normalize
	float EPSILON = 0.00024414;
	float magSqr = max(EPSILON, dot(force, force));
	force = force * inversesqrt(magSqr);

	force = force * dxscale * vC * vec2(1.0,-1.0);

	vec2 V = texelFetch(SourceTexture, T, 0).xy;
    FragColor = TimeStep * force + V;
	
}
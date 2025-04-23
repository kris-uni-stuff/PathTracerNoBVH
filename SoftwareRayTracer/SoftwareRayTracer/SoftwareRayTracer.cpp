#define _USE_MATH_DEFINES 
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>


#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "bmp.h"
#include "obj.h"
#include "verts.h"
#include "BVH.h"
#include "timer.h"

#define PIXEL_W 1920
#define PIXEL_H 1080



glm::vec3 bkgd = glm::vec3(1.f, 1.f, 1.f);

const float atten = 1.f;

int max_recursion_depth = 1;


int samples_per_pixel = 32;
int tracedSegments = 4;


std::vector<triangle> tris;
float pixelBuffer[PIXEL_W * PIXEL_H * 3];

glm::vec3 eye = glm::vec3(-1.f, 2.5f, 3.0f);
float vfov = glm::radians(60.f);
float aspect = (float)PIXEL_W / (float)PIXEL_H;

glm::vec3 light_pos(1.f, 2.f, 1.f);

void TracePathThroughTriangles(glm::vec3 o, glm::vec3 dir, glm::vec3& ioCol, int depth);


vec3 skyColour(vec3 rayDir)
{
    vec3 colour;
    // +y in world space is up, so:
    if (rayDir.y > 0.0f)
    {
        colour = mix(vec3(1.0f), vec3(0.25f, 0.5f, 1.0f), rayDir.y);
    }
    else
    {
        colour = vec3(0.03f);
    }
    return colour;
}

vec3 GetTriangleColour(triangle* tri)
{
    vec3 colour(0.7f);


    if (tri->primitiveID == 6 || tri->primitiveID == 7)
        colour = vec3(1, 0, 0);
    if (tri->primitiveID == 8 || tri->primitiveID == 9)
        colour = vec3(0, 1, 0);
    if (tri->primitiveID == 10 || tri->primitiveID == 11)
        colour = vec3(0, 0, 1);

    //    if (tri->primitiveID == 0)
    //        colour = vec3(1, 0, 0);
    //    if (tri->primitiveID == 1)
    //        colour = vec3(0, 1, 0);

    return colour;
}


vec3 GetRandomRayDirection(float px, float py, int W, int H, float aspect_ratio, float fov)
{
    float randx = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - .5f;
    vec3 X = aspect_ratio * fov * ((2 * (px + randx)) / W - 1) * vec3(1, 0, 0);
    float randy = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - .5f;
    vec3 Y = fov * ((2 * (py + randy)) / H - 1) * vec3(0, 1, 0);
    vec3 Z = vec3(0, 0, 1);
    return X + Y + Z;
}


void GetRandomRayHemisphere(triangle *tri, vec3 dir, vec3 p, Sray* ray)
{
    vec3 worldNrm = tri->v1.nor;

    vec3 offset = worldNrm;
    offset *= (0.0001 * sign(dot(dir, worldNrm)));
    vec3 origin = p - offset;
    ray->origin = origin;


    // For a random diffuse bounce direction, we follow the approach of
    // Ray Tracing in One Weekend, and generate a random point on a sphere
    // of radius 1 centered at the normal. This uses the random_unit_vector
    // function from chapter 8.5:
    const float theta = 6.2831853 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);  // Random in [0, 2pi]
    const float u = 2.0 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 1.0;  // Random in [-1, 1]
    const float r = sqrt(1.0 - u * u);
    vec3 rayDir = worldNrm + vec3(r * cos(theta), r * sin(theta), u);
    // Then normalize the ray direction:
    rayDir = normalize(rayDir);

    //    rayDir = worldNrm + random_unit_vector(); 

    ray->dir = rayDir;
    //prd->hitNormal = worldNrm;
}


glm::vec3 Shade(triangle* tri, int depth, glm::vec3 p, glm::vec3 dir)
{
//    prd->rayHitSky = 0;

    Sray ray;
    GetRandomRayHemisphere(tri, dir, p, &ray);

    vec3 tri_colour = GetTriangleColour(tri);

    vec3 col;
    TracePathThroughTriangles(ray.origin, ray.dir, col, depth+1);
    tri_colour *= col;

    return tri_colour;
}




bool PointInTriangle(glm::vec3 pt, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
{
    glm::vec3 V12 = v2 - v1;
    glm::vec3 V13 = v3 - v1;
    glm::vec3 V1p = pt - v1;
    glm::vec3 C11 = cross(V12, V1p);
    glm::vec3 C12 = cross(V12, V13);
    float d1 = dot(C11, C12);

    glm::vec3 V23 = v3 - v2;
    glm::vec3 V21 = v1 - v2;
    glm::vec3 V2p = pt - v2;
    glm::vec3 C21 = cross(V23, V2p);
    glm::vec3 C22 = cross(V23, V21);
    float d2 = dot(C21, C22);

    glm::vec3 V31 = v1 - v3;
    glm::vec3 V32 = v2 - v3;
    glm::vec3 V3p = pt - v3;
    glm::vec3 C31 = cross(V31, V3p);
    glm::vec3 C32 = cross(V31, V32);
    float d3 = dot(C31, C22);

    if (d1 > 0 && d2 > 0 && d3 > 0)
        return true;

    return false;
}


float RayTriangleIntersection(glm::vec3 o, glm::vec3 dir, triangle *tri, glm::vec3 &point)
{
    //on triangle plane?
    glm::vec3 V1 = tri->v2.pos - tri->v1.pos;
    glm::vec3 V2 = tri->v3.pos - tri->v1.pos;
    glm::vec3 n = cross(V2, V1);
    glm::vec3 N = normalize(n);

    glm::vec3 p = tri->v1.pos - o;
    float d1 = dot(p, N);
    float d2 = dot(dir, N);
    if (d2 == 0.f)
    {
        return FLT_MAX;
    }
    float t1 = d1 / d2;

    if (t1 < 0.001f)
        return FLT_MAX;

    glm::vec3 V3 = dir * t1;
    glm::vec3 pop = o + V3;


    //inside triangle?
    float alpha, beta, gamma;
    if (PointInTriangle(pop, tri->v1.pos, tri->v2.pos, tri->v3.pos))
    {
        point = pop;
        return t1;
    }

    return FLT_MAX;
}







void TracePathThroughTriangles(glm::vec3 o, glm::vec3 dir, glm::vec3 &ioCol, int depth)
{
    vec3 miss_col = skyColour(dir);
    vec3 black = vec3(0);

    if (depth == tracedSegments)
    {
        ioCol = black;
        return;
    }


    glm::vec3 p = glm::vec3(0);
    glm::vec3 closest_p = glm::vec3(0);
    float t = FLT_MAX;
    float closest_t = FLT_MAX;
    int closest_tri;
    //payload closest_prd;

    for (int tc=0; tc<tris.size(); tc++)
    {
            

        t = RayTriangleIntersection(o, dir, &tris[tc], p);
        if (t < closest_t)
        {
            closest_t = t;
            closest_tri = tc;
            closest_p = p;
        }
    }

    if (closest_t == FLT_MAX)
    {
        ioCol = miss_col;
//        io_prd->rayHitSky = 1;
        return;
    }

//    ioCol = Shade(&tris[closest_tri], depth, closest_p, dir);
    Sray ray;
    vec3 ray_col;
    GetRandomRayHemisphere(&tris[closest_tri], dir, closest_p, &ray);
    vec3 tri_colour = GetTriangleColour(&tris[closest_tri]);
    TracePathThroughTriangles(ray.origin, ray.dir, ray_col, depth + 1);
    tri_colour *= ray_col;
    ioCol = tri_colour;

  //  io_prd->rayOrigin = closest_prd.rayOrigin;
   // io_prd->rayDir = closest_prd.rayDir;
   // io_prd->rayHitSky = closest_prd.rayHitSky;
    return;

//    ioCol = miss_col;
//    return;
}

void writeColour(vec3 average_pixel_colour, int pixel_x, int pixel_y)
{
    auto r = linear_to_gamma(average_pixel_colour.x);
    auto g = linear_to_gamma(average_pixel_colour.y);
    auto b = linear_to_gamma(average_pixel_colour.z);


    float rc = std::clamp(r, .0f, 1.f);
    float gc = std::clamp(g, .0f, 1.f);
    float bc = std::clamp(b, .0f, 1.f);

    float pixel_r = rc * 255.f;
    float pixel_g = gc * 255.f;
    float pixel_b = bc * 255.f;

    pixelBuffer[(pixel_y * PIXEL_W * 3) + (pixel_x * 3) + 0] = pixel_r;
    pixelBuffer[(pixel_y * PIXEL_W * 3) + (pixel_x * 3) + 1] = pixel_g;
    pixelBuffer[(pixel_y * PIXEL_W * 3) + (pixel_x * 3) + 2] = pixel_b;

}

void PathTrace()
{
//    payload prd;
    for (int pixel_y = 0; pixel_y < PIXEL_H; ++pixel_y)
    {
        float percf = (float)pixel_y / (float)PIXEL_H;
        int perci = percf * 100;
        std::clog << "\rScanlines done: " << perci << "%" << ' ' << std::flush;


        for (int pixel_x = 0; pixel_x < PIXEL_W; ++pixel_x)
        {

            glm::vec3 summed_pixel_color(0, 0, 0);
            for (int sample = 0; sample < samples_per_pixel; sample++)
            {
                glm::vec3 pp = GetRandomRayDirection(pixel_x, pixel_y, PIXEL_W, PIXEL_H, aspect, vfov);

                vec3 origin = eye;

                glm::vec3 dir = normalize(pp);
                dir.y = -dir.y;
                dir.z = -dir.z;

                //vec3 accumulatedRayColor = vec3(1.0);  // The amount of light that made it to the end of the current ray.


                //for (int segments = 0; segments < tracedSegments; segments++)
                {

                    glm::vec3 ray_col(0.f);
                    TracePathThroughTriangles(origin, dir, ray_col, 0);
                    summed_pixel_color += ray_col;
                    //accumulatedRayColor *= col;

/*                    if (prd.rayHitSky == 1)
                    {
                        summed_pixel_color += accumulatedRayColor;
                        break;
                    }
                    else
                    {
                        origin = prd.rayOrigin;
                        dir = prd.rayDir;
                        prd.rayHitSky = 0;  // Will stop if light is hit
                    }*/
                }
            }

            vec3 average_pixel_colour = summed_pixel_color / float(samples_per_pixel);

            writeColour(average_pixel_colour, pixel_x, pixel_y);

        }
    }
    std::clog << "\rFinish rendering.           \n";

}



int main()
{

//    srand((unsigned int)time(NULL));

    LARGE_INTEGER Frequency;
    QueryPerformanceFrequency(&Frequency);

    const std::string MODEL_PATH = "objs/cornell2/cornell-box.obj";
   // const std::string MODEL_PATH = "objs/quad/quad.obj";

    obj_parse(MODEL_PATH.c_str(), &tris, 1.f);

    LARGE_INTEGER Render_StartingTime, Render_EndingTime;
    QueryPerformanceCounter(&Render_StartingTime);

    PathTrace();

    CounterEndAndPrint(Render_StartingTime, &Render_EndingTime, Frequency);

    savebitmap("render.bmp", pixelBuffer, PIXEL_W, PIXEL_H);

    return EXIT_SUCCESS;
}

#pragma once

#include <../glm/glm.hpp>

struct vec4
{
    float x, y, z, w;
};
/*
struct vec3
{
    float x, y, z;

    vec3() { x = 0.f; y = 0.f; z = 0.f; };
    vec3(float X, float Y, float Z) { x = X; y = Y; z = Z; };
};
*/
struct vec2
{
    float x, y;
};

struct mat4
{
    vec4 cols[4];
};

class vertex
{
public:
    glm::vec3 pos;
    glm::vec3 col;
    glm::vec3 nor;

/*    vertex() {}
    vertex(const vertex& v)
    {
        pos = v.pos;
        col = v.col;
        nor = v.nor;
    }
    vertex& operator=(vertex v)
    {
        pos = v.pos;
        col = v.col;
        nor = v.nor;
        return *this;
    }*/
};

class triangle
{
public:
    vertex v1, v2, v3;
    bool reflect;
    int primitiveID;
    /*
    triangle() {}
    triangle(const triangle& t)
    {
        v1 = t.v1;
        v2 = t.v2;
        v3 = t.v3;
        reflect = t.reflect;
    }
    triangle& operator=(triangle t)
    {
        v1 = t.v1;
        v2 = t.v2;
        v3 = t.v3;
        reflect = t.reflect;
        return *this;
    }*/
};
/*
float vec3len(vec3 in)
{
    float L = (in.x * in.x) + (in.y * in.y) + (in.z * in.z);
    L = sqrt(L);
    return L;
}


vec3 vec3div(vec3 in, float d)
{
    vec3 out;
    out.x = in.x / d;
    out.y = in.y / d;
    out.z = in.z / d;
    return out;
}

vec3 vec3mul(vec3 in, float d)
{
    vec3 out;
    out.x = in.x * d;
    out.y = in.y * d;
    out.z = in.z * d;
    return out;
}

vec3 vec3normalise(vec3 in)
{
    float L = vec3len(in);
    vec3 out = vec3div(in, L);
    return out;
}

vec3 vec3add(vec3 l, vec3 r)
{
    vec3 out;
    out.x = l.x + r.x;
    out.y = l.y + r.y;
    out.z = l.z + r.z;
    return out;
}

vec3 vec3sub(vec3 l, vec3 r)
{
    vec3 out;
    out.x = l.x - r.x;
    out.y = l.y - r.y;
    out.z = l.z - r.z;
    return out;
}

vec3 vec3cross(vec3 l, vec3 r)
{
    vec3 out;
    out.x = (l.y * r.z) - (l.z * r.y);
    out.y = (l.z * r.x) - (l.x * r.z);
    out.z = (l.x * r.y) - (l.y * r.x);
    return out;
}

float vec3dot(vec3 l, vec3 r)
{
    float out = (l.x * r.x) + (l.y * r.y) + (l.z * r.z);
    return out;
}

float vec4dot(vec4 l, vec4 r)
{
    float out = (l.x * r.x) + (l.y * r.y) + (l.z * r.z) + (l.w * r.w);
    return out;
}

vec3 vec3reflect(vec3 in, vec3 n)
{
    float d = vec3dot(in, n);
    vec3 m = vec3sub(n, in);
    vec3 v3 = vec3mul(m, d);
    vec3 w = vec3mul(v3, 2);
    return w;
}


vec4 mat4vec4mul(mat4 m, vec4 v)
{
    vec4 out;
    out.x = vec4dot({ m.cols[0].x, m.cols[1].x, m.cols[2].x, m.cols[3].x }, v);
    out.y = vec4dot({ m.cols[0].y, m.cols[1].y, m.cols[2].y, m.cols[3].y }, v);
    out.z = vec4dot({ m.cols[0].z, m.cols[1].z, m.cols[2].z, m.cols[3].z }, v);
    out.w = vec4dot({ m.cols[0].w, m.cols[1].w, m.cols[2].w, m.cols[3].w }, v);
    return out;
}

mat4 mat4trans(mat4 in)
{
    mat4 out;
    out.cols[0] = { in.cols[0].x, in.cols[1].x, in.cols[2].x, in.cols[3].x };
    out.cols[1] = { in.cols[0].y, in.cols[1].y, in.cols[2].y, in.cols[3].y };
    out.cols[2] = { in.cols[0].z, in.cols[1].z, in.cols[2].z, in.cols[3].z };
    out.cols[3] = { in.cols[0].w, in.cols[1].w, in.cols[2].w, in.cols[3].w };
    return out;
}

mat4 mat4mat4mul(mat4 l, mat4 r)
{
    mat4 out;
    out.cols[0].x = vec4dot({ l.cols[0].x, l.cols[1].x, l.cols[2].x, l.cols[3].x }, r.cols[0]);
    out.cols[0].y = vec4dot({ l.cols[0].y, l.cols[1].y, l.cols[2].y, l.cols[3].y }, r.cols[0]);
    out.cols[0].z = vec4dot({ l.cols[0].z, l.cols[1].z, l.cols[2].z, l.cols[3].z }, r.cols[0]);
    out.cols[0].w = vec4dot({ l.cols[0].w, l.cols[1].w, l.cols[2].w, l.cols[3].w }, r.cols[0]);

    out.cols[1].x = vec4dot({ l.cols[0].x, l.cols[1].x, l.cols[2].x, l.cols[3].x }, r.cols[1]);
    out.cols[1].y = vec4dot({ l.cols[0].y, l.cols[1].y, l.cols[2].y, l.cols[3].y }, r.cols[1]);
    out.cols[1].z = vec4dot({ l.cols[0].z, l.cols[1].z, l.cols[2].z, l.cols[3].z }, r.cols[1]);
    out.cols[1].w = vec4dot({ l.cols[0].w, l.cols[1].w, l.cols[2].w, l.cols[3].w }, r.cols[1]);

    out.cols[2].x = vec4dot({ l.cols[0].x, l.cols[1].x, l.cols[2].x, l.cols[3].x }, r.cols[2]);
    out.cols[2].y = vec4dot({ l.cols[0].y, l.cols[1].y, l.cols[2].y, l.cols[3].y }, r.cols[2]);
    out.cols[2].z = vec4dot({ l.cols[0].z, l.cols[1].z, l.cols[2].z, l.cols[3].z }, r.cols[2]);
    out.cols[2].w = vec4dot({ l.cols[0].w, l.cols[1].w, l.cols[2].w, l.cols[3].w }, r.cols[2]);

    out.cols[3].x = vec4dot({ l.cols[0].x, l.cols[1].x, l.cols[2].x, l.cols[3].x }, r.cols[3]);
    out.cols[3].y = vec4dot({ l.cols[0].y, l.cols[1].y, l.cols[2].y, l.cols[3].y }, r.cols[3]);
    out.cols[3].z = vec4dot({ l.cols[0].z, l.cols[1].z, l.cols[2].z, l.cols[3].z }, r.cols[3]);
    out.cols[3].w = vec4dot({ l.cols[0].w, l.cols[1].w, l.cols[2].w, l.cols[3].w }, r.cols[3]);
    return out;
}
*/
#define deg2rad(d) (d/180)*M_PI

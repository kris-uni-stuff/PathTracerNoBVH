#pragma once

void CounterEndAndPrint(LARGE_INTEGER StartingTime, LARGE_INTEGER* EndingTime, LARGE_INTEGER Frequency)
{
    QueryPerformanceCounter(EndingTime);

    LARGE_INTEGER ElapsedMicroseconds;
    ElapsedMicroseconds.QuadPart = (*EndingTime).QuadPart - StartingTime.QuadPart;
    ElapsedMicroseconds.QuadPart *= 1000000;
    ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
    std::cout << (float)ElapsedMicroseconds.QuadPart / (float)1000000 << std::endl;
}

vec3 random_unit_vector()
{
    while (true)
    {
        float randx = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        randx *= 2.f;
        randx -= 1.f;

        float randy = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        randy *= 2.f;
        randy -= 1.f;

        float randz = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        randz *= 2.f;
        randz -= 1.f;

        vec3 v = vec3(randx, randy, randz);
        if (!(v.x == 0.f || v.y == 0.f || v.z == 0.f))
            return normalize(v);
    }
}

struct payload
{
    vec3 colour;
    int  depth;
    int  rayHitSky;
    vec3 rayOrigin;
    vec3 rayDir;
    uint rngState;
    vec3 hitNormal;
};

#pragma once

int num_triangles_added = 0;

class AABB
{
public:
    glm::vec3 ftl;
    glm::vec3 bbr;
};

class BVH_node
{
public:
    AABB                bb;
    vector<BVH_node*>   child_bvhs;
    vector<triangle*>   tris;
};

typedef std::multimap<float, triangle*> TTriangleMap;

float GetLeftMostVertex(triangle t)
{
    std::map<float, glm::vec3> sorted_verts;
    sorted_verts[t.v1.pos.x] = t.v1.pos;
    sorted_verts[t.v2.pos.x] = t.v2.pos;
    sorted_verts[t.v3.pos.x] = t.v3.pos;
    auto it = sorted_verts.begin();
    return it->second.x;
}


float GetRightMostVertex(triangle t)
{
    std::map<float, glm::vec3> sorted_verts;
    sorted_verts[t.v1.pos.x] = t.v1.pos;
    sorted_verts[t.v2.pos.x] = t.v2.pos;
    sorted_verts[t.v3.pos.x] = t.v3.pos;
    auto it = sorted_verts.end();
    it--;
    return it->second.x;
}

float GetTopMostVertex(triangle t)
{
    std::map<float, glm::vec3> sorted_verts;
    sorted_verts[t.v1.pos.y] = t.v1.pos;
    sorted_verts[t.v2.pos.y] = t.v2.pos;
    sorted_verts[t.v3.pos.y] = t.v3.pos;
    auto it = sorted_verts.end();
    it--;
    return it->second.y;
}

float GetBottomMostVertex(triangle t)
{
    std::map<float, glm::vec3> sorted_verts;
    sorted_verts[t.v1.pos.y] = t.v1.pos;
    sorted_verts[t.v2.pos.y] = t.v2.pos;
    sorted_verts[t.v3.pos.y] = t.v3.pos;
    auto it = sorted_verts.begin();
    return it->second.y;
}

float GetNearMostVertex(triangle t)
{
    std::map<float, glm::vec3> sorted_verts;
    sorted_verts[t.v1.pos.z] = t.v1.pos;
    sorted_verts[t.v2.pos.z] = t.v2.pos;
    sorted_verts[t.v3.pos.z] = t.v3.pos;
    auto it = sorted_verts.end();
    it--;
    return it->second.z;
}
float GetFarMostVertex(triangle t)
{
    std::map<float, glm::vec3> sorted_verts;
    sorted_verts[t.v1.pos.z] = t.v1.pos;
    sorted_verts[t.v2.pos.z] = t.v2.pos;
    sorted_verts[t.v3.pos.z] = t.v3.pos;
    auto it = sorted_verts.begin();
    return it->second.z;
}


glm::vec3 GetCentroid(triangle* t)
{
    //  vec3 r;
    //  r.x = (t->v1.pos.x + t->v2.pos.x + t->v3.pos.x) / 3.f;
   //   r.y = (t->v1.pos.y + t->v2.pos.y + t->v3.pos.y) / 3.f;
  //    r.z = (t->v1.pos.z + t->v2.pos.z + t->v3.pos.z) / 3.f;
  //    return r;
    return glm::vec3((t->v1.pos.x + t->v2.pos.x + t->v3.pos.x) / 3.f, (t->v1.pos.y + t->v2.pos.y + t->v3.pos.y) / 3.f, (t->v1.pos.z + t->v2.pos.z + t->v3.pos.z) / 3.f);
}

glm::vec3 GetLeftMostTrianglePosition(TTriangleMap tris)
{
    float leftmost = FLT_MAX;
    triangle leftmost_tri;
    for (auto& it : tris)
    {
        float left = GetLeftMostVertex(*it.second);
        if (left < leftmost)
        {
            leftmost = left;
            leftmost_tri = *it.second;
        }
    }
    return GetCentroid(&leftmost_tri);
}

glm::vec3 GetBottomMostTrianglePosition(TTriangleMap tris)
{
    float bottommost = FLT_MAX;
    triangle bottommost_tri;
    for (auto& it : tris)
    {
        float bottom = GetBottomMostVertex(*it.second);
        if (bottom < bottommost)
        {
            bottommost = bottom;
            bottommost_tri = *it.second;
        }
    }
    return GetCentroid(&bottommost_tri);
}

glm::vec3 GetNearMostTrianglePosition(TTriangleMap tris)
{
    float nearmost = FLT_MAX;
    triangle nearmost_tri;
    for (auto& it : tris)
    {
        float n = GetNearMostVertex(*it.second);
        if (n < nearmost)
        {
            nearmost = n;
            nearmost_tri = *it.second;
        }
    }
    return GetCentroid(&nearmost_tri);
}

//return leftmost vertex x of all triangles
float GetLeftMostTriangleVertex(TTriangleMap tris)
{
    float leftmost = FLT_MAX;
    triangle leftmost_tri;
    for (auto& it : tris)
    {
        float left = GetLeftMostVertex(*it.second);
        if (left < leftmost)
        {
            leftmost = left;
            leftmost_tri = *it.second;
        }
    }
    return GetLeftMostVertex(leftmost_tri);
}
//return rightmost vertex x of all triangles
float GetRightMostTriangleVertex(TTriangleMap tris)
{
    float rightmost = -FLT_MAX;
    triangle rightmost_tri;
    for (auto& it : tris)
    {
        float right = GetRightMostVertex(*it.second);
        if (right >= rightmost)
        {
            rightmost = right;
            rightmost_tri = *it.second;
        }
    }
    return GetRightMostVertex(rightmost_tri);
}

//return topmost vertex y of all triangles
float GetTopMostTriangleVertex(TTriangleMap tris)
{
    float topmost = -FLT_MAX;
    triangle topmost_tri;
    for (auto& it : tris)
    {
        float top = GetTopMostVertex(*it.second);
        if (top >= topmost)
        {
            topmost = top;
            topmost_tri = *it.second;
        }
    }
    return GetTopMostVertex(topmost_tri);
}
//return topmost vertex y of all triangles
float GetBottomMostTriangleVertex(TTriangleMap tris)
{
    float bottommost = FLT_MAX;
    triangle bottommost_tri;
    for (auto& it : tris)
    {
        float bottom = GetBottomMostVertex(*it.second);
        if (bottom < bottommost)
        {
            bottommost = bottom;
            bottommost_tri = *it.second;
        }
    }
    return GetBottomMostVertex(bottommost_tri);
}

//return nearmost vertex z of all triangles
float GetNearMostTriangleVertex(TTriangleMap tris)
{
    float nearmost = -FLT_MAX;
    triangle nearmost_tri;
    for (auto& it : tris)
    {
        float n = GetNearMostVertex(*it.second);
        if (n >= nearmost)
        {
            nearmost = n;
            nearmost_tri = *it.second;
        }
    }
    return GetNearMostVertex(nearmost_tri);
}
//return farmost vertex z of all triangles
float GetFarMostTriangleVertex(TTriangleMap tris)
{
    float farmost = FLT_MAX;
    triangle farmost_tri;
    for (auto& it : tris)
    {
        float f = GetFarMostVertex(*it.second);
        if (f < farmost)
        {
            farmost = f;
            farmost_tri = *it.second;
        }
    }
    return GetFarMostVertex(farmost_tri);
}

//void ConstructBVH(BVH_node* BVH, std::map<float, triangle> sorted_triangles);
//void ConstructBVH(BVH_node* BVH, std::vector<triangle> tris);



void ConstructBVH(BVH_node* BVH, TTriangleMap triangles, int bvh_w)
{
    if (triangles.size() <= 0)
        return;

    //specify aabb
    float l = GetLeftMostTriangleVertex(triangles);
    float r = GetRightMostTriangleVertex(triangles);
    float t = GetTopMostTriangleVertex(triangles);
    float b = GetBottomMostTriangleVertex(triangles);
    float n = GetNearMostTriangleVertex(triangles);
    float f = GetFarMostTriangleVertex(triangles);
    BVH->bb.ftl = glm::vec3(l, t, n);
    BVH->bb.bbr = glm::vec3(r, b, f);


    //leaf node containing triangles
    if (triangles.size() <= bvh_w)
    {
        for (auto it: triangles)
        {
            triangle* new_tri = new triangle();
            *new_tri = *it.second;
            BVH->tris.push_back(new_tri);
            num_triangles_added++;
        }

        return;
    }

    //calculate longest dimension
    float aabb_w = abs(r - l);
    float aabb_h = abs(t - b);
    float aabb_d = abs(n - f);
    glm::vec3 extreme;
    if (aabb_w >= aabb_h && aabb_w >= aabb_d)         extreme = GetLeftMostTrianglePosition(triangles);
    else if (aabb_h >= aabb_w && aabb_h >= aabb_d)    extreme = GetBottomMostTrianglePosition(triangles);
    else if (aabb_d >= aabb_w && aabb_d >= aabb_h)    extreme = GetNearMostTrianglePosition(triangles);

    //resort triangles along longest dimension
    TTriangleMap sorted_triangles;
    for (auto it : triangles)
    {
        glm::vec3 cen = GetCentroid(it.second);
        glm::vec3 v = extreme - cen;
        float l = glm::length(v);
        sorted_triangles.insert({ l, it.second });
    }

    //internal node containing child nodes
    int split_point = ceil((float)sorted_triangles.size() / (float)bvh_w);
    //printf("split point %d\n", split_point);

    auto it = sorted_triangles.begin();
    for (int sp = 0; sp < bvh_w; sp++)
    {
        TTriangleMap split;
        for (int i=0; i < split_point; i++)
        {
            if (it == sorted_triangles.end())
                break;
            split.insert({ it->first, it->second });
            it++;
        }

        //debug
        //printf("triangles in node %d, triangles in split %d, width limit %d\n", split.size(), split_point, bvh_w);

        BVH_node* child = new BVH_node();
        ConstructBVH(child, split, bvh_w);
        BVH->child_bvhs.push_back(child);
    }
    
   // printf("created node\n\n");
}

void ConstructBVH(BVH_node* BVH, std::vector<triangle> tris, int bvh_w)
{
    if (tris.size() <= 0)
        return;

    TTriangleMap sorted_triangles;
    for (int t = 0; t < tris.size(); t++)
    {
        sorted_triangles.insert({ t, &tris[t] });
    }

    ConstructBVH(BVH, sorted_triangles, bvh_w);

    printf("%d triangles added to BVH\n", num_triangles_added);
}

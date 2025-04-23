#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <../glm/glm.hpp>

#include <vector>

#include "math.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

using namespace std;
using namespace glm;
/*
class vec3
{
public:
	float x, y, z;

	vec3() {}
	vec3(float l, float m, float r)
	{
		x = l;
		y = m;
		z = r;
	}
	~vec3() {}
};

class vec2
{
public:
	float x, y;

	vec2() {}
	vec2(float l, float r)
	{
		x = l;
		y = r;
	}
	~vec2() {}
};

struct vertex
{
public:
	vec3 vc;
	vec3 co;
	vec3 no;
	vec3 tc;

	vertex() {}
	vertex(vec3 vc_in, vec3 co_in, vec3 no_in, vec3 tc_in)
	{
		vc = vc_in;
		co = co_in;
		no = no_in;
		tc = tc_in;
	}
	~vertex() {}
};
struct triangle
{
public:
	vertex verts[3];

	triangle() {}
	triangle(vertex v0, vertex v1, vertex v2)
	{
		verts[0] = v0;
		verts[1] = v1;
		verts[2] = v2;
	}
	~triangle() {}
};
*/
class Material
{
public:
	char mtl_name[256];
	char fil_name[256];

	Material() {}
	Material(char* n, char* f)
	{
		strcpy(mtl_name, n);
		strcpy(fil_name, f);
	}
	~Material()
	{
	}
};

class Object
{
public:
	unsigned int VAO;
	unsigned int VBO;
	vector<triangle> tris;
	Material mtl;
	int texture;


	Object() {}
	Object(Material m)
	{
		strcpy(mtl.fil_name, m.fil_name);
		strcpy(mtl.mtl_name, m.mtl_name);
	}
	~Object()
	{
	}
};

int mtl_parse(char* filename, vector<Material>* mtls)
{
	char line[256];

	char dir[256];
	strcpy(dir, filename);
	int l = strlen(dir);
	for (int c = l; c > 0; c--)
	{
		if (dir[c] == '/')
		{
			dir[c + 1] = '\0';
			break;
		}
	}



	FILE* file = fopen(filename, "r");
	if (file != NULL)
	{
		while (!feof(file))
		{
			char* p;

			fgets(line, 256, file);

			p = strtok(line, "\t");

			if (line[0] == 'n' &&
				line[1] == 'e' &&
				line[2] == 'w' &&
				line[3] == 'm' &&
				line[4] == 't' &&
				line[5] == 'l')
			{
				char mtl_name[256];
				char fil_name[256];

				strcpy(mtl_name, &line[7]);

				while (true)
				{
					fgets(line, 256, file);

					p = strtok(line, "\t ");

					//if bitmap filename
					if (p[0] == 'm' &&
						p[1] == 'a' &&
						p[2] == 'p' &&
						p[3] == '_' &&
						p[4] == 'K' &&
						p[5] == 'd')
					{

						char tmp[256];
						int tc;
						for (tc = 0; tc < 256 - 7; tc++)
						{
							tmp[tc] = p[7 + tc];
						}
						int l = strlen(tmp);

						strcpy(fil_name, dir);
						strncat(fil_name, tmp, l - 1);

						break;
					}
				}

				mtls->push_back(Material(mtl_name, fil_name));
			}

		}

		//close file
		fclose(file);
	}
	else
	{
		printf("Cannot open file %s, %s\n", filename, strerror(errno));
		return 0;
	}

	printf("successfully parsed %s and read %d materials(s)\n", filename, mtls->size());

	//success
	return 1;
}

int obj_parse(const char* filename, vector<Object>* objs, float scale)
{
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string warn, err;

	if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename)) 
	{
		throw std::runtime_error(warn + err);
	}

	std::vector<vertex> vertices;
	std::vector<uint32_t> indices;

	float max_vert = 0.f;
	for (const auto& v : attrib.vertices)
	{
		if (v > max_vert)
			max_vert = 1;
	}

	for (const auto& shape : shapes) 
	{
		for (const auto& index : shape.mesh.indices) 
		{
			vertex vert{};

			vert.pos =
			{
				(attrib.vertices[3 * index.vertex_index + 0] / max_vert) * scale,
				(attrib.vertices[3 * index.vertex_index + 1] / max_vert) * scale,
				(attrib.vertices[3 * index.vertex_index + 2] / max_vert) * scale
			};

			vert.nor =
			{
				attrib.normals[3 * index.normal_index + 0],
				attrib.normals[3 * index.normal_index + 1],
				attrib.normals[3 * index.normal_index + 2]
			};


			vert.col = { 1, 1, 1 };

			vertices.push_back(vert);
			indices.push_back(indices.size());
		}
	}


	Object obj;
	for (int i = 0; i < indices.size()/3; i++)
	{
		triangle tri;
		tri.v1 = vertices[indices[(i * 3) + 0]];
		tri.v2 = vertices[indices[(i * 3) + 1]];
		tri.v3 = vertices[indices[(i * 3) + 2]];

		tri.reflect = false;
		tri.primitiveID = i;

		bool randomise_colour = false;
		if (randomise_colour)
		{
			float r = 0, g = 0, b = 0;
			while (r <= 0.2)
				r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			while (g <= 0.2)
				g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			while (b <= 0.2)
				b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			tri.v1.col = { r, g, b };
			tri.v2.col = { r, g, b };
			tri.v3.col = { r, g, b };
		}

		obj.tris.push_back(tri);
	}

	objs->push_back(obj);

	int num_tris = 0;
	for (auto& o : *objs)
	{
		num_tris += o.tris.size();
	}
	printf("successfully parsed %s and read %d object(s)  %d triangles \n", filename, objs->size(), num_tris);

}
/*

int obj_parse(const char* filename, vector<Object>* objs)
{
	vector<Material> materials;
	vector<vec3> vecs;
	vector<vec3> uvs;
	vector<vec3> norms;

	float max_vert = 0.f;

	char line[256];

	FILE* file = fopen(filename, "r");

	if (file != NULL)
	{
		while (fgets(line, 256, file) != NULL)
		{
			if (line[0] == 'v' && line[1] == 't')
			{
				char* tok1 = strtok(line, " ");
				char* tok2 = strtok(NULL, " ");
				char* tok3 = strtok(NULL, " ");
				uvs.push_back(vec3(1.0f - atof(tok2), 1.0f - atof(tok3), 0.f));
			}
			else if (line[0] == 'v' && line[1] == 'n')
			{
				char* tok1 = strtok(line, " ");
				char* tok2 = strtok(NULL, " ");
				char* tok3 = strtok(NULL, " ");
				char* tok4 = strtok(NULL, " ");
				vec3 v(atof(tok2), atof(tok3), atof(tok4));
				norms.push_back(v);
			}
			else if (line[0] == 'v')
			{

				char* tok1 = strtok(line, " ");
				char* tok2 = strtok(NULL, " ");
				char* tok3 = strtok(NULL, " ");
				char* tok4 = strtok(NULL, " ");
				vec3 v(atof(tok2), atof(tok3), atof(tok4));

				if (abs(v.x) > max_vert)
					max_vert = abs(v.x);
				if (abs(v.y) > max_vert)
					max_vert = abs(v.y);
				if (abs(v.z) > max_vert)
					max_vert = abs(v.z);

				vecs.push_back(v);

			}

			//if triangle data
			else if (line[0] == 'f')
			{
				char* tok1 = strtok(line, " ");
				char* tok2 = strtok(NULL, " ");
				char* tok3 = strtok(NULL, " ");
				char* tok4 = strtok(NULL, " ");

				//possible 2nd triangle
				char* tok5 = strtok(NULL, " ");

				char* X1 = strtok(tok2, "/");
				char* Y1 = strtok(NULL, "/");
				char* Z1 = strtok(NULL, "");
				char* X2 = strtok(tok3, "/");
				char* Y2 = strtok(NULL, "/");
				char* Z2 = strtok(NULL, "");
				char* X3 = strtok(tok4, "/");
				char* Y3 = strtok(NULL, "/");
				char* Z3 = strtok(NULL, "");


				int av = atoi(X1) - 1;
				int au = atoi(Y1) - 1;
				int an = atoi(Z1) - 1;
				int bv = atoi(X2) - 1;
				int bu = atoi(Y2) - 1;
				int bn = atoi(Z2) - 1;
				int cv = atoi(X3) - 1;
				int cu = atoi(Y3) - 1;
				int cn = atoi(Z3) - 1;
				//printf("%dth tri, %d/%d/%d %d/%d/%d %d/%d/%d\n", (*tricount)+1, av, au, an, bv, bu, bn, cv, cu, cn);

				//max_vert = 1.f;
				float r = 1.f;// static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				float g = 1.f;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				float b = 1.f;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

				vertex v0;
				v0.pos = vec3((vecs[av].x / max_vert), (vecs[av].y / max_vert), (vecs[av].z / max_vert));
//				v0.col = vec3(1.f, 1.f, 1.f);
				v0.col = vec3(r, g, b);
				v0.nor = vec3(norms[an].x, norms[an].y, norms[an].z);
				//	vec3(1 - uvs[au].x, uvs[au].y, 0.f));
				vertex v1;
				v1.pos = vec3((vecs[bv].x / max_vert), (vecs[bv].y / max_vert), (vecs[bv].z / max_vert));
//				v1.col = vec3(1.f, 1.f, 1.f);
				v1.col = vec3(r, g, b);
				v1.nor = vec3(norms[bn].x, norms[bn].y, norms[bn].z);
				//	vec3(1 - uvs[bu].x, uvs[bu].y, 0.f));
				vertex v2;
				v2.pos = vec3((vecs[cv].x / max_vert), (vecs[cv].y / max_vert), (vecs[cv].z / max_vert));
//				v2.col = vec3(1.f, 1.f, 1.f);
				v2.col = vec3(r, g, b);
				v2.nor = vec3(norms[cn].x, norms[cn].y, norms[cn].z);
				//	vec3(1 - uvs[cu].x, uvs[cu].y, 0.f));
				triangle t;
				t.v1 = v0;
				t.v2 = v1;
				t.v3 = v2;
				t.reflect = false;
				objs->back().tris.push_back(t);

				//possible 2nd triangle
				if (tok5 != NULL && strcmp(tok5, "\n") != 0)
				{
					char* X4 = strtok(tok5, "/");
					char* Y4 = strtok(NULL, "/");
					char* Z4 = strtok(NULL, "");
					int dv = atoi(X4) - 1;
					int du = atoi(Y4) - 1;
					int dn = atoi(Z4) - 1;

					float r = 1.f;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
					float g = 1.f;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
					float b = 1.f;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

					vertex v0;
					v0.pos = vec3((vecs[av].x / max_vert), (vecs[av].y / max_vert), (vecs[av].z / max_vert));
//					v0.col = vec3(1.f, 1.f, 1.f);
					v0.col = vec3(r, g, b);
					v0.nor = vec3(norms[an].x, norms[an].y, norms[an].z);
					//	vec3(1 - uvs[au].x, uvs[au].y, 0.f));
					vertex v2;
					v2.pos = vec3((vecs[cv].x / max_vert), (vecs[cv].y / max_vert), (vecs[cv].z / max_vert));
//					v2.col = vec3(1.f, 1.f, 1.f);
					v2.col = vec3(r, g, b);
					v2.nor = vec3(norms[cn].x, norms[cn].y, norms[cn].z);
					//	vec3(1 - uvs[cu].x, uvs[cu].y, 0.f));
					vertex v1;
					v1.pos = vec3((vecs[dv].x / max_vert), (vecs[dv].y / max_vert), (vecs[dv].z / max_vert));
//					v1.col = vec3(1.f, 1.f, 1.f);
					v1.col = vec3(r, g, b);
					v1.nor = vec3(norms[dn].x, norms[dn].y, norms[dn].z);
					//	vec3(1 - uvs[du].x, uvs[du].y, 0.f));
					triangle t;
					t.v1 = v0;
					t.v2 = v1;
					t.v3 = v2;
					t.reflect = false;
					objs->back().tris.push_back(t);
				}
			}
			//mtl data 
			else if (line[0] == 'u' &&
				line[1] == 's' &&
				line[2] == 'e' &&
				line[3] == 'm' &&
				line[4] == 't' &&
				line[5] == 'l')
			{
				char* p = strtok(&line[6], " ");

				for (int m = 0; m < materials.size(); m++)
				{
					if (strcmp(p, materials[m].mtl_name) == 0)
					{
						objs->push_back(Object(materials[m]));
					}
				}
			}

			//mtl data 
			else if (line[0] == 'm' && line[1] == 't' && line[2] == 'l' && line[3] == 'l' && line[4] == 'i' && line[5] == 'b')
			{
				int len = strlen(filename);
				int loclength = len - 1;
				while (filename[loclength] != '/') loclength--;
				char locname[256];
				strncpy(locname, filename, loclength + 1);
				//printf("filename = %s\n", filename);
				locname[loclength + 1] = '\0';
				//printf("location only = %s\n", locname);

				//printf("line[7] = %s\n", &line[7]);
				char tmp[256];
				int tc;
				for (tc = 0; tc < 256 - 7; tc++)
				{
					if (line[7 + tc] == '\n')
					{
						//tc = 512;
						break;
					}
					tmp[tc] = line[7 + tc];
				}
				tmp[tc] = '\0';
				//printf("mtl filename = %s\n", tmp);
				strncat(locname, tmp, strlen(tmp));

				//printf("mtlfile = %s", locname);

				mtl_parse(locname, &materials);
			}

		}

		//close file
		fclose(file);
	}
	else
	{
		printf("Cannot open file %s, %s\n", filename, strerror(errno));
		return 0;
	}

	int num_tris = 0;
	for (auto& o : *objs)
	{
		num_tris += o.tris.size();
	}

	printf("successfully parsed %s and read %d object(s)  %d triangles \n", filename, objs->size(), num_tris);


	//success
	return 1;
}

*/

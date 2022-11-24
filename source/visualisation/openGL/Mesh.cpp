#include "visualisation/openGL/Mesh.hpp"

#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

using namespace std;

bool Mesh::LoadFromFile(std::string path)
{
	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(path, aiProcess_Triangulate );
	if (!scene->HasMeshes())
	{
		return false;
	}
	const aiMesh* mesh = scene->mMeshes[0];
	if (!mesh->HasPositions())
	{
		cerr << "Mesh has no vertices" << endl;
		return false;
	}
	if (!mesh->HasVertexColors(0))
	{
		cerr << "Mesh has no vertex colors" << endl;
		//return false;
	}
	Positions.reserve(mesh->mNumVertices *3);
	Colors.reserve(mesh->mNumVertices *3);
	for (int i = 0; i < mesh->mNumVertices; i++)
	{
		aiVector3D pos = mesh->mVertices[i];
		aiColor4D col;
		if (mesh->HasVertexColors(0))
		{
			col = mesh->mColors[0][i];
		}
		else
		{
			col = aiColor4D(0.5f,0,0.5f,1);
		}
		aiVector3D normal = mesh->mNormals[i];
		for (int j = 0; j < 3; j++)
		{
			Positions.push_back(pos[j]);
			Colors.push_back(col[j]);
			Normals.push_back(normal[j]);
		}
	}
	if (mesh->HasFaces())
	{
		Indices.reserve(mesh->mNumFaces *3);
		for (int i = 0; i < mesh->mNumFaces; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Indices.push_back(mesh->mFaces[i].mIndices[j]);
			}
		}
	}
	
	return true;
}

void Mesh::BindMesh()
{
	// Generate 1 buffer, put the resulting identifier in vertexbuffer
	glGenBuffers(1, &PositionBuffer);
	// The following commands will talk about our 'vertexbuffer' buffer
	glBindBuffer(GL_ARRAY_BUFFER, PositionBuffer);
	// Give our vertices to OpenGL.
	glBufferData(GL_ARRAY_BUFFER, Positions.size() * sizeof(GLfloat), Positions.data(), GL_STATIC_DRAW);

	glGenBuffers(1, &ColorBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, ColorBuffer);
	glBufferData(GL_ARRAY_BUFFER, Colors.size() * sizeof(GLfloat), Colors.data(), GL_STATIC_DRAW);

	glGenBuffers(1, &NormalBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, NormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, Normals.size() * sizeof(GLfloat), Normals.data(), GL_STATIC_DRAW);

	// Generate a buffer for the indices
	glGenBuffers(1, &IndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, Indices.size() * sizeof(unsigned int), Indices.data(), GL_STATIC_DRAW);
}

void Mesh::Draw()
{
	// 1st attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, PositionBuffer);
	glVertexAttribPointer(
	0,                   // attribute 0. No particular reason for 0, but must match the layout in the shader.
	3,                   // size
	GL_FLOAT,            // type
	GL_FALSE,            // normalized?
	0,                   // stride
	(void*)0             // array buffer offset
	);

	// 2nd attribute buffer : colors
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, ColorBuffer);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// 3nd attribute buffer : normals
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, ColorBuffer);
	glVertexAttribPointer(
		2,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// Index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBuffer);

	// Draw the triangles !
	glDrawElements(
		GL_TRIANGLES,      // mode
		Indices.size(),    // count
		GL_UNSIGNED_INT,   // type
		(void*)0           // element array buffer offset
	);
}
#include "visualisation/openGL/Mesh.hpp"

#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

using namespace std;

bool Mesh::LoadMesh(std::string path)
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
		cerr << "Mesh " << path << " has no vertices" << endl;
		return false;
	}
	if (!mesh->HasVertexColors(0))
	{
		cerr << "Mesh " << path << " has no vertex colors" << endl;
		//return false;
	}
	Positions.reserve(mesh->mNumVertices *3);
	Normals.reserve(mesh->mNumVertices *3);
	UVs.reserve(mesh->mNumVertices *2);
	Colors.reserve(mesh->mNumVertices *3);
	for (int i = 0; i < mesh->mNumVertices; i++)
	{
		aiVector3D pos = mesh->mVertices[i];
		aiVector3D normal = mesh->mNormals[i];
		aiVector3D UV;
		if (mesh->HasTextureCoords(0))
		{
			UV = mesh->mTextureCoords[0][i];
		}
		else
		{
			UV = aiVector3D(0,0,0);
		}
		aiColor4D col;
		if (mesh->HasVertexColors(0))
		{
			col = mesh->mColors[0][i];
		}
		else
		{
			col = aiColor4D(0.5f,0,0.5f,1);
		}
		UVs.push_back(UV[0]);
		UVs.push_back(1-UV[1]);
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


bool Mesh::LoadTexture(cv::Mat Texture)
{
	texture.Texture = Texture;
	texture.valid = true;
	return true;
}

bool Mesh::LoadTexture(std::string path)
{
	if (path != "")
	{
		texture.LoadFromFile(path);
		return true;
	}
	return false;
}


bool Mesh::LoadFromFile(std::string path, std::string texturepath)
{
	if(!LoadMesh(path))
	{
		return false;
	}
	if(!LoadTexture(texturepath))
	{
		return false;
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

	glGenBuffers(1, &NormalBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, NormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, Normals.size() * sizeof(GLfloat), Normals.data(), GL_STATIC_DRAW);

	glGenBuffers(1, &UVBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, UVBuffer);
	glBufferData(GL_ARRAY_BUFFER, UVs.size() * sizeof(GLfloat), UVs.data(), GL_STATIC_DRAW);

	glGenBuffers(1, &ColorBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, ColorBuffer);
	glBufferData(GL_ARRAY_BUFFER, Colors.size() * sizeof(GLfloat), Colors.data(), GL_STATIC_DRAW);

	// Generate a buffer for the indices
	glGenBuffers(1, &IndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, Indices.size() * sizeof(unsigned int), Indices.data(), GL_STATIC_DRAW);

	if (texture.valid)
	{
		texture.Bind();
	}
	
}

void Mesh::Draw(GLuint ParamHandle, bool forceTexture)
{
	int i = 0;

	glEnableVertexAttribArray(i);
	glBindBuffer(GL_ARRAY_BUFFER, PositionBuffer);
	glVertexAttribPointer(
	i,                   // attribute 0. No particular reason for 0, but must match the layout in the shader.
	3,                   // size
	GL_FLOAT,            // type
	GL_FALSE,            // normalized?
	0,                   // stride
	(void*)0             // array buffer offset
	);
	i++;

	glEnableVertexAttribArray(i);
	glBindBuffer(GL_ARRAY_BUFFER, NormalBuffer);
	glVertexAttribPointer(
		i,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);
	i++;

	glEnableVertexAttribArray(i);
	glBindBuffer(GL_ARRAY_BUFFER, UVBuffer);
	glVertexAttribPointer(
		i,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		2,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);
	i++;

	glEnableVertexAttribArray(i);
	glBindBuffer(GL_ARRAY_BUFFER, ColorBuffer);
	glVertexAttribPointer(
		i,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);
	i++;
	
	if (texture.valid)
	{
		texture.Draw();
	}
	glUniform1i(ParamHandle, texture.valid || forceTexture);
	
	

	// Index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBuffer);

	// Draw the triangles !
	glDrawElements(
		GL_TRIANGLES,      // mode
		Indices.size(),    // count
		GL_UNSIGNED_INT,   // type
		(void*)0           // element array buffer offset
	);

	for (; i >= 0; i--)
	{
		glDisableVertexAttribArray(i);
	}
	
}
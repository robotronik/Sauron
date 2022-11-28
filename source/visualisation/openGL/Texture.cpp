#include "visualisation/openGL/Texture.hpp"

#include <opencv2/imgcodecs.hpp>

using namespace std;

void Texture::LoadFromFile(string path)
{
	Texture = cv::imread(path, cv::IMREAD_COLOR);
	valid = true;
}

void Texture::Bind()
{
	glGenTextures(1, &TextureID);
	glBindTexture(GL_TEXTURE_2D, TextureID);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, Texture.cols, Texture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, Texture.data);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

void Texture::Draw()
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, TextureID);
}
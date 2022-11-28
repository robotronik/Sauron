#version 330 core

in vec3 fragmentColor;
in vec2 UV;

uniform sampler2D TextureSampler;
uniform int Parameters;
out vec3 color;

void main(){
	if((Parameters & 1) > 0)
	{
		color = texture( TextureSampler, UV ).rgb;
	}
	else
	{
		color = fragmentColor;
	}
}
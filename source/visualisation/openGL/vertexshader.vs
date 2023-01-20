#version 330 core
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexUV;
layout(location = 3) in vec3 vertexColor;

uniform mat4 MVP;
uniform float scale;

out vec3 fragmentColor;
out vec2 UV;

void main(){
	// Output position of the vertex, in clip space : MVP * position
	gl_Position =  MVP * vec4(vertexPosition_modelspace * scale,1);
	
    fragmentColor = vertexColor;
	UV = vertexUV;
}
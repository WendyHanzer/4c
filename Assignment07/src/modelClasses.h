#ifndef MODELCLASSES_H
#define MODELCLASSES_H
#define GLM_FORCE_RADIANS
#include <GL/glew.h> 
#include <GL/glut.h>
#include <Magick++.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>




struct bodyData
{
    float selfSpin;
    float revolution;
    float axisTilt;
    float raduis;
    float revolutionRadius;
    float revolutionTilt;
};

//--Data types
//This object will define the attributes of a vertex(position, color, etc...)
struct Vertex
{
    GLfloat position[3];
    GLfloat texuv[2];
};

class Texture{  //CLASS FROM OGLDEV_TEXTURE
	public:
		Texture(GLenum TextureTarget, const std::string& FileName);
		bool Load();
		void Bind(GLenum TextureUnit);
	private:
		std::string mfileName;
		GLenum mtextureTarget;
		GLuint mtextureObj;
		Magick::Image mimage;
		Magick::Blob mblob;
	};

class Object
{
    public:
        Object();
        bool render();
        bool bind();
        bool loadModel(std::string objName);
        void tick(float dt);

	    int numMesh;
        Vertex *Geo;
        char *name;
        unsigned int NumVert;
        Texture *Texs;
        bodyData planetData;
        glm::mat4 model;
        bool isMoon;
};

#endif

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


struct meshData
{
    int numMesh;
    unsigned int NumVert;
    Vertex *Geo;
    GLuint bufferName;
};

struct bodyData
{
    float selfSpin;
    float revolution;
    float axisTilt; // self lean
    float raduis; // of planet
    float revolutionRadius; // orbit raduis
    float revolutionTilt; // tilt of orbit
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
        Object(GLuint buffer, bool moon);
        bool bind();
        void render();
        bool load(std::string objName); //load obj model and texture info
        void tick(float dt);


    private:
        // model
        meshData mesh;
        glm::mat4 modelMatrix;

        // texture
        Texture *Texs;

        // other
        char *name;
        bodyData planetData;

        // moon stuff
        bool isMoon;
};

#endif

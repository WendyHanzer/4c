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

struct Vertex
{
    GLfloat position[3];
    GLfloat texuv[2];
};

struct meshData
{
    unsigned int NumVert;
    Vertex *Geo;
    GLuint bufferName;
};

struct bodyData
{
    float selfSpin;
    float revolution;
    float axisTilt; // self lean
    float radius; // of planet
    float revolutionRadius; // orbit raduis
    float revolutionTilt; // tilt of orbit
    bool isMoon;
    int parentInd;
};

//--Data types
//This object will define the attributes of a vertex(position, color, etc...)


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
<<<<<<< HEAD
        bool bind(int index);
=======
        bool bind();
>>>>>>> e61a61e35d7c282a10a022494c6d57f96f60cfab
        void render();
        bool load(std::string objName); //load obj model and texture info
        void tick(float dt);
        
        char *name;
        bodyData planetData;

        // model
        meshData *mesh;
        glm::mat4 modelMatrix;
        int numMesh= 0;
   
        // texture
        Texture *Texs;

        // other
        GLuint bufferName;
};

#endif

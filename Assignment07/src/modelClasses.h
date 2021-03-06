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
#include <vector>

using namespace std;

class Object;

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

struct meshData
{
    unsigned int NumVert;
    Vertex *Geo;
    GLuint bufferName;
    Texture *Texs;
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
    Object* parent;
};

class Object
{
    public:
        Object();

        bool bind(int index);
        bool bind();
        glm::mat4 render(glm::mat4 vp);
        bool load(char *objName); //load obj model and texture info
        void tick(float dt);
        glm::vec3 getPosition();
        
        char name[50];
        bodyData planetData;

        // model
        meshData *mesh;
        glm::mat4 modelMatrix;
        int numMesh;

        float orbitAngle;
        float spinAngle;

};

#endif

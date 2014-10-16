#define GLM_FORCE_RADIANS
#include "modelClasses.h"
#include <iostream>
#include <GL/glew.h> 
#include <GL/glut.h>
#include <Magick++.h>
#include <string>

using namespace std;

Texture::Texture(GLenum TextureTarget, const std::string& FileName)
	{
	mtextureTarget = TextureTarget;
	mfileName = FileName;
	}

bool Texture::Load(){
	try{
		mimage.read(mfileName);
		mimage.write(&mblob, "RGBA");
		}
	catch (Magick::Error& Error){
		cout<<"Didn't load texture"<<mfileName<<Error.what()<<endl;
		return false;
		}
	glGenTextures(1, &mtextureObj);
	glBindTexture(mtextureTarget, mtextureObj);
	glTexImage2D(mtextureTarget, 0, GL_RGBA, mimage.columns(), mimage.rows(), 0, GL_RGBA, GL_UNSIGNED_BYTE, mblob.data());
	glTexParameterf(mtextureTarget, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(mtextureTarget, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(mtextureTarget, 0);
	return true;
	}

void Texture::Bind(GLenum TextureUnit){
	glActiveTexture(TextureUnit);
	glBindTexture(mtextureTarget, mtextureObj);
	} 
	

bool Object::bind()
    {
     glBindBuffer(GL_ARRAY_BUFFER, bufferName);

     return true;
    }

void Object::render()
    {
     // something
    }

bool Object::load(std::string objName)
    {
     return false;
    }

void Object::tick(float dt)
    {
     static float orbitAngle = 0;
     static float spinAngle = 0;

     orbitAngle += planetData.revolution * dt;
     spinAngle += planetData.selfSpin * dt;

     // add orbit tilt
     modelMatrix = glm::rotate(glm::mat4(1.0f), planetData.revolutionTilt, glm::vec3(0.0,0.0,1.0));

     // orbit position
     modelMatrix = glm::translate(modelMatrix, glm::vec3(planetData.revolutionRadius * sin(orbitAngle), 0.0, planetData.revolutionRadius * cos(orbitAngle)));

     // axis tilt
     modelMatrix = glm::rotate(modelMatrix, planetData.axisTilt, glm::vec3(1.0,0.0,0.0));

     // self rotation
     modelMatrix = glm::rotate(modelMatrix, spinAngle, glm::vec3(0.0,1.0,0.0));

     // scale planet size
     modelMatrix = glm::scale(modelMatrix, glm::vec3(planetData.radius));


    }

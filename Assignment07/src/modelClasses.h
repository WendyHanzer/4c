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

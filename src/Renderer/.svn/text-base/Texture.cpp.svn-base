//////////////////////////////////////////////////////////////////////
//
// OpenGL Texture Class
// by: Matthew Fairfax
//
// Texture.cpp: implementation of the Texture class.
// This class loads a texture file and prepares it
// to be used in OpenGL. It can open a bitmap or a
// targa file. The min filter is set to mipmap b/c
// they look better and the performance cost on
// modern video cards in negligible. I leave all of
// the texture management to the application. I have
// included the ability to load the texture from a
// Visual Studio resource. The bitmap's id must be
// be surrounded by quotation marks (i.e. "Texture.bmp").
// The targa files must be in a resource type of "TGA"
// (including the quotes). The targa's id must be
// surrounded by quotation marks (i.e. "Texture.tga").
//
// Usage:
// Texture tex;
// Texture tex1;
// Texture tex3;
//
// tex.Load("texture.bmp"); // Loads a bitmap
// tex.Use();				// Binds the bitmap for use
// 
// tex1.LoadFromResource("texture.tga"); // Loads a targa
// tex1.Use();				 // Binds the targa for use
//
// // You can also build a texture with a single color and use it
// tex3.BuildColorTexture(255, 0, 0);	// Builds a solid red texture
// tex3.Use();				 // Binds the targa for use
//
//	Jaeyoung Haan, Apr 14, 2009
//	Change class name.
//	Make platform-independent.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#include "srg/srgBitmap.h"
#include "Renderer/Texture.h"

#ifdef __STDC__
	#include <ctype.h>

	char* _strlwr(char* string)
	{
		int i=0;
		while(string[i])
			tolower(string[i++]);

		return string;
	}
#endif

using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
Texture::Texture()
{
	m_TextureName = 0;
	m_Width = 0;
	m_Height = 0;
}

Texture::~Texture()
{
}

void Texture::Load(char *name)
{
	// make the texture name all lower case
	m_TextureName = _strlwr(strdup(name));

	// strip "'s
	if (strstr(m_TextureName, "\""))
		m_TextureName = strtok(m_TextureName, "\"");

	// check the file extension to see what type of texture
	if(strstr(m_TextureName, ".bmp"))	
		LoadBMP(m_TextureName);
	if(strstr(m_TextureName, ".tga"))	
		LoadTGA(m_TextureName);
}

void Texture::Use()
{
	glEnable(GL_TEXTURE_2D);					// Enable texture mapping
	glBindTexture(GL_TEXTURE_2D, texture[0]);	// Bind the texture as the current one
}

void Texture::LoadBMP(char *name)
{
	srgBitmap bmp;

	bmp.BMPLoad(name);

	// Just in case we want to use the m_Width and m_Height later
	m_Width  = bmp.GetWidth();
	m_Height = bmp.GetHeight();

	// Generate the OpenGL texture id
	glGenTextures(1, &texture[0]);

	// Bind this texture to its id
	glBindTexture(GL_TEXTURE_2D, texture[0]);

	// Use mipmapping filter
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Generate the mipmaps
	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, m_Width, m_Height, GL_RGB, GL_UNSIGNED_BYTE, bmp.GetData());
}

void Texture::LoadTGA(char *name)
{
	GLubyte	TGAheader[12] = {0,0,2,0,0,0,0,0,0,0,0,0};	// Uncompressed TGA header
	GLubyte	TGAcompare[12];								// Used to compare TGA header
	GLubyte	header[6];									// First 6 useful bytes of the header
	GLuint	bytesPerPixel;								// Holds the number of bytes per pixel used
	GLuint	imageSize;									// Used to store the image size
	GLuint	temp;										// Temporary variable
	GLuint	type = GL_RGBA;								// Set the default type to RBGA (32 BPP)
	GLubyte	*imageData;									// Image data (up to 32 Bits)
	GLuint	bpp;										// Image color depth in bits per pixel.

	FILE *file;
	file = fopen(name, "rb");							// Open the TGA file

	// Load the file and perform checks
	if(file == NULL ||														// Does file exist?
	   fread(TGAcompare,1,sizeof(TGAcompare),file) != sizeof(TGAcompare) ||	// Are there 12 bytes to read?
	   memcmp(TGAheader,TGAcompare,sizeof(TGAheader)) != 0				 ||	// Is it the right format?
	   fread(header,1,sizeof(header),file) != sizeof(header))				// If so then read the next 6 header bytes
	{
		if (file == NULL)									// If the file didn't exist then return
			return;
		else
		{
			fclose(file);									// If something broke then close the file and return
			return;
		}
	}

	// Determine the TGA m_Width and m_Height (highbyte*256+lowbyte)
	m_Width  = header[1] * 256 + header[0];
	m_Height = header[3] * 256 + header[2];
    
	// Check to make sure the targa is valid and is 24 bit or 32 bit
	if(m_Width	<=0	||										// Is the m_Width less than or equal to zero
	   m_Height	<=0	||										// Is the m_Height less than or equal to zero
	   (header[4] != 24 && header[4] != 32))				// Is it 24 or 32 bit?
	{
		fclose(file);										// If anything didn't check out then close the file and return
		return;
	}

	bpp				= header[4];							// Grab the bits per pixel
	bytesPerPixel	= bpp / 8;								// Divide by 8 to get the bytes per pixel
	imageSize		= m_Width * m_Height * bytesPerPixel;		// Calculate the memory required for the data

	// Allocate the memory for the image data
	imageData		= new GLubyte[imageSize];

	// Make sure the data is allocated write and load it
	if(imageData == NULL ||									// Does the memory storage exist?
	   fread(imageData, 1, imageSize, file) != imageSize)	// Does the image size match the memory reserved?
	{
		if(imageData != NULL)								// Was the image data loaded
			free(imageData);								// If so, then release the image data

		fclose(file);										// Close the file
		return;
	}

	// Loop through the image data and swap the 1st and 3rd bytes (red and blue)
	for(GLuint i = 0; i < int(imageSize); i += bytesPerPixel)
	{
		temp = imageData[i];
		imageData[i] = imageData[i + 2];
		imageData[i + 2] = temp;
	}

	// We are done with the file so close it
	fclose(file);

	// Set the type
	if (bpp == 24)
		type = GL_RGB;
	
	// Generate the OpenGL texture id
	glGenTextures(1, &texture[0]);

	// Bind this texture to its id
	glBindTexture(GL_TEXTURE_2D, texture[0]);

	// Use mipmapping filter
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

	// Generate the mipmaps
	gluBuild2DMipmaps(GL_TEXTURE_2D, type, m_Width, m_Height, type, GL_UNSIGNED_BYTE, imageData);

	// Cleanup
	free(imageData);
}

void Texture::BuildColorTexture(unsigned char r, unsigned char g, unsigned char b)
{
	unsigned char data[12];	// a 2x2 texture at 24 bits

	// Store the data
	for(int i = 0; i < 12; i += 3)
	{
		data[i]   = r;
		data[i+1] = g;
		data[i+2] = b;
	}

	// Generate the OpenGL texture id
	glGenTextures(1, &texture[0]);

	// Bind this texture to its id
	glBindTexture(GL_TEXTURE_2D, texture[0]);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	// Use mipmapping filter
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Generate the texture
	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 2, 2, GL_RGB, GL_UNSIGNED_BYTE, data);
}

/*******************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.001 : 2008-11-11			Jeongseok Lee
*******************************************************************************/

#ifndef __SRG_GL__
#define __SRG_GL__

/*!
OpenGL libraries for multi platform.
*/

// Windows
#ifdef WIN32
	#include <windows.h>
	#include <GL/glew.h> 
	#define GLUT_DISABLE_ATEXIT_HACK
	#include <GL/glut.h>

	#define _CRT_SECURE_NO_DEPRECATE 
	#define _CRT_SECURE_NO_WARNINGS 
	#define _CRT_NONSTDC_NO_DEPRECATE 

	#pragma warning ( disable:4996 )
#endif

// Mac OS X
#ifdef __APPLE__
	#include <GL/glew.h> 
	#include <GLUT/glut.h>
	#include <sys/time.h>
	#define TRUE true
	#define FALSE false
	typedef unsigned long DWORD;	
#endif

// LINUX
#if defined(__linux__) || defined(linux)
	#include <GL/glew.h>
	#include <GL/glut.h>
	#include <unistd.h>
	#define TRUE true
	#define FALSE false
	typedef unsigned long DWORD;
#endif



#endif	// __SRG_GL__

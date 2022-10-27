// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "SnippetFontData.h"
#include "SnippetFontRenderer.h"
#include "SnippetRender.h"

bool GLFontRenderer::m_isInit=false;
unsigned int GLFontRenderer::m_textureObject=0;
int GLFontRenderer::m_screenWidth=640;
int GLFontRenderer::m_screenHeight=480;
float GLFontRenderer::m_color[4]={1.0f, 1.0f, 1.0f, 1.0f};

namespace
{
	struct RGBAPixel
	{
		unsigned char R,G,B,A;
	};

	#define PIXEL_OPAQUE	0xff
}

bool GLFontRenderer::init()
{
	glGenTextures(1, (GLuint*)&m_textureObject);
	if(!m_textureObject)
		return false;

	glBindTexture(GL_TEXTURE_2D, m_textureObject);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// expand to rgba
	RGBAPixel* P = new RGBAPixel[OGL_FONT_TEXTURE_WIDTH*OGL_FONT_TEXTURE_HEIGHT];
	for(int i=0;i<OGL_FONT_TEXTURE_WIDTH*OGL_FONT_TEXTURE_HEIGHT;i++)
	{
		if(1)
		{
			P[i].R = PIXEL_OPAQUE;
			P[i].G = PIXEL_OPAQUE;
			P[i].B = PIXEL_OPAQUE;
			P[i].A = OGLFontData[i];
		}
		else
		{
			P[i].R = OGLFontData[i];
			P[i].G = OGLFontData[i];
			P[i].B = OGLFontData[i];
			P[i].A = PIXEL_OPAQUE;
		}
	}

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, OGL_FONT_TEXTURE_WIDTH, OGL_FONT_TEXTURE_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, P);

	delete [] P;

	m_isInit = true;

	return true;
}

void GLFontRenderer::print(float x, float y, float fontSize, const char* pString, bool forceMonoSpace, int monoSpaceWidth, bool doOrthoProj)
{
	if(1)
	{
		const float Saved0 = m_color[0];
		const float Saved1 = m_color[1];
		const float Saved2 = m_color[2];

		m_color[0] = 0.0f;
		m_color[1] = 0.0f;
		m_color[2] = 0.0f;

//		const float Offset = fontSize * 0.05f;
//		const float Offset = fontSize * 0.075f;
		const float Offset = fontSize * 0.1f;

		print_(x+Offset, y-Offset, fontSize, pString, forceMonoSpace, monoSpaceWidth, doOrthoProj);
		//print_(x-Offset, y-Offset, fontSize, pString, forceMonoSpace, monoSpaceWidth, doOrthoProj);
		//print_(x+Offset, y+Offset, fontSize, pString, forceMonoSpace, monoSpaceWidth, doOrthoProj);
		//print_(x-Offset, y+Offset, fontSize, pString, forceMonoSpace, monoSpaceWidth, doOrthoProj);

		m_color[0] = Saved0;
		m_color[1] = Saved1;
		m_color[2] = Saved2;
	}
	print_(x, y, fontSize, pString, forceMonoSpace, monoSpaceWidth, doOrthoProj);
}

void GLFontRenderer::print_(float x, float y, float fontSize, const char* pString, bool forceMonoSpace, int monoSpaceWidth, bool doOrthoProj)
{
	x = x*m_screenWidth;
	y = y*m_screenHeight;
	fontSize = fontSize*m_screenHeight;

	if(!m_isInit)
		m_isInit = init();

	unsigned int num = (unsigned int)(strlen(pString));
	if(m_isInit && num > 0)
	{
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, m_textureObject);

		if(doOrthoProj)
		{
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0, m_screenWidth, 0, m_screenHeight, -1, 1);
		}
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glEnable(GL_BLEND);
		glDisable(GL_CULL_FACE);

		glColor4f(m_color[0], m_color[1], m_color[2], m_color[3]);

		const float glyphHeightUV = ((float)OGL_FONT_CHARS_PER_COL)/OGL_FONT_TEXTURE_HEIGHT*2 - 1.0f/128.0f;
		//const float glyphHeightUV = ((float)OGL_FONT_CHARS_PER_COL)/OGL_FONT_TEXTURE_HEIGHT*2;

		float translate = 0.0f;

		float* pVertList = new float[num*3*6];
    	float* pTextureCoordList = new float[num*2*6];

		int vertIndex = 0;
    	int textureCoordIndex = 0;

		float translateDown = 0.0f;
		unsigned int count = 0;

		for(unsigned int i=0;i<num; i++)
		{
			const float glyphWidthUV = ((float)OGL_FONT_CHARS_PER_ROW)/OGL_FONT_TEXTURE_WIDTH;

			if (pString[i] == '\n')
			{
				translateDown-=0.005f*m_screenHeight+fontSize;
				translate = 0.0f;
				continue;
			}

			int c = pString[i]-OGL_FONT_CHAR_BASE;
			if (c < OGL_FONT_CHARS_PER_ROW*OGL_FONT_CHARS_PER_COL)
			{
				count++;

				float glyphWidth = (float)GLFontGlyphWidth[c]+1;
				if(forceMonoSpace)
					glyphWidth = (float)monoSpaceWidth;
				
				glyphWidth = glyphWidth*(fontSize/(((float)OGL_FONT_TEXTURE_WIDTH)/OGL_FONT_CHARS_PER_ROW))-0.01f;

				const float cxUV = float((c)%OGL_FONT_CHARS_PER_ROW)/OGL_FONT_CHARS_PER_ROW+0.008f;
				const float cyUV = float((c)/OGL_FONT_CHARS_PER_ROW)/OGL_FONT_CHARS_PER_COL+0.008f;

				pTextureCoordList[textureCoordIndex++] = cxUV;
				pTextureCoordList[textureCoordIndex++] = cyUV+glyphHeightUV;

				pVertList[vertIndex++] = x+0+translate;
				pVertList[vertIndex++] = y+0+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV+glyphWidthUV;
				pTextureCoordList[textureCoordIndex++] = cyUV;

				pVertList[vertIndex++] = x+fontSize+translate;
				pVertList[vertIndex++] = y+fontSize+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV;
				pTextureCoordList[textureCoordIndex++] = cyUV;

				pVertList[vertIndex++] = x+0+translate;
				pVertList[vertIndex++] = y+fontSize+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV;
				pTextureCoordList[textureCoordIndex++] = cyUV+glyphHeightUV;

				pVertList[vertIndex++] = x+0+translate;
				pVertList[vertIndex++] = y+0+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV+glyphWidthUV;
				pTextureCoordList[textureCoordIndex++] = cyUV+glyphHeightUV;
				
				pVertList[vertIndex++] = x+fontSize+translate;
				pVertList[vertIndex++] = y+0+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV+glyphWidthUV;
				pTextureCoordList[textureCoordIndex++] = cyUV;
				
				pVertList[vertIndex++] = x+fontSize+translate;
				pVertList[vertIndex++] = y+fontSize+translateDown;
				pVertList[vertIndex++] = 0;

				translate+=glyphWidth;
			}
		}

		glEnableClientState(GL_VERTEX_ARRAY);		
//		glVertexPointer(3, GL_FLOAT, num*6, pVertList);
		glVertexPointer(3, GL_FLOAT, 3*4, pVertList);

		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
//		glTexCoordPointer(2, GL_FLOAT, num*6, pTextureCoordList);
		glTexCoordPointer(2, GL_FLOAT, 2*4, pTextureCoordList);

		glDrawArrays(GL_TRIANGLES, 0, count*6);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

		delete[] pVertList;
		delete[] pTextureCoordList;

//		glMatrixMode(GL_MODELVIEW);
//		glPopMatrix();

		if(doOrthoProj)
		{
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
		}
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);	
		glDisable(GL_BLEND);
		glEnable(GL_CULL_FACE);
	}
}

void GLFontRenderer::setScreenResolution(int screenWidth, int screenHeight)
{
	m_screenWidth = screenWidth;
	m_screenHeight = screenHeight;
}

void GLFontRenderer::setColor(float r, float g, float b, float a)
{
	m_color[0] = r;
	m_color[1] = g;
	m_color[2] = b;
	m_color[3] = a;
}

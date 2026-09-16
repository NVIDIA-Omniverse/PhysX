// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SNIPPET_FONT_RENDERER_H
#define SNIPPET_FONT_RENDERER_H

class GLFontRenderer
{
private:

	static bool m_isInit;
	static unsigned int m_textureObject;
	static int m_screenWidth;
	static int m_screenHeight;
	static float m_color[4];

public:
	
	static bool init();
	static void print(float x, float y, float fontSize, const char* pString, bool forceMonoSpace=false, int monoSpaceWidth=11, bool doOrthoProj=true);
	static void print_(float x, float y, float fontSize, const char* pString, bool forceMonoSpace=false, int monoSpaceWidth=11, bool doOrthoProj=true);
	static void setScreenResolution(int screenWidth, int screenHeight);
	static void setColor(float r, float g, float b, float a);
};

#endif

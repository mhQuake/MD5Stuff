/*
Copyright (C) 1996-2001 Id Software, Inc.
Copyright (C) 2002-2009 John Fitzgibbons and others

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/
//gl_warp.c -- warping animation support

#include "quakedef.h"

extern cvar_t r_drawflat;

cvar_t r_oldwater = {"r_oldwater", "1"};
cvar_t r_waterquality = {"r_waterquality", "8"};
cvar_t r_waterwarp = {"r_waterwarp", "1"};

float load_subdivide_size; //johnfitz -- remember what subdivide_size value was when this map was loaded

float	turbsin[] =
{
	#include "gl_warp_sin.h"
};

#define WARPCALC(s,t) ((s + turbsin[(int)((t*2)+(cl.time*(128.0/M_PI))) & 255]) * (1.0/64)) //johnfitz -- correct warp
#define WARPCALC2(s,t) ((s + turbsin[(int)((t*0.125+cl.time)*(128.0/M_PI)) & 255]) * (1.0/64)) //johnfitz -- old warp

//==============================================================================
//
//  OLD-STYLE WATER
//
//==============================================================================

extern	model_t	*loadmodel;

msurface_t	*warpface;

cvar_t gl_subdivide_size = {"gl_subdivide_size", "128", true};

void BoundPoly (int numverts, float *verts, vec3_t mins, vec3_t maxs)
{
	int		i, j;
	float	*v;

	mins[0] = mins[1] = mins[2] = 9999;
	maxs[0] = maxs[1] = maxs[2] = -9999;
	v = verts;
	for (i=0 ; i<numverts ; i++)
		for (j=0 ; j<3 ; j++, v++)
		{
			if (*v < mins[j])
				mins[j] = *v;
			if (*v > maxs[j])
				maxs[j] = *v;
		}
}

void SubdividePolygon (int numverts, float *verts)
{
	int		i, j, k;
	vec3_t	mins, maxs;
	float	m;
	float	*v;
	vec3_t	front[64], back[64];
	int		f, b;
	float	dist[64];
	float	frac;
	glpoly_t	*poly;
	float	s, t;

	if (numverts > 60)
		Sys_Error ("numverts = %i", numverts);

	BoundPoly (numverts, verts, mins, maxs);

	for (i=0 ; i<3 ; i++)
	{
		m = (mins[i] + maxs[i]) * 0.5;
		m = gl_subdivide_size.value * floor (m/gl_subdivide_size.value + 0.5);
		if (maxs[i] - m < 8)
			continue;
		if (m - mins[i] < 8)
			continue;

		// cut it
		v = verts + i;
		for (j=0 ; j<numverts ; j++, v+= 3)
			dist[j] = *v - m;

		// wrap cases
		dist[j] = dist[0];
		v-=i;
		VectorCopy (verts, v);

		f = b = 0;
		v = verts;
		for (j=0 ; j<numverts ; j++, v+= 3)
		{
			if (dist[j] >= 0)
			{
				VectorCopy (v, front[f]);
				f++;
			}
			if (dist[j] <= 0)
			{
				VectorCopy (v, back[b]);
				b++;
			}
			if (dist[j] == 0 || dist[j+1] == 0)
				continue;
			if ( (dist[j] > 0) != (dist[j+1] > 0) )
			{
				// clip point
				frac = dist[j] / (dist[j] - dist[j+1]);
				for (k=0 ; k<3 ; k++)
					front[f][k] = back[b][k] = v[k] + frac*(v[3+k] - v[k]);
				f++;
				b++;
			}
		}

		SubdividePolygon (f, front[0]);
		SubdividePolygon (b, back[0]);
		return;
	}

	poly = Hunk_Alloc (sizeof(glpoly_t) + (numverts-4) * VERTEXSIZE*sizeof(float));
	poly->next = warpface->polys->next;
	warpface->polys->next = poly;
	poly->numverts = numverts;
	for (i=0 ; i<numverts ; i++, verts+= 3)
	{
		VectorCopy (verts, poly->verts[i]);
		s = DotProduct (verts, warpface->texinfo->vecs[0]);
		t = DotProduct (verts, warpface->texinfo->vecs[1]);
		poly->verts[i][3] = s;
		poly->verts[i][4] = t;
	}
}

/*
================
GL_SubdivideSurface
================
*/
void GL_SubdivideSurface (msurface_t *fa)
{
	vec3_t	verts[64];
	int		i;

	warpface = fa;

	//the first poly in the chain is the undivided poly for newwater rendering.
	//grab the verts from that.
	for (i=0; i<fa->polys->numverts; i++)
		VectorCopy (fa->polys->verts[i], verts[i]);

	SubdividePolygon (fa->polys->numverts, verts[0]);
}

/*
================
DrawWaterPoly -- johnfitz
================
*/
void DrawWaterPoly (glpoly_t *p)
{
	float	*v;
	int		i;

	if (load_subdivide_size > 48)
	{
		glBegin (GL_POLYGON);
		v = p->verts[0];
		for (i=0 ; i<p->numverts ; i++, v+= VERTEXSIZE)
		{
			glTexCoord2f (WARPCALC2(v[3],v[4]), WARPCALC2(v[4],v[3]));
			glVertex3fv (v);
		}
		glEnd ();
	}
	else
	{
		glBegin (GL_POLYGON);
		v = p->verts[0];
		for (i=0 ; i<p->numverts ; i++, v+= VERTEXSIZE)
		{
			glTexCoord2f (WARPCALC(v[3],v[4]), WARPCALC(v[4],v[3]));
			glVertex3fv (v);
		}
		glEnd ();
	}
}

//==============================================================================
//
//  RENDER-TO-FRAMEBUFFER WATER
//
//==============================================================================

/*
=============
R_UpdateWarpTextures -- johnfitz -- each frame, update warping textures
=============
*/
void R_UpdateWarpTextures (void)
{
	texture_t *tx;
	int i;
	float x, y, x2, warptess;

	if (r_oldwater.value || cl.paused || r_drawflat_cheatsafe || r_lightmap_cheatsafe)
		return;

	warptess = 128.0/CLAMP (3.0, floor(r_waterquality.value), 64.0);

	for (i=0; i<cl.worldmodel->numtextures; i++)
	{
		if (!(tx = cl.worldmodel->textures[i]))
			continue;

		if (!tx->update_warp)
			continue;

		//render warp
		GL_SetCanvas (CANVAS_WARPIMAGE);
		GL_Bind (tx->gltexture);
		for (x=0.0; x<128.0; x=x2)
		{
			x2 = x + warptess;
			glBegin (GL_TRIANGLE_STRIP);
			for (y=0.0; y<128.01; y+=warptess) // .01 for rounding errors
			{
				glTexCoord2f (WARPCALC(x,y), WARPCALC(y,x));
				glVertex2f (x,y);
				glTexCoord2f (WARPCALC(x2,y), WARPCALC(y,x2));
				glVertex2f (x2,y);
			}
			glEnd();
		}

		//copy to texture
		GL_Bind (tx->warpimage);
		glCopyTexSubImage2D (GL_TEXTURE_2D, 0, 0, 0, glx, gly+glheight-gl_warpimagesize, gl_warpimagesize, gl_warpimagesize);

		tx->update_warp = false;
	}

	//if warp render went down into sbar territory, we need to be sure to refresh it next frame
	if (gl_warpimagesize + sb_lines > glheight)
		Sbar_Changed ();

	//if viewsize is less than 100, we need to redraw the frame around the viewport
	scr_tileclear_updates = 0;
}


#ifdef UNDERWATER_WARP
// values are consistent with tests at http://forums.insideqc.com/viewtopic.php?f=3&t=5827 at 1280x960
// ==================================================================================================================================
// this is a good-quality waterwarp, consistent with software Quake, using (mostly) GL 1.1 fuctionality; the only thing not from
// GL 1.1 is GL_CLAMP_TO_EDGE, which requires GL 1.2, but that could be replaced with GL_REPEAT and the edge-handling will prevent
// it looking ugly.  GL_ARB_texture_non_power_of_two is not required but can be used if available.  the little-known and little-used
// texture object 0 is used for the screen copy target, which avoids needing to specify a texture via Fitz's TexMgr, but requires
// implementing GL_Unbind to keep texture binding states in sync.
// ==================================================================================================================================
cvar_t r_waterwarp_cycle = {"r_waterwarp_cycle", "5"};
cvar_t r_waterwarp_amp = {"r_waterwarp_amp", "150"};
cvar_t r_waterwarp_downscale = {"r_waterwarp_downscale", "1"};

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE                        0x812F
#endif

void R_InitUnderwaterWarpTexture (void)
{
	if (strstr (gl_extensions, "GL_ARB_texture_non_power_of_two") && !r_waterwarp_downscale.value)
	{
		// let them choose if they don't want this
		vid.maxwarpwidth = vid.width;
		vid.maxwarpheight = vid.height;
	}
	else
	{
		// called at startup and whenever the video mode changes
		for (vid.maxwarpwidth = 1; vid.maxwarpwidth < vid.width; vid.maxwarpwidth <<= 1);
		for (vid.maxwarpheight = 1; vid.maxwarpheight < vid.height; vid.maxwarpheight <<= 1);

		// take a power of 2 down from the screen res so that we can maintain perf if warping
		vid.maxwarpwidth >>= 1;
		vid.maxwarpheight >>= 1;
	}
}


void R_CalcUnderwaterCoords (float x, float y, float CYCLE_X, float CYCLE_Y, float AMP_X, float AMP_Y)
{
	// vkQuake is actually 99% of the way there; all that it's missing is a sign flip and it's AMP is half what it should be.
	// !!!!! this was correct at the time i originally wrote this; vkQuake may have been changed since then !!!!!
	// this is now extremely close to consistency with mankrip's tests
	const float texX = (x - (sin (y * CYCLE_X + cl.time) * AMP_X)) * (1.0f - AMP_X * 2.0f) + AMP_X;
	const float texY = (y - (sin (x * CYCLE_Y + cl.time) * AMP_Y)) * (1.0f - AMP_Y * 2.0f) + AMP_Y;

	glTexCoord2f (texX, texY);
	glVertex2f (x * 2 - 1, y * 2 - 1);
}


void R_WarpScreen (void)
{
	int x, y;

	// ripped this from vkQuake at https://github.com/Novum/vkQuake/blob/master/Shaders/screen_warp.comp
	// !!!!! this was the version used in vkQuake at the time i originally wrote this; it may have been changed since then !!!!!
	// our x and y coords are already incoming at 0..1 range so we don't need to rescale them.
	const float aspect = (float) r_refdef.vrect.width / (float) r_refdef.vrect.height;
	const float CYCLE_X = M_PI * r_waterwarp_cycle.value; // tune or cvarize as you wish
	const float CYCLE_Y = CYCLE_X * aspect;
	const float AMP_X = 1.0f / r_waterwarp_amp.value; // tune or cvarize as you wish
	const float AMP_Y = AMP_X * aspect;

	// copy over the texture
	GL_Unbind (); // binds texture object 0
	glCopyTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, vid.maxwarpwidth, vid.maxwarpheight, 0);

	// because we're using glCopyTexImage2D we need to specify this to satisfy OpenGL texture completeness rules
	// this should always be GL_LINEAR irrespective of gl_texturemode so that the sine warp looks correct
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// see note on GL_CLAMP_TO_EDGE above; this could be replaced with GL_REPEAT if you wish to keep pure GL 1.1; there would be no visual degradation
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	// switch vp, ortho, mvp, etc; this should be the same viewport rect as is set in the main glViewport call for the 3D scene; if you've changed it from
	// the stock Fitz code, you should change this too to match.
	glViewport (glx + r_refdef.vrect.x,
				gly + glheight - r_refdef.vrect.y - r_refdef.vrect.height,
				r_refdef.vrect.width,
				r_refdef.vrect.height);

	// in a shader you wouldn't need to bother doing this; the positions would be pass-through to output
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();

	// in a shader you wouldn't need to bother doing this; the positions would be pass-through to output
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();

	glDisable (GL_DEPTH_TEST);
	glDisable (GL_CULL_FACE);
	glDisable (GL_BLEND);
	glDisable (GL_ALPHA_TEST);
	glColor4f (1, 1, 1, 1);

	// draw the warped view; tune or cvarize this all you wish; maybe you'll create an r_underwaterwarpquality cvar?
	// yeah, it's a lot of verts - so what?  There's far far more pixels than vertexes so vertex count isn't that big a deal here.
	for (x = 0; x < 32; x++)
	{
		glBegin (GL_TRIANGLE_STRIP);

		for (y = 0; y <= 32; y++)
		{
			R_CalcUnderwaterCoords ((float) x / 32.0f, (float) y / 32.0f, CYCLE_X, CYCLE_Y, AMP_X, AMP_Y);
			R_CalcUnderwaterCoords ((float) (x + 1) / 32.0f, (float) y / 32.0f, CYCLE_X, CYCLE_Y, AMP_X, AMP_Y);
		}

		glEnd ();
	}

	// if viewsize is less than 100, we need to redraw the frame around the viewport
	scr_tileclear_updates = 0;
}
#endif


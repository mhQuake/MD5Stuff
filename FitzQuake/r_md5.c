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

//r_alias.c -- alias model rendering

#include "quakedef.h"

extern qboolean mtexenabled; //johnfitz
extern cvar_t r_drawflat, gl_overbright_models, gl_fullbrights, r_lerpmodels, r_lerpmove; //johnfitz

extern vec3_t	lightcolor; //johnfitz -- replaces "float shadelight" for lit support
extern vec3_t	shadevector;

// alias/md5 shared functions
void R_SetupEntityTransform (entity_t *e, lerpdata_t *lerpdata);
void R_SetupAliasLighting (entity_t	*e);


/*
=================
R_SetupMD5Frame

this is largely common with R_SetupAliasFrame and you might wish to refactor around that
=================
*/
void R_SetupMD5Frame (int frame, lerpdata_t *lerpdata)
{
	entity_t		*e = currententity;
	int				posenum, numposes;

	// MD5s don't have frame auto-animation
	posenum = frame;
	numposes = 1;
	e->lerptime = 0.1;

	if (e->lerpflags & LERP_RESETANIM) //kill any lerp in progress
	{
		e->lerpstart = 0;
		e->previouspose = posenum;
		e->currentpose = posenum;
		e->lerpflags -= LERP_RESETANIM;
	}
	else if (e->currentpose != posenum) // pose changed, start new lerp
	{
		if (e->lerpflags & LERP_RESETANIM2) //defer lerping one more time
		{
			e->lerpstart = 0;
			e->previouspose = posenum;
			e->currentpose = posenum;
			e->lerpflags -= LERP_RESETANIM2;
		}
		else
		{
			e->lerpstart = cl.time;
			e->previouspose = e->currentpose;
			e->currentpose = posenum;
		}
	}

	//set up values
	if (r_lerpmodels.value && !(e->model->flags & MOD_NOLERP && r_lerpmodels.value != 2))
	{
		if (e->lerpflags & LERP_FINISH && numposes == 1)
			lerpdata->blend = CLAMP (0, (cl.time - e->lerpstart) / (e->lerpfinish - e->lerpstart), 1);
		else
			lerpdata->blend = CLAMP (0, (cl.time - e->lerpstart) / e->lerptime, 1);
		lerpdata->pose1 = e->previouspose;
		lerpdata->pose2 = e->currentpose;
	}
	else //don't lerp
	{
		lerpdata->blend = 1;
		lerpdata->pose1 = posenum;
		lerpdata->pose2 = posenum;
	}
}


/*
==================
MD5_InterpolateSkeletons

==================
*/
void MD5_InterpolateSkeletons (const struct md5_joint_t *skelA, const struct md5_joint_t *skelB, int num_joints, float interp, struct md5_joint_t *out)
{
	int i;

	for (i = 0; i < num_joints; ++i)
	{
		// Copy parent index
		out[i].parent = skelA[i].parent;

		// Linear interpolation for position
		out[i].pos[0] = skelA[i].pos[0] + interp * (skelB[i].pos[0] - skelA[i].pos[0]);
		out[i].pos[1] = skelA[i].pos[1] + interp * (skelB[i].pos[1] - skelA[i].pos[1]);
		out[i].pos[2] = skelA[i].pos[2] + interp * (skelB[i].pos[2] - skelA[i].pos[2]);

		// Spherical linear interpolation for orientation
		Quat_slerp (skelA[i].orient, skelB[i].orient, interp, out[i].orient);
	}
}


/*
==================
MD5_PrepareMesh

==================
*/
static void MD5_PrepareMesh (const struct md5_mesh_t *mesh, const struct md5_joint_t *skeleton, md5polyvert_t *vertexes)
{
	int i, j;

	// Setup vertices
	for (i = 0; i < mesh->num_verts; i++)
	{
		vec3_t finalVertex = {0.0f, 0.0f, 0.0f};
		float Angle;

		// Calculate final vertex to draw with weights
		for (j = 0; j < mesh->vertices[i].count; j++)
		{
			const struct md5_weight_t *weight = &mesh->weights[mesh->vertices[i].start + j];
			const struct md5_joint_t *joint = &skeleton[weight->joint];

			// Calculate transformed vertex for this weight
			vec3_t wv;
			Quat_rotatePoint (joint->orient, weight->pos, wv);

			// The sum of all weight->bias should be 1.0
			finalVertex[0] += (joint->pos[0] + wv[0]) * weight->bias;
			finalVertex[1] += (joint->pos[1] + wv[1]) * weight->bias;
			finalVertex[2] += (joint->pos[2] + wv[2]) * weight->bias;
		}

		// store out position
		vertexes[i].position[0] = finalVertex[0];
		vertexes[i].position[1] = finalVertex[1];
		vertexes[i].position[2] = finalVertex[2];

		// normalise the position as a cheesy normal - this effectively projects the position out to a point on a unit sphere centered on the model origin,
		// which is a really cheesy substitute for real normals, but Quake MDL lighting never pretended to be ultra-accurate anyway
		VectorNormalize (finalVertex);

		// this calc isn't correct per-theory but it matches with the calc used by light.exe and qrad.exe
		Angle = DotProduct (finalVertex, shadevector);

		/*if (Angle < 0)
			Angle = 0.5f;
		else*/ //Angle = Angle * 0.5f + 0.5f;

		// store out colour
		vertexes[i].colour[0] = lightcolor[0] * Angle;
		vertexes[i].colour[1] = lightcolor[1] * Angle;
		vertexes[i].colour[2] = lightcolor[2] * Angle;
		vertexes[i].colour[3] = 1;
	}
}


void R_SetupMD5SkinTextures (entity_t *e, md5header_t *hdr)
{
	// auto-animation
	md5skin_t *skin = &hdr->skins[e->skinnum % hdr->numskins];
	skinpair_t *image = &skin->image[(int) ((cl.time + e->syncbase) * 10) % skin->numskins];

	gltexture_t *tx = image->tx;
	gltexture_t *fb = image->fb;

	if (!gl_fullbrights.value)
		fb = NULL;

	GL_DisableMultitexture ();
	GL_Bind (tx);
}


void R_DrawMD5Model (entity_t *e)
{
	md5header_t *hdr = (md5header_t *) e->model->cache.data;
	lerpdata_t	lerpdata;

	R_SetupMD5Frame (e->frame, &lerpdata);
	R_SetupEntityTransform (e, &lerpdata);

	// cull it - do this yourself
	//if (R_CullModelForEntity (e))
	//	return;

	// transform it
    glPushMatrix ();
	R_RotateForEntity (lerpdata.origin, lerpdata.angles);

	// set up lighting
	rs_aliaspolys += hdr->numindexes / 3;
	R_SetupAliasLighting (e);

	// textures
	R_SetupMD5SkinTextures (e, hdr);

	// run the skeletal animation
	MD5_InterpolateSkeletons (hdr->md5anim.skelFrames[lerpdata.pose1], hdr->md5anim.skelFrames[lerpdata.pose2], hdr->md5anim.num_joints, lerpdata.blend, hdr->skeleton);

	// set up the vertex array
	MD5_PrepareMesh (&hdr->md5mesh.meshes[0], hdr->skeleton, hdr->vertexes);

	// set up arrays
	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_COLOR_ARRAY);
	glEnableClientState (GL_TEXTURE_COORD_ARRAY);

	// set up pointers
	glVertexPointer (3, GL_FLOAT, sizeof (md5polyvert_t), hdr->vertexes->position);
	glColorPointer (4, GL_FLOAT, sizeof (md5polyvert_t), hdr->vertexes->colour);
	glTexCoordPointer (2, GL_FLOAT, sizeof (md5polyvert_t), hdr->vertexes->texcoord);

	glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// draw it
	glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);

	glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	// shut down arrays
	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_COLOR_ARRAY);
	glDisableClientState (GL_TEXTURE_COORD_ARRAY);

	glPopMatrix ();

	// current colour is undefined after using GL_COLOR_ARRAY so define it again
	glColor3f (1, 1, 1);
}



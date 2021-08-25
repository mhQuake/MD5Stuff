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

extern qboolean mtexenabled, overbright; //johnfitz
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
static void MD5_PrepareMesh (const struct md5_mesh_t *mesh, const struct md5_joint_t *skeleton, md5polyvert_t *vertexes, vertexnormals_t *vnorms)
{
	int i, j;

	// Setup vertices
	for (i = 0; i < mesh->num_verts; i++)
	{
		vec3_t finalVertex = {0.0f, 0.0f, 0.0f};

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
	}

	for (i = 0; i < mesh->num_verts; i++)
	{
		// no normals initially
		vnorms[i].normal[0] = vnorms[i].normal[1] = vnorms[i].normal[2] = 0;
		vnorms[i].numnormals = 0;
	}

	for (i = 0; i < mesh->num_tris; i++)
	{
		float triverts[3][3];
		float vtemp1[3], vtemp2[3], normal[3];

		// undo the vertex rotation from modelgen.c here too
		for (j = 0; j < 3; j++)
		{
			triverts[j][0] = vertexes[mesh->triangles[i].index[j]].position[1];
			triverts[j][1] = -vertexes[mesh->triangles[i].index[j]].position[0];
			triverts[j][2] = vertexes[mesh->triangles[i].index[j]].position[2];
		}

		// calc the per-triangle normal
		VectorSubtract (triverts[0], triverts[1], vtemp1);
		VectorSubtract (triverts[2], triverts[1], vtemp2);
		CrossProduct (vtemp1, vtemp2, normal);
		VectorNormalize (normal);

		// and accumulate it into the calculated normals array
		for (j = 0; j < 3; j++)
		{
			// rotate the normal so the model faces down the positive x axis
			vnorms[mesh->triangles[i].index[j]].normal[0] -= normal[1];
			vnorms[mesh->triangles[i].index[j]].normal[1] += normal[0];
			vnorms[mesh->triangles[i].index[j]].normal[2] += normal[2];

			// count the normals for averaging
			vnorms[mesh->triangles[i].index[j]].numnormals++;
		}
	}

	// calculate final normal and set up lighting
	for (i = 0; i < mesh->num_verts; i++)
	{
		float Angle;

		// numnormals was checked for > 0 in modelgen.c so we shouldn't need to do it again 
		// here but we do anyway just in case a rogue modder has used a bad modelling tool
		if (vnorms[i].numnormals > 0)
		{
			VectorScale (vnorms[i].normal, (float) vnorms[i].numnormals, vnorms[i].normal);
			VectorNormalize (vnorms[i].normal);
		}
		else
		{
			vnorms[i].normal[0] = vnorms[i].normal[1] = 0;
			vnorms[i].normal[2] = 1;
		}

		// this calc isn't correct per-theory but it matches with the calc used by light.exe and qrad.exe
		if ((Angle = DotProduct (vnorms[i].normal, shadevector)) < 0)
			Angle = 0.5f;
		else Angle = Angle * 0.5f + 0.5f;

		// store out colour
		vertexes[i].colour[0] = lightcolor[0] * Angle;
		vertexes[i].colour[1] = lightcolor[1] * Angle;
		vertexes[i].colour[2] = lightcolor[2] * Angle;
		vertexes[i].colour[3] = 1;
	}
}


skinpair_t *R_SetupMD5SkinTextures (entity_t *e, md5header_t *hdr)
{
	// auto-animation
	md5skin_t *skin = &hdr->skins[e->skinnum % hdr->numskins];
	return &skin->image[(int) ((cl.time + e->syncbase) * 10) % skin->numskins];
}


qboolean R_CullMD5Model (lerpdata_t lerpdata, const struct md5_anim_t *anim)
{
	float mins[3] = {
		anim->bboxes[lerpdata.pose1].min[0] * (1.0f - lerpdata.blend) + anim->bboxes[lerpdata.pose2].min[0] * lerpdata.blend + lerpdata.origin[0],
		anim->bboxes[lerpdata.pose1].min[1] * (1.0f - lerpdata.blend) + anim->bboxes[lerpdata.pose2].min[1] * lerpdata.blend + lerpdata.origin[1],
		anim->bboxes[lerpdata.pose1].min[2] * (1.0f - lerpdata.blend) + anim->bboxes[lerpdata.pose2].min[2] * lerpdata.blend + lerpdata.origin[2]
	};

	float maxs[3] = {
		anim->bboxes[lerpdata.pose1].max[0] * (1.0f - lerpdata.blend) + anim->bboxes[lerpdata.pose2].max[0] * lerpdata.blend + lerpdata.origin[0],
		anim->bboxes[lerpdata.pose1].max[1] * (1.0f - lerpdata.blend) + anim->bboxes[lerpdata.pose2].max[1] * lerpdata.blend + lerpdata.origin[1],
		anim->bboxes[lerpdata.pose1].max[2] * (1.0f - lerpdata.blend) + anim->bboxes[lerpdata.pose2].max[2] * lerpdata.blend + lerpdata.origin[2]
	};

	return R_CullBox (mins, maxs);
}


void R_DrawMD5Model (entity_t *e)
{
	md5header_t *hdr = (md5header_t *) e->model->cache.data;
	lerpdata_t	lerpdata;
	skinpair_t *image;

	R_SetupMD5Frame (e->frame, &lerpdata);
	R_SetupEntityTransform (e, &lerpdata);

	// cull it
	if (e != &cl.viewent)
		if (R_CullMD5Model (lerpdata, &hdr->md5anim))
			return;

	// transform it
    glPushMatrix ();
	R_RotateForEntity (lerpdata.origin, lerpdata.angles);

	// set up lighting
	overbright = gl_overbright_models.value;
	rs_aliaspolys += hdr->numindexes / 3;
	R_SetupAliasLighting (e);

	// set up shadevector from the specified yaw angle
	shadevector[0] = cos (-(e->angles[1] / 180 * M_PI));
	shadevector[1] = sin (-(e->angles[1] / 180 * M_PI));
	shadevector[2] = 1;
	VectorNormalize (shadevector);

	// textures
	image = R_SetupMD5SkinTextures (e, hdr);

	GL_DisableMultitexture ();
	GL_Bind (image->tx);

	// run the skeletal animation
	MD5_InterpolateSkeletons (hdr->md5anim.skelFrames[lerpdata.pose1], hdr->md5anim.skelFrames[lerpdata.pose2], hdr->md5anim.num_joints, lerpdata.blend, hdr->skeleton);

	// set up the vertex array
	MD5_PrepareMesh (&hdr->md5mesh.meshes[0], hdr->skeleton, hdr->vertexes, hdr->vnorms);

	// set up arrays
	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_COLOR_ARRAY);
	glEnableClientState (GL_TEXTURE_COORD_ARRAY);

	// set up pointers
	glVertexPointer (3, GL_FLOAT, sizeof (md5polyvert_t), hdr->vertexes->position);
	glColorPointer (4, GL_FLOAT, sizeof (md5polyvert_t), hdr->vertexes->colour);
	glTexCoordPointer (2, GL_FLOAT, sizeof (md5polyvert_t), hdr->vertexes->texcoord);

	if (gl_smoothmodels.value && !r_drawflat_cheatsafe)
		glShadeModel (GL_SMOOTH);
	if (gl_affinemodels.value)
		glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);

	// draw it
	if (r_drawflat_cheatsafe)
	{
		glDisable (GL_TEXTURE_2D);
		glShadeModel (GL_FLAT);
		glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);
		glEnable (GL_TEXTURE_2D);
	}
	else if (r_fullbright_cheatsafe)
	{
		glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);
	}
	else if (r_lightmap_cheatsafe)
	{
		glDisable (GL_TEXTURE_2D);
		glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);
		glEnable (GL_TEXTURE_2D);
	}
	else
	{
		if (overbright)
		{
			if (gl_texture_env_combine)
			{
				// single-pass it
				glTexEnvi (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE_EXT);
				glTexEnvi (GL_TEXTURE_ENV, GL_COMBINE_RGB_EXT, GL_MODULATE);
				glTexEnvi (GL_TEXTURE_ENV, GL_SOURCE0_RGB_EXT, GL_TEXTURE);
				glTexEnvi (GL_TEXTURE_ENV, GL_SOURCE1_RGB_EXT, GL_PRIMARY_COLOR_EXT);
				glTexEnvf (GL_TEXTURE_ENV, GL_RGB_SCALE_EXT, 2.0f);

				glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);

				glTexEnvf (GL_TEXTURE_ENV, GL_RGB_SCALE_EXT, 1.0f);
				glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			}
			else
			{
				// first pass
				glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
				glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);

				// second pass
				glEnable (GL_BLEND);
				glBlendFunc (GL_ONE, GL_ONE);
				glDepthMask (GL_FALSE);

				Fog_StartAdditive ();
				glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);
				Fog_StopAdditive ();

				glDepthMask (GL_TRUE);
				glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glDisable (GL_BLEND);
			}
		}
		else
		{
			// one pass only
			glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);
		}

		// fullbright mask (if present)
		if (image->fb && gl_fullbrights.value)
		{
			GL_Bind (image->fb);
			glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			glEnable (GL_BLEND);
			glBlendFunc (GL_ONE, GL_ONE);
			glDepthMask (GL_FALSE);

			Fog_StartAdditive ();
			glDrawElements (GL_TRIANGLES, hdr->numindexes, GL_UNSIGNED_SHORT, hdr->indexes);
			Fog_StopAdditive ();

			glDepthMask (GL_TRUE);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable (GL_BLEND);
		}
	}

	// revert state
	glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glShadeModel (GL_FLAT);
	glDepthMask (GL_TRUE);
	glDisable (GL_BLEND);

	// shut down arrays
	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_COLOR_ARRAY);
	glDisableClientState (GL_TEXTURE_COORD_ARRAY);

	glPopMatrix ();

	// current colour is undefined after using GL_COLOR_ARRAY so define it again
	glColor3f (1, 1, 1);
}



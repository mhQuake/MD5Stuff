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
// models.c -- model loading and caching

// models are the only shared resource between a client and server running
// on the same machine.

#include "quakedef.h"

extern model_t *loadmodel;


/*
==================
MD5_BuildBaseNormals

==================
*/
void MD5_BuildBaseNormals (md5header_t *hdr, struct md5_mesh_t *mesh)
{
	// allocate memory for vertexes
	md5polyvert_t *vertexes = (md5polyvert_t *) Hunk_Alloc (sizeof (md5polyvert_t) * mesh->num_verts);

	// allocate memory for normals
	vertexnormals_t *vnorms = (vertexnormals_t *) Hunk_Alloc (sizeof (vertexnormals_t) * mesh->num_verts);

	// get rhe rest of the data we need
	const struct md5_joint_t *skeleton = hdr->md5mesh.baseSkel;

	int i, j;

	// Setup vertices for the base frame
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

	// no normals initially
	for (i = 0; i < mesh->num_verts; i++)
	{
		vnorms[i].normal[0] = vnorms[i].normal[1] = vnorms[i].normal[2] = 0;
		vnorms[i].numnormals = 0;
	}

	// accumulate triangle normals to vnorms
	for (i = 0; i < mesh->num_tris; i++)
	{
		float triverts[3][3];
		float vtemp1[3], vtemp2[3], normal[3];

		// undo the vertex rotation from modelgen.c here
		// i don't know why id did it this way, but i decided to adapt it unmodified for consistency with modelgen.c
		for (j = 0; j < 3; j++)
		{
			triverts[j][0] = vertexes[mesh->triangles[i].vertindex[j]].position[1];
			triverts[j][1] = -vertexes[mesh->triangles[i].vertindex[j]].position[0];
			triverts[j][2] = vertexes[mesh->triangles[i].vertindex[j]].position[2];
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
			// i don't know why id did it this way, but i decided to adapt it unmodified for consistency with modelgen.c
			vnorms[mesh->triangles[i].vertindex[j]].normal[0] -= normal[1];
			vnorms[mesh->triangles[i].vertindex[j]].normal[1] += normal[0];
			vnorms[mesh->triangles[i].vertindex[j]].normal[2] += normal[2];

			// count the normals for averaging
			vnorms[mesh->triangles[i].vertindex[j]].numnormals++;
		}
	}

	// calculate final normals
	for (i = 0; i < mesh->num_verts; i++)
	{
		vec3_t finalNormal = {0.0f, 0.0f, 0.0f};

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

		// this normal is now in object space for the base frame so rotate it back to joint-local space
		for (j = 0; j < mesh->vertices[i].count; j++)
		{
			const struct md5_weight_t *weight = &mesh->weights[mesh->vertices[i].start + j];
			const struct md5_joint_t *joint = &skeleton[weight->joint];

			// Calculate transformed normal for this weight - inverse rotate it back to joint-local space
			vec3_t wv;
			Quat_inverseRotatePoint (joint->orient, vnorms[i].normal, wv);

			// The sum of all weight->bias should be 1.0
			finalNormal[0] += wv[0] * weight->bias;
			finalNormal[1] += wv[1] * weight->bias;
			finalNormal[2] += wv[2] * weight->bias;
		}

		// store out the final normal in joint-local space
		vnorms[i].normal[0] = finalNormal[0];
		vnorms[i].normal[1] = finalNormal[1];
		vnorms[i].normal[2] = finalNormal[2];
	}

	// store the vertexes and normals out in the MD5 header now
	hdr->vertexes = vertexes;
	hdr->vnorms = vnorms;
}


/*
==================
MD5_ReadMeshFile

==================
*/
static int MD5_ReadMeshFile (char *filename, struct md5_model_t *mdl)
{
	int version;
	int curr_mesh = 0;
	int i;
	char *data = (char *) COM_LoadHunkFile (filename);

	if (!data)
	{
		Con_DPrintf ("MD5_ReadMeshFile : couldn't open \"%s\"!\n", filename);
		return 0;
	}

	// Read whole line
	while ((data = COM_ParseLine (data)) != NULL)
	{
		if (sscanf (com_token, " MD5Version %d", &version) == 1)
		{
			if (version != 10)
			{
				// Bad version
				Con_DPrintf ("MD5_ReadMeshFile : bad model version for \"%s\"\n", filename);
				return 0;
			}
		}
		else if (sscanf (com_token, " numJoints %d", &mdl->num_joints) == 1)
		{
			if (mdl->num_joints > 0)
			{
				// Allocate memory for base skeleton joints
				mdl->baseSkel = (struct md5_joint_t *) Hunk_Alloc (mdl->num_joints * sizeof (struct md5_joint_t));
			}
		}
		else if (sscanf (com_token, " numMeshes %d", &mdl->num_meshes) == 1)
		{
			if (mdl->num_meshes > 0)
			{
				// Allocate memory for meshes
				mdl->meshes = (struct md5_mesh_t *) Hunk_Alloc (mdl->num_meshes * sizeof (struct md5_mesh_t));
			}
		}
		else if (strncmp (com_token, "joints {", 8) == 0)
		{
			// Read each joint
			for (i = 0; i < mdl->num_joints; i++)
			{
				struct md5_joint_t *joint = &mdl->baseSkel[i];

				// Read whole line
				if ((data = COM_ParseLine (data)) == NULL) return 0;

				if (sscanf (com_token, "%s %d ( %f %f %f ) ( %f %f %f )", joint->name, &joint->parent, &joint->pos[0], &joint->pos[1], &joint->pos[2], &joint->orient[0], &joint->orient[1], &joint->orient[2]) == 8)
				{
					// Compute the w component
					Quat_computeW (joint->orient);
				}
			}
		}
		else if (strncmp (com_token, "mesh {", 6) == 0)
		{
			struct md5_mesh_t *mesh = &mdl->meshes[curr_mesh];
			int vert_index = 0;
			int tri_index = 0;
			int weight_index = 0;
			float fdata[4];
			int idata[3];

			while (com_token[0] != '}')
			{
				// Read whole line
				if ((data = COM_ParseLine (data)) == NULL) break;

				if (strstr (com_token, "shader "))
				{
					int quote = 0, j = 0;

					// Copy the shader name whithout the quote marks
					for (i = 0; i < strlen (com_token) && (quote < 2); i++)
					{
						if (com_token[i] == '\"')
							quote++;

						if ((quote == 1) && (com_token[i] != '\"'))
						{
							mesh->shader[j] = com_token[i];
							j++;
						}
					}
				}
				else if (sscanf (com_token, " numverts %d", &mesh->num_verts) == 1)
				{
					if (mesh->num_verts > 0)
					{
						// Allocate memory for vertices
						mesh->vertices = (struct md5_vertex_t *) Hunk_Alloc (sizeof (struct md5_vertex_t) * mesh->num_verts);
					}
				}
				else if (sscanf (com_token, " numtris %d", &mesh->num_tris) == 1)
				{
					if (mesh->num_tris > 0)
					{
						// Allocate memory for triangles
						mesh->triangles = (mtriangle_t *) Hunk_Alloc (sizeof (mtriangle_t) * mesh->num_tris);
					}
				}
				else if (sscanf (com_token, " numweights %d", &mesh->num_weights) == 1)
				{
					if (mesh->num_weights > 0)
					{
						// Allocate memory for vertex weights
						mesh->weights = (struct md5_weight_t *) Hunk_Alloc (sizeof (struct md5_weight_t) * mesh->num_weights);
					}
				}
				else if (sscanf (com_token, " vert %d ( %f %f ) %d %d", &vert_index, &fdata[0], &fdata[1], &idata[0], &idata[1]) == 5)
				{
					// Copy vertex data
					mesh->vertices[vert_index].st[0] = fdata[0];
					mesh->vertices[vert_index].st[1] = fdata[1];
					mesh->vertices[vert_index].start = idata[0];
					mesh->vertices[vert_index].count = idata[1];
				}
				else if (sscanf (com_token, " tri %d %d %d %d", &tri_index, &idata[0], &idata[1], &idata[2]) == 4)
				{
					// Copy triangle data
					mesh->triangles[tri_index].vertindex[0] = idata[0];
					mesh->triangles[tri_index].vertindex[1] = idata[1];
					mesh->triangles[tri_index].vertindex[2] = idata[2];
					mesh->triangles[tri_index].facesfront = 0; // always for MD5s
				}
				else if (sscanf (com_token, " weight %d %d %f ( %f %f %f )", &weight_index, &idata[0], &fdata[3], &fdata[0], &fdata[1], &fdata[2]) == 6)
				{
					// Copy vertex data
					mesh->weights[weight_index].joint = idata[0];
					mesh->weights[weight_index].bias = fdata[3];
					mesh->weights[weight_index].pos[0] = fdata[0];
					mesh->weights[weight_index].pos[1] = fdata[1];
					mesh->weights[weight_index].pos[2] = fdata[2];
				}
			}

			curr_mesh++;
		}
	}

	return 1;
}


/*
==================
MD5_BuildFrameSkeleton

Build skeleton for a given frame data.
==================
*/
static void MD5_BuildFrameSkeleton (const struct joint_info_t *jointInfos, const struct baseframe_joint_t *baseFrame, const float *animFrameData, struct md5_joint_t *skelFrame, int num_joints)
{
	int i;

	for (i = 0; i < num_joints; i++)
	{
		const struct baseframe_joint_t *baseJoint = &baseFrame[i];
		vec3_t animatedPos;
		quat4_t animatedOrient;
		int j = 0;

		memcpy (animatedPos, baseJoint->pos, sizeof (vec3_t));
		memcpy (animatedOrient, baseJoint->orient, sizeof (quat4_t));

		if (jointInfos[i].flags & 1) // Tx
		{
			animatedPos[0] = animFrameData[jointInfos[i].startIndex + j];
			j++;
		}

		if (jointInfos[i].flags & 2) // Ty
		{
			animatedPos[1] = animFrameData[jointInfos[i].startIndex + j];
			j++;
		}

		if (jointInfos[i].flags & 4) // Tz
		{
			animatedPos[2] = animFrameData[jointInfos[i].startIndex + j];
			j++;
		}

		if (jointInfos[i].flags & 8) // Qx
		{
			animatedOrient[0] = animFrameData[jointInfos[i].startIndex + j];
			j++;
		}

		if (jointInfos[i].flags & 16) // Qy
		{
			animatedOrient[1] = animFrameData[jointInfos[i].startIndex + j];
			j++;
		}

		if (jointInfos[i].flags & 32) // Qz
		{
			animatedOrient[2] = animFrameData[jointInfos[i].startIndex + j];
			j++;
		}

		// Compute orient quaternion's w value
		Quat_computeW (animatedOrient);

		// NOTE: we assume that this joint's parent has already been calculated, i.e. joint's ID should never be smaller than its parent ID.
		{
		struct md5_joint_t *thisJoint = &skelFrame[i];

		int parent = jointInfos[i].parent;
		thisJoint->parent = parent;
		strcpy (thisJoint->name, jointInfos[i].name);

		// Has parent?
		if (thisJoint->parent < 0)
		{
			memcpy (thisJoint->pos, animatedPos, sizeof (vec3_t));
			memcpy (thisJoint->orient, animatedOrient, sizeof (quat4_t));
		}
		else
		{
			struct md5_joint_t *parentJoint = &skelFrame[parent];
			vec3_t rpos; // Rotated position

			// Add positions
			Quat_rotatePoint (parentJoint->orient, animatedPos, rpos);
			thisJoint->pos[0] = rpos[0] + parentJoint->pos[0];
			thisJoint->pos[1] = rpos[1] + parentJoint->pos[1];
			thisJoint->pos[2] = rpos[2] + parentJoint->pos[2];

			// Concatenate rotations
			Quat_multQuat (parentJoint->orient, animatedOrient, thisJoint->orient);
			Quat_normalize (thisJoint->orient);
		}
		}
	}
}


/*
==================
MD5_ReadAnimFile

Load an MD5 animation from file.
==================
*/
static int MD5_ReadAnimFile (char *filename, struct md5_anim_t *anim)
{
	struct joint_info_t *jointInfos = NULL;
	struct baseframe_joint_t *baseFrame = NULL;
	float *animFrameData = NULL;
	int version;
	int numAnimatedComponents;
	int frame_index;
	int i;
	char *data = (char *) COM_LoadHunkFile (filename);

	if (!data)
	{
		Con_DPrintf ("error: couldn't open \"%s\"!\n", filename);
		return 0;
	}

	// Read whole line
	while ((data = COM_ParseLine (data)) != NULL)
	{
		if (sscanf (com_token, " MD5Version %d", &version) == 1)
		{
			if (version != 10)
			{
				// Bad version
				Con_DPrintf ("Error: bad animation version for \"%s\"\n", filename);
				return 0;
			}
		}
		else if (sscanf (com_token, " numFrames %d", &anim->num_frames) == 1)
		{
			// Allocate memory for skeleton frames and bounding boxes
			if (anim->num_frames > 0)
			{
				anim->skelFrames = (struct md5_joint_t **) Hunk_Alloc (sizeof (struct md5_joint_t *) * anim->num_frames);
				anim->bboxes = (struct md5_bbox_t *) Hunk_Alloc (sizeof (struct md5_bbox_t) * anim->num_frames);
			}
		}
		else if (sscanf (com_token, " numJoints %d", &anim->num_joints) == 1)
		{
			if (anim->num_joints > 0)
			{
				for (i = 0; i < anim->num_frames; i++)
				{
					// Allocate memory for joints of each frame
					anim->skelFrames[i] = (struct md5_joint_t *) Hunk_Alloc (sizeof (struct md5_joint_t) * anim->num_joints);
				}

				// Allocate temporary memory for building skeleton frames
				jointInfos = (struct joint_info_t *) Hunk_Alloc (sizeof (struct joint_info_t) * anim->num_joints);
				baseFrame = (struct baseframe_joint_t *) Hunk_Alloc (sizeof (struct baseframe_joint_t) * anim->num_joints);
			}
		}
		else if (sscanf (com_token, " frameRate %d", &anim->frameRate) == 1)
			; // unused in this code
		else if (sscanf (com_token, " numAnimatedComponents %d", &numAnimatedComponents) == 1)
		{
			if (numAnimatedComponents > 0)
			{
				// Allocate memory for animation frame data
				animFrameData = (float *) Hunk_Alloc (sizeof (float) * numAnimatedComponents);
			}
		}
		else if (strncmp (com_token, "hierarchy {", 11) == 0)
		{
			for (i = 0; i < anim->num_joints; i++)
			{
				// Read whole line
				if ((data = COM_ParseLine (data)) == NULL) return 0;

				// Read joint info
				sscanf (com_token, " %s %d %d %d", jointInfos[i].name, &jointInfos[i].parent, &jointInfos[i].flags, &jointInfos[i].startIndex);
			}
		}
		else if (strncmp (com_token, "bounds {", 8) == 0)
		{
			for (i = 0; i < anim->num_frames; i++)
			{
				// Read whole line
				if ((data = COM_ParseLine (data)) == NULL) return 0;

				// Read bounding box
				sscanf (com_token, " ( %f %f %f ) ( %f %f %f )", &anim->bboxes[i].min[0], &anim->bboxes[i].min[1], &anim->bboxes[i].min[2], &anim->bboxes[i].max[0], &anim->bboxes[i].max[1], &anim->bboxes[i].max[2]);
			}
		}
		else if (strncmp (com_token, "baseframe {", 10) == 0)
		{
			for (i = 0; i < anim->num_joints; i++)
			{
				// Read whole line
				if ((data = COM_ParseLine (data)) == NULL) return 0;

				// Read base frame joint
				if (sscanf (com_token, " ( %f %f %f ) ( %f %f %f )", &baseFrame[i].pos[0], &baseFrame[i].pos[1], &baseFrame[i].pos[2], &baseFrame[i].orient[0], &baseFrame[i].orient[1], &baseFrame[i].orient[2]) == 6)
				{
					// Compute the w component
					Quat_computeW (baseFrame[i].orient);
				}
			}
		}
		else if (sscanf (com_token, " frame %d", &frame_index) == 1)
		{
			// Read frame data
			for (i = 0; i < numAnimatedComponents; i++)
			{
				// parse a single token
				if ((data = COM_Parse (data)) == NULL) return 0;
				animFrameData[i] = atof (com_token);
			}

			// Build frame skeleton from the collected data
			MD5_BuildFrameSkeleton (jointInfos, baseFrame, animFrameData, anim->skelFrames[frame_index], anim->num_joints);
		}
	}

	return 1;
}


/*
==================
MD5_CullboxForFrame

==================
*/
static void MD5_CullboxForFrame (const struct md5_mesh_t *mesh, const struct md5_joint_t *skeleton, struct md5_bbox_t *cullbox)
{
	int i, j;

	// init this cullbox
	cullbox->min[0] = cullbox->min[1] = cullbox->min[2] = 999999;
	cullbox->max[0] = cullbox->max[1] = cullbox->max[2] = -999999;

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

		// accumulate to the cullbox
		for (j = 0; j < 3; j++)
		{
			if (finalVertex[j] < cullbox->min[j]) cullbox->min[j] = finalVertex[j];
			if (finalVertex[j] > cullbox->max[j]) cullbox->max[j] = finalVertex[j];
		}
	}

	// spread the mins and maxs by 0.5 to ensure we never have zero in any dimension
	for (j = 0; j < 3; j++)
	{
		cullbox->min[j] -= 0.5f;
		cullbox->max[j] += 0.5f;
	}
}


/*
==================
MD5_MakeCullboxes

==================
*/
static void MD5_MakeCullboxes (md5header_t *hdr, struct md5_mesh_t *mesh, struct md5_anim_t *anim)
{
	int f;

	for (f = 0; f < anim->num_frames; f++)
	{
		// calc the cullbox for this frame
		MD5_CullboxForFrame (mesh, anim->skelFrames[f], &anim->bboxes[f]);
	}
}


/*
==================
MD5_LoadSkins

==================
*/
static void MD5_LoadSkins (md5header_t *hdr, char *shader)
{
	// this just needs to be arbitrarily large enough
	// protocol specifies byte data for U_SKIN so this will do us just fine
	md5skin_t allskins[256];
	int i, j;

	// no skins to begin with
	hdr->numskins = 0;

	// load skins
	for (i = 0; i < 256; i++)
	{
		// load skin frames for auto-animation
		qpic_t *images[256];
		int numskins = 0;

		for (j = 0; j < 256; j++)
		{
			// the protocol specifies up to 256 skins but the image naming format only has 2 digits, so we assume that hex is the intent and load it as that
			char skinname[MAX_QPATH];
			sprintf (skinname, "progs/%s_%02x_%02x.lmp", shader, i, j);

			// load until one fails
			if ((images[numskins] = (qpic_t *) COM_LoadHunkFile (skinname)) == NULL)
				break;

			// skins are in .lmp format; byte-swap the header (lmp skins are already flood-filled)
			SwapPic (images[numskins]);

			// another content hack - these texels are fullbright on the original MDL skin but were not on the MD5 replacement, so put them back how they should be
			if (images[numskins]->width == 256 && images[numskins]->height == 256 && !strcmp (skinname, "progs/soldier_00_00.lmp"))
			{
				byte *data = images[numskins]->data;
				data[46827] = data[46828] = data[46829] = data[47083] = data[47084] = data[47085] = data[47340] = data[47341] = 248;
				data[47339] = 247;
			}

			// go to the next skin
			numskins++;
		}

		// if we got animations for this skin then store it out, otherwise we're done
		if (numskins > 0)
		{
			// store it out
			allskins[hdr->numskins].images = (qpic_t **) Hunk_Alloc (numskins * sizeof (qpic_t *));
			memcpy (allskins[hdr->numskins].images, images, numskins * sizeof (qpic_t *));

			// copy off the number of skins too...
			allskins[hdr->numskins].numskins = numskins;

			// go to the next skin
			hdr->numskins++;
		}
		else break;
	}

	// see did we get any skins
	if (hdr->numskins > 0)
	{
		// store it out
		hdr->skins = (md5skin_t *) Hunk_Alloc (hdr->numskins * sizeof (md5skin_t));
		memcpy (hdr->skins, allskins, hdr->numskins * sizeof (md5skin_t));
	}
	else Sys_Error ("MD5_LoadSkins : no skins loaded for \"%s\"", shader);
}


/*
==================
Mod_LoadMD5Model

==================
*/
qboolean Mod_LoadMD5Model (model_t *mod, void *buffer)
{
	// retain a pointer to the original MDL so that we can copy over and validate some data
	mdl_t *pinmodel = (mdl_t *) buffer;

	// copy these off for use and validation
	int mdlframes = LittleLong (pinmodel->numframes);
	int mdlflags = LittleLong (pinmodel->flags);
	int mdlsynctype = LittleLong (pinmodel->synctype);

	// we can't change the original model name so we must copy it off for loading
	char copyname[64];

	// everything after this is freed if the load fails
	int mark = Hunk_LowMark ();

	// alloc header space
	md5header_t *hdr = (md5header_t *) Hunk_Alloc (sizeof (md5header_t));

	// get the baseline name
	COM_StripExtension (mod->name, copyname);

	// look for a mesh
	if (!MD5_ReadMeshFile (va ("%s.md5mesh", copyname), &hdr->md5mesh)) goto md5_bad;

	// look for an animation
	if (!MD5_ReadAnimFile (va ("%s.md5anim", copyname), &hdr->md5anim)) goto md5_bad;

	// validate the frames
	if (hdr->md5anim.num_frames == 2 && mdlframes == 1)
		; // special case; some content has this and it's valid
	else if (hdr->md5anim.num_frames != mdlframes)
		goto md5_bad; // frames don't match so it can't be used as a drop-in replacement
	else if (hdr->md5anim.num_frames > 65536)
		goto md5_bad; // exceeds protocol maximum

	// the MD5 spec allows for more than 1 mesh but we don't need to support it for Quake21 content
	if (hdr->md5mesh.num_meshes > 1) goto md5_bad;

	// load the cullboxes
	// some of the source MD5s were exported with bad cullboxes, so we must regenerate them correctly
	MD5_MakeCullboxes (hdr, hdr->md5mesh.meshes, &hdr->md5anim);

	// allocate memory for the animated skeleton
	hdr->skeleton = (struct md5_joint_t *) Hunk_Alloc (sizeof (struct md5_joint_t) * hdr->md5anim.num_joints);

	// build the baseframe normals
	MD5_BuildBaseNormals (hdr, &hdr->md5mesh.meshes[0]);

	// load textures from .lmp files
	MD5_LoadSkins (hdr, hdr->md5mesh.meshes[0].shader);

	// and done
	mod->cache.data = hdr;
	mod->type = mod_md5;

	// mh - using certain properties the same as the source MDL
	// don't change the physics cullboxes; instead we'll use the per-frame cullboxes stored in the MD5 itself.
	mod->mins[0] = mod->mins[1] = mod->mins[2] = -16;
	mod->maxs[0] = mod->maxs[1] = mod->maxs[2] = 16;

	// copy these over as well
	mod->flags = mdlflags;
	mod->synctype = mdlsynctype;

	// and done
	return true;

	// jump-out point for a bad/mismatched replacement
md5_bad:;
	// failed; free all memory and returns false
	Hunk_FreeToLowMark (mark);

	// didn't load it
	return false;
}


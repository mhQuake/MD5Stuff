/*
Copyright (C) 1996-1997 Id Software, Inc.

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

// r_md5.c: routines for setting up to draw md5 models
// a lot of this is common to alias models and r_alias.c and this file can be refactored/combined massively, but i left it this way so as to
// leave r_alias.c unmutilated, and so that everything for MD5s is reasonably self-contained here.

#include "quakedef.h"
#include "r_local.h"
#include "d_local.h"	// FIXME: shouldn't be needed (is needed for patch
						// right now, but that should move)

#define LIGHT_MIN	5		// lowest light value we'll allow, to avoid the
							//  need for inner-loop light clamping

// externs shared with r_alias.c
extern vec3_t		r_plightvec;
extern int			r_ambientlight;
extern float		r_shadelight;
extern void			*acolormap;	// FIXME: should go away

md5polyvert_t		*r_md5verts;

static float		ziscale;
static model_t		*pmodel;

static vec3_t		md5_forward, md5_right, md5_up;

int				r_md5numverts;

float	md5transform[3][4];

typedef struct {
	int	index0;
	int	index1;
} md5edge_t;

// these are exactly the same as aedges in r_alias.c
static md5edge_t	md5edges[12] = {
{0, 1}, {1, 2}, {2, 3}, {3, 0},
{4, 5}, {5, 6}, {6, 7}, {7, 4},
{0, 5}, {1, 4}, {2, 7}, {3, 6}
};

void R_MD5TransformAndProjectFinalVerts (finalvert_t *fv);
void R_MD5SetUpTransform (int trivial_accept);
void R_MD5TransformVector (vec3_t in, vec3_t out);
void R_MD5TransformFinalVert (finalvert_t *fv, auxvert_t *av, md5polyvert_t *pverts);
void R_MD5ProjectFinalVert (finalvert_t *fv, auxvert_t *av);


/*
================
R_MD5CheckBBox
================
*/
qboolean R_MD5CheckBBox (void)
{
	int					i, flags, frame, numv;
	md5header_t			*hdr;
	struct md5_bbox_t	*bbox;
	float				zi, basepts[8][3], v0, v1, frac;
	finalvert_t			*pv0, *pv1, viewpts[16];
	auxvert_t			*pa0, *pa1, viewaux[16];
	qboolean			zclipped, zfullyclipped;
	unsigned			anyclip, allclip;
	int					minz;

	// expand, rotate, and translate points into worldspace
	currententity->trivial_accept = 0;
	pmodel = currententity->model;
	hdr = (md5header_t *) pmodel->cache.data;

	R_MD5SetUpTransform (0);

	// construct the base bounding box for this frame
	frame = currententity->frame;

	// TODO: don't repeat this check when drawing?
	if ((frame >= hdr->md5anim.num_frames) || (frame < 0))
	{
		Con_DPrintf ("No such frame %d %s\n", frame, pmodel->name);
		frame = 0;
	}

	bbox = &hdr->md5anim.bboxes[frame];

	// x worldspace coordinates
	basepts[0][0] = basepts[1][0] = basepts[2][0] = basepts[3][0] = bbox->min[0];
	basepts[4][0] = basepts[5][0] = basepts[6][0] = basepts[7][0] = bbox->max[0];

	// y worldspace coordinates
	basepts[0][1] = basepts[3][1] = basepts[5][1] = basepts[6][1] = bbox->min[1];
	basepts[1][1] = basepts[2][1] = basepts[4][1] = basepts[7][1] = bbox->max[1];

	// z worldspace coordinates
	basepts[0][2] = basepts[1][2] = basepts[4][2] = basepts[5][2] = bbox->min[2];
	basepts[2][2] = basepts[3][2] = basepts[6][2] = basepts[7][2] = bbox->max[2];

	zclipped = false;
	zfullyclipped = true;

	// mh - adding another coupla 9s here
	minz = 999999;

	for (i = 0; i < 8; i++)
	{
		R_MD5TransformVector (&basepts[i][0], &viewaux[i].fv[0]);

		if (viewaux[i].fv[2] < ALIAS_Z_CLIP_PLANE)
		{
			// we must clip points that are closer than the near clip plane
			viewpts[i].flags = ALIAS_Z_CLIP;
			zclipped = true;
		}
		else
		{
			if (viewaux[i].fv[2] < minz)
				minz = viewaux[i].fv[2];
			viewpts[i].flags = 0;
			zfullyclipped = false;
		}
	}

	if (zfullyclipped)
	{
		return false;	// everything was near-z-clipped
	}

	numv = 8;

	if (zclipped)
	{
	// organize points by edges, use edges to get new points (possible trivial
	// reject)
		for (i=0 ; i<12 ; i++)
		{
		// edge endpoints
			pv0 = &viewpts[md5edges[i].index0];
			pv1 = &viewpts[md5edges[i].index1];
			pa0 = &viewaux[md5edges[i].index0];
			pa1 = &viewaux[md5edges[i].index1];

		// if one end is clipped and the other isn't, make a new point
			if (pv0->flags ^ pv1->flags)
			{
				frac = (ALIAS_Z_CLIP_PLANE - pa0->fv[2]) /
					   (pa1->fv[2] - pa0->fv[2]);
				viewaux[numv].fv[0] = pa0->fv[0] +
						(pa1->fv[0] - pa0->fv[0]) * frac;
				viewaux[numv].fv[1] = pa0->fv[1] +
						(pa1->fv[1] - pa0->fv[1]) * frac;
				viewaux[numv].fv[2] = ALIAS_Z_CLIP_PLANE;
				viewpts[numv].flags = 0;
				numv++;
			}
		}
	}

// project the vertices that remain after clipping
	anyclip = 0;
	allclip = ALIAS_XY_CLIP_MASK;

// TODO: probably should do this loop in ASM, especially if we use floats
	for (i=0 ; i<numv ; i++)
	{
	// we don't need to bother with vertices that were z-clipped
		if (viewpts[i].flags & ALIAS_Z_CLIP)
			continue;

		zi = 1.0 / viewaux[i].fv[2];

	// FIXME: do with chop mode in ASM, or convert to float
		v0 = (viewaux[i].fv[0] * xscale * zi) + xcenter;
		v1 = (viewaux[i].fv[1] * yscale * zi) + ycenter;

		flags = 0;

		if (v0 < r_refdef.fvrectx)
			flags |= ALIAS_LEFT_CLIP;
		if (v1 < r_refdef.fvrecty)
			flags |= ALIAS_TOP_CLIP;
		if (v0 > r_refdef.fvrectright)
			flags |= ALIAS_RIGHT_CLIP;
		if (v1 > r_refdef.fvrectbottom)
			flags |= ALIAS_BOTTOM_CLIP;

		anyclip |= flags;
		allclip &= flags;
	}

	if (allclip)
		return false;	// trivial reject off one side

	currententity->trivial_accept = !anyclip & !zclipped;

	// to do - check this for assumptions
#if 0
	if (currententity->trivial_accept)
	{
		if (minz > (r_aliastransition + (pmdl->size * r_resfudge)))
		{
			currententity->trivial_accept |= 2;
		}
	}
#endif

	return true;
}


/*
================
R_MD5TransformVector
================
*/
void R_MD5TransformVector (vec3_t in, vec3_t out)
{
	out[0] = DotProduct(in, md5transform[0]) + md5transform[0][3];
	out[1] = DotProduct(in, md5transform[1]) + md5transform[1][3];
	out[2] = DotProduct(in, md5transform[2]) + md5transform[2][3];
}


/*
================
R_MD5PreparePoints

General clipped case
================
*/
void R_MD5PreparePoints (md5header_t *hdr)
{
	int			i;
	finalvert_t	*fv;
	auxvert_t	*av;
	mtriangle_t	*ptri;
	int			numtris;
	finalvert_t	*pfv[3];

	r_md5numverts = hdr->md5mesh.meshes[0].num_verts;
 	fv = pfinalverts;
	av = pauxverts;

	for (i = 0; i < r_md5numverts; i++, fv++, av++, r_md5verts++)
	{
		R_MD5TransformFinalVert (fv, av, r_md5verts);

		if (av->fv[2] < ALIAS_Z_CLIP_PLANE)
			fv->flags |= ALIAS_Z_CLIP;
		else
		{
			 R_MD5ProjectFinalVert (fv, av);

			if (fv->v[0] < r_refdef.aliasvrect.x)
				fv->flags |= ALIAS_LEFT_CLIP;
			if (fv->v[1] < r_refdef.aliasvrect.y)
				fv->flags |= ALIAS_TOP_CLIP;
			if (fv->v[0] > r_refdef.aliasvrectright)
				fv->flags |= ALIAS_RIGHT_CLIP;
			if (fv->v[1] > r_refdef.aliasvrectbottom)
				fv->flags |= ALIAS_BOTTOM_CLIP;	
		}
	}

	// clip and draw all triangles
	r_affinetridesc.numtriangles = 1;

	ptri = hdr->md5mesh.meshes[0].triangles;
	numtris = hdr->md5mesh.meshes[0].num_tris;

	for (i = 0; i < numtris; i++, ptri++)
	{
		pfv[0] = &pfinalverts[ptri->vertindex[0]];
		pfv[1] = &pfinalverts[ptri->vertindex[1]];
		pfv[2] = &pfinalverts[ptri->vertindex[2]];

		if (pfv[0]->flags & pfv[1]->flags & pfv[2]->flags & (ALIAS_XY_CLIP_MASK | ALIAS_Z_CLIP))
			continue;		// completely clipped

		if (!((pfv[0]->flags | pfv[1]->flags | pfv[2]->flags) & (ALIAS_XY_CLIP_MASK | ALIAS_Z_CLIP)))
		{
			// totally unclipped
			r_affinetridesc.pfinalverts = pfinalverts;
			r_affinetridesc.ptriangles = ptri;
			D_PolysetDraw ();
		}
		else		
		{
			// partially clipped
			R_AliasClipTriangle (ptri);
		}
	}
}


/*
================
R_MD5SetUpTransform
================
*/
void R_MD5SetUpTransform (int trivial_accept)
{
	int				i;
	float			rotationmatrix[3][4];
	static float	viewmatrix[3][4];
	vec3_t			angles;

// TODO: should really be stored with the entity instead of being reconstructed
// TODO: should use a look-up table
// TODO: could cache lazily, stored in the entity

	angles[ROLL] = currententity->angles[ROLL];
	angles[PITCH] = -currententity->angles[PITCH];
	angles[YAW] = currententity->angles[YAW];
	AngleVectors (angles, md5_forward, md5_right, md5_up);

// TODO: can do this with simple matrix rearrangement

	for (i=0 ; i<3 ; i++)
	{
		rotationmatrix[i][0] = md5_forward[i];
		rotationmatrix[i][1] = -md5_right[i];
		rotationmatrix[i][2] = md5_up[i];
	}

	rotationmatrix[0][3] = -modelorg[0];
	rotationmatrix[1][3] = -modelorg[1];
	rotationmatrix[2][3] = -modelorg[2];

// TODO: should be global, set when vright, etc., set
	VectorCopy (vright, viewmatrix[0]);
	VectorCopy (vup, viewmatrix[1]);
	VectorInverse (viewmatrix[1]);
	VectorCopy (vpn, viewmatrix[2]);

//	viewmatrix[0][3] = 0;
//	viewmatrix[1][3] = 0;
//	viewmatrix[2][3] = 0;

	R_ConcatTransforms (viewmatrix, rotationmatrix, md5transform);

// do the scaling up of x and y to screen coordinates as part of the transform
// for the unclipped case (it would mess up clipping in the clipped case).
// Also scale down z, so 1/z is scaled 31 bits for free, and scale down x and y
// correspondingly so the projected x and y come out right
// FIXME: make this work for clipped case too?
	if (trivial_accept)
	{
		for (i=0 ; i<4 ; i++)
		{
			md5transform[0][i] *= aliasxscale *
					(1.0 / ((float)0x8000 * 0x10000));
			md5transform[1][i] *= aliasyscale *
					(1.0 / ((float)0x8000 * 0x10000));
			md5transform[2][i] *= 1.0 / ((float)0x8000 * 0x10000);

		}
	}
}


/*
================
R_MD5TransformFinalVert
================
*/
void R_MD5TransformFinalVert (finalvert_t *fv, auxvert_t *av, md5polyvert_t *pverts)
{
	int		temp;
	float	lightcos, *plightnormal;

	av->fv[0] = DotProduct (pverts->position, md5transform[0]) + md5transform[0][3];
	av->fv[1] = DotProduct (pverts->position, md5transform[1]) + md5transform[1][3];
	av->fv[2] = DotProduct (pverts->position, md5transform[2]) + md5transform[2][3];

	fv->v[2] = pverts->texcoord[0];
	fv->v[3] = pverts->texcoord[1];

	fv->flags = 0; // never onseam with MD5

	// lighting
	plightnormal = pverts->normal;
	lightcos = DotProduct (plightnormal, r_plightvec);
	temp = r_ambientlight;

	if (lightcos < 0)
	{
		temp += (int) (r_shadelight * lightcos);

		// clamp; because we limited the minimum ambient and shading light, we
		// don't have to clamp low light, just bright
		if (temp < 0)
			temp = 0;
	}

	fv->v[4] = temp;
}


/*
================
R_MD5TransformAndProjectFinalVerts_C
================
*/
void R_MD5TransformAndProjectFinalVerts_C (finalvert_t *fv)
{
	int				i, temp;
	float			lightcos, *plightnormal, zi;
	md5polyvert_t	*pverts;

	pverts = r_md5verts;

	for (i = 0; i < r_md5numverts; i++, fv++, pverts++)
	{
		// transform and project
		zi = 1.0 / (DotProduct (pverts->position, md5transform[2]) + md5transform[2][3]);

		// x, y, and z are scaled down by 1/2**31 in the transform, so 1/z is
		// scaled up by 1/2**31, and the scaling cancels out for x and y in the
		// projection
		fv->v[5] = zi;

		fv->v[0] = ((DotProduct (pverts->position, md5transform[0]) + md5transform[0][3]) * zi) + aliasxcenter;
		fv->v[1] = ((DotProduct (pverts->position, md5transform[1]) + md5transform[1][3]) * zi) + aliasycenter;

		fv->v[2] = pverts->texcoord[0];
		fv->v[3] = pverts->texcoord[1];
		fv->flags = 0; // never onseam with MD5

		// lighting
		plightnormal = pverts->normal;
		lightcos = DotProduct (plightnormal, r_plightvec);
		temp = r_ambientlight;

		if (lightcos < 0)
		{
			temp += (int) (r_shadelight * lightcos);

			// clamp; because we limited the minimum ambient and shading light, we
			// don't have to clamp low light, just bright
			if (temp < 0)
				temp = 0;
		}

		fv->v[4] = temp;
	}
}


/*
================
R_MD5ProjectFinalVert
================
*/
void R_MD5ProjectFinalVert (finalvert_t *fv, auxvert_t *av)
{
	float	zi;

	// project points
	zi = 1.0 / av->fv[2];

	fv->v[5] = zi * ziscale;

	fv->v[0] = (av->fv[0] * aliasxscale * zi) + aliasxcenter;
	fv->v[1] = (av->fv[1] * aliasyscale * zi) + aliasycenter;
}


/*
================
R_MD5PrepareUnclippedPoints
================
*/
void R_MD5PrepareUnclippedPoints (md5header_t *hdr)
{
	r_md5numverts = hdr->md5mesh.meshes[0].num_verts;

	R_MD5TransformAndProjectFinalVerts_C (pfinalverts);

	if (r_affinetridesc.drawtype)
		D_PolysetDrawFinalVerts (pfinalverts, r_md5numverts);

	r_affinetridesc.pfinalverts = pfinalverts;
	r_affinetridesc.ptriangles = hdr->md5mesh.meshes[0].triangles;
	r_affinetridesc.numtriangles = hdr->md5mesh.meshes[0].num_tris;

	D_PolysetDraw ();
}


/*
===============
R_AliasSetupSkin
===============
*/
void R_MD5SetupSkin (md5header_t *hdr)
{
	int					skinnum;
	int					numskins;
	md5skin_t			*skin;
	qpic_t				*image;

	skinnum = currententity->skinnum;

	if ((skinnum >= hdr->numskins) || (skinnum < 0))
	{
		Con_DPrintf ("R_AliasSetupSkin: no such skin # %d\n", skinnum);
		skinnum = 0;
	}

	// skin selection and auto-animation
	skin = &hdr->skins[skinnum % hdr->numskins];
	image = skin->images[(int) ((cl.time + currententity->syncbase) * 10) % skin->numskins];

	a_skinwidth = image->width;

	r_affinetridesc.pskindesc = NULL; // ?????seems to be unused?????
	r_affinetridesc.pskin = image->data;
	r_affinetridesc.skinwidth = a_skinwidth;
	r_affinetridesc.seamfixupX16 = 0;//(a_skinwidth >> 1) << 16;
	r_affinetridesc.skinheight = image->height;
}


/*
================
R_MD5SetupLighting
================
*/
void R_MD5SetupLighting (alight_t *plighting)
{
	// guarantee that no vertex will ever be lit below LIGHT_MIN, so we don't have
	// to clamp off the bottom
	r_ambientlight = plighting->ambientlight;

	if (r_ambientlight < LIGHT_MIN)
		r_ambientlight = LIGHT_MIN;

	r_ambientlight = (255 - r_ambientlight) << VID_CBITS;

	if (r_ambientlight < LIGHT_MIN)
		r_ambientlight = LIGHT_MIN;

	r_shadelight = plighting->shadelight;

	if (r_shadelight < 0)
		r_shadelight = 0;

	r_shadelight *= VID_GRADES;

	// rotate the lighting vector into the model's frame of reference
	r_plightvec[0] = DotProduct (plighting->plightvec, md5_forward);
	r_plightvec[1] = -DotProduct (plighting->plightvec, md5_right);
	r_plightvec[2] = DotProduct (plighting->plightvec, md5_up);
}


/*
==================
MD5_InterpolateSkeletons

==================
*/
static void MD5_InterpolateSkeletons (const struct md5_joint_t *skelA, const struct md5_joint_t *skelB, int num_joints, float interp, struct md5_joint_t *out)
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
		vec3_t finalNormal = {0.0f, 0.0f, 0.0f};
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

			// Calculate transformed normal for this weight
			Quat_rotatePoint (joint->orient, vnorms[i].normal, wv);

			// The sum of all weight->bias should be 1.0
			finalNormal[0] += wv[0] * weight->bias;
			finalNormal[1] += wv[1] * weight->bias;
			finalNormal[2] += wv[2] * weight->bias;
		}

		// store out position
		vertexes[i].position[0] = finalVertex[0];
		vertexes[i].position[1] = finalVertex[1];
		vertexes[i].position[2] = finalVertex[2];

		// store out normal
		vertexes[i].normal[0] = finalNormal[0];
		vertexes[i].normal[1] = finalNormal[1];
		vertexes[i].normal[2] = finalNormal[2];

		// store out texcoords - this needs the same calculation as setup of stverts in Mod_LoadAliasModel, including scaling the skin up to
		// an unnormalized range.  in theory each MD5 skin can be a different size so it's deferred to here.
		vertexes[i].texcoord[0] = (int) (mesh->vertices[i].st[0] * r_affinetridesc.skinwidth) << 16;
		vertexes[i].texcoord[1] = (int) (mesh->vertices[i].st[1] * r_affinetridesc.skinheight) << 16;
	}
}


/*
==================
R_InterpolateMD5Model

==================
*/
void R_InterpolateMD5Model (md5header_t *hdr, int pose1, int pose2, float lerpfrac)
{
	// optimize the non-interpolated cases
	if (pose1 == pose2)
	{
		// case #1 - not lerping, just animate from a single skeleton
		MD5_PrepareMesh (&hdr->md5mesh.meshes[0], hdr->md5anim.skelFrames[pose1], hdr->vertexes, hdr->vnorms);
	}
	else if (!(lerpfrac > 0))
	{
		// case #2 : lerpblend is 0 so just animate from one frame
		MD5_PrepareMesh (&hdr->md5mesh.meshes[0], hdr->md5anim.skelFrames[pose1], hdr->vertexes, hdr->vnorms);
	}
	else if (!(lerpfrac < 1))
	{
		// case #3 : lerpblend is 1 so just animate from one frame
		MD5_PrepareMesh (&hdr->md5mesh.meshes[0], hdr->md5anim.skelFrames[pose2], hdr->vertexes, hdr->vnorms);
	}
	else
	{
		// case #4 : full interpolation - run the skeletal animation
		MD5_InterpolateSkeletons (hdr->md5anim.skelFrames[pose1], hdr->md5anim.skelFrames[pose2], hdr->md5anim.num_joints, lerpfrac, hdr->skeleton);

		// and set up the vertex array
		MD5_PrepareMesh (&hdr->md5mesh.meshes[0], hdr->skeleton, hdr->vertexes, hdr->vnorms);
	}
}


/*
=================
R_MD5SetupFrame

set r_apverts
=================
*/
void R_MD5SetupFrame (md5header_t *hdr)
{
	int		frame;

	frame = currententity->frame;

	if ((frame >= hdr->md5anim.num_frames) || (frame < 0))
	{
		Con_DPrintf ("R_MD5SetupFrame: no such frame %d\n", frame);
		frame = 0;
	}

	// animate the skeleton into hdr->vertexes
	// this code isn't doing frame interpolation; in an interpolated case there would be two different poses here and a non-0 (or non-1) interpolation fraction
	R_InterpolateMD5Model (hdr, frame, frame, 0);

	// yayy software Quake globals
	r_md5verts = hdr->vertexes;
}


/*
================
R_MD5DrawModel
================
*/
void R_MD5DrawModel (alight_t *plighting)
{
	md5header_t *hdr = (md5header_t *) currententity->model->cache.data;

	// to do - this can reuse the same structs as alias drawing for there, but is the vertex count enough????
	finalvert_t		finalverts[MAXALIASVERTS + ((CACHE_SIZE - 1) / sizeof (finalvert_t)) + 1];
	auxvert_t		auxverts[MAXALIASVERTS];

	r_amodels_drawn++;

	// cache align
	pfinalverts = (finalvert_t *) (((long) &finalverts[0] + CACHE_SIZE - 1) & ~(CACHE_SIZE - 1));
	pauxverts = &auxverts[0];

	R_MD5SetupSkin (hdr);
	R_MD5SetUpTransform (currententity->trivial_accept);
	R_MD5SetupLighting (plighting);
	R_MD5SetupFrame (hdr);

	if (!currententity->colormap)
		Sys_Error ("R_MD5DrawModel: !currententity->colormap");

	r_affinetridesc.drawtype = (currententity->trivial_accept == 3) && r_recursiveaffinetriangles;

	if (r_affinetridesc.drawtype)
	{
		D_PolysetUpdateTables ();		// FIXME: precalc...
	}
	else
	{
#if	id386
		D_Aff8Patch (currententity->colormap);
#endif
	}

	acolormap = currententity->colormap;

	if (currententity != &cl.viewent)
		ziscale = (float) 0x8000 * (float) 0x10000;
	else
		ziscale = (float) 0x8000 * (float) 0x10000 * 3.0;

	if (currententity->trivial_accept)
		R_MD5PrepareUnclippedPoints (hdr);
	else
		R_MD5PreparePoints (hdr);
}


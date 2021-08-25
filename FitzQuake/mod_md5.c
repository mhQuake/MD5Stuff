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

	return false;
}


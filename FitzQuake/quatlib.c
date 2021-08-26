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
// quaternion stuff - just so it doesn't mix in with the rest of the code...

#include "quakedef.h"


enum {
	X, Y, Z, W
};

void Quat_computeW (quat4_t q)
{
	float t = 1.0f - (q[X] * q[X]) - (q[Y] * q[Y]) - (q[Z] * q[Z]);

	if (t < 0.0f)
		q[W] = 0.0f;
	else q[W] = -sqrt (t);
}


void Quat_normalize (quat4_t q)
{
	// compute magnitude of the quaternion
	float mag = sqrt ((q[X] * q[X]) + (q[Y] * q[Y]) + (q[Z] * q[Z]) + (q[W] * q[W]));

	// check for bogus length, to protect against divide by zero
	if (mag > 0.0f)
	{
		// normalize it
		float oneOverMag = 1.0f / mag;

		q[X] *= oneOverMag;
		q[Y] *= oneOverMag;
		q[Z] *= oneOverMag;
		q[W] *= oneOverMag;
	}
}

void Quat_multQuat (const quat4_t qa, const quat4_t qb, quat4_t out)
{
	out[W] = (qa[W] * qb[W]) - (qa[X] * qb[X]) - (qa[Y] * qb[Y]) - (qa[Z] * qb[Z]);
	out[X] = (qa[X] * qb[W]) + (qa[W] * qb[X]) + (qa[Y] * qb[Z]) - (qa[Z] * qb[Y]);
	out[Y] = (qa[Y] * qb[W]) + (qa[W] * qb[Y]) + (qa[Z] * qb[X]) - (qa[X] * qb[Z]);
	out[Z] = (qa[Z] * qb[W]) + (qa[W] * qb[Z]) + (qa[X] * qb[Y]) - (qa[Y] * qb[X]);
}

void Quat_multVec (const quat4_t q, const vec3_t v, quat4_t out)
{
	out[W] = -(q[X] * v[X]) - (q[Y] * v[Y]) - (q[Z] * v[Z]);
	out[X] = (q[W] * v[X]) + (q[Y] * v[Z]) - (q[Z] * v[Y]);
	out[Y] = (q[W] * v[Y]) + (q[Z] * v[X]) - (q[X] * v[Z]);
	out[Z] = (q[W] * v[Z]) + (q[X] * v[Y]) - (q[Y] * v[X]);
}

void Quat_inverse (const quat4_t q, quat4_t inv)
{
	inv[X] = -q[X]; inv[Y] = -q[Y];
	inv[Z] = -q[Z]; inv[W] = q[W];

	Quat_normalize (inv);
}

void Quat_rotatePoint (const quat4_t q, const vec3_t in, vec3_t out)
{
	quat4_t tmp, inv, final;

	Quat_inverse (q, inv);
	Quat_multVec (q, in, tmp);
	Quat_multQuat (tmp, inv, final);

	out[X] = final[X];
	out[Y] = final[Y];
	out[Z] = final[Z];
}


/*
==================
Quat_dotProduct

==================
*/
float Quat_dotProduct (const quat4_t qa, const quat4_t qb)
{
	return ((qa[X] * qb[X]) + (qa[Y] * qb[Y]) + (qa[Z] * qb[Z]) + (qa[W] * qb[W]));
}


/*
==================
Quat_slerp

==================
*/
void Quat_slerp (const quat4_t qa, const quat4_t qb, float t, quat4_t out)
{
	// Check for out-of range parameter and return edge points if so
	if (t <= 0.0)
	{
		memcpy (out, qa, sizeof (quat4_t));
		return;
	}

	if (t >= 1.0)
	{
		memcpy (out, qb, sizeof (quat4_t));
		return;
	}

	{
	// Compute "cosine of angle between quaternions" using dot product
	float cosOmega = Quat_dotProduct (qa, qb);

	// If negative dot, use -q1.  Two quaternions q and -q represent the same rotation, but may produce different slerp.  We chose q or -q to rotate using the acute angle.
	float q1w = qb[W];
	float q1x = qb[X];
	float q1y = qb[Y];
	float q1z = qb[Z];

	if (cosOmega < 0.0f)
	{
		q1w = -q1w;
		q1x = -q1x;
		q1y = -q1y;
		q1z = -q1z;
		cosOmega = -cosOmega;
	}

	{
	// Compute interpolation fraction, checking for quaternions almost exactly the same
	float k0, k1;

	if (cosOmega > 0.9999f)
	{
		// Very close - just use linear interpolation, which will protect againt a divide by zero
		k0 = 1.0f - t;
		k1 = t;
	}
	else
	{
		// Compute the sin of the angle using the trig identity sin^2(omega) + cos^2(omega) = 1
		float sinOmega = sqrt (1.0f - (cosOmega * cosOmega));

		// Compute the angle from its sin and cosine
		float omega = atan2 (sinOmega, cosOmega);

		// Compute inverse of denominator, so we only have to divide once
		float oneOverSinOmega = 1.0f / sinOmega;

		// Compute interpolation parameters
		k0 = sin ((1.0f - t) * omega) * oneOverSinOmega;
		k1 = sin (t * omega) * oneOverSinOmega;
	}

	// Interpolate and return new quaternion
	out[W] = (k0 * qa[3]) + (k1 * q1w);
	out[X] = (k0 * qa[0]) + (k1 * q1x);
	out[Y] = (k0 * qa[1]) + (k1 * q1y);
	out[Z] = (k0 * qa[2]) + (k1 * q1z);
	}
	}
}


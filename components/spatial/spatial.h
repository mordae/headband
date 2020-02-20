/*
 * Copyright (C)  Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _COMPONENT_SPATIAL_H
#define _COMPONENT_SPATIAL_H 1


struct vec3 {
	float row[3];
};

typedef struct vec3 vec3;


struct mat3 {
	vec3 col[3];
};

typedef struct mat3 mat3;


struct quat {
	float w, x, y, z;
};

typedef struct quat quat;


inline static float maxf(float a, float b)
{
	return a >= b ? a : b;
}


inline static float minf(float a, float b)
{
	return a >= b ? b : a;
}


inline static vec3 vec3scale(float c, vec3 v)
{
	return (vec3){{c * v.row[0], c * v.row[1], c * v.row[2]}};
}


inline static vec3 vec3add2(vec3 a, vec3 b)
{
	return (vec3){{
		a.row[0] + b.row[0],
		a.row[1] + b.row[1],
		a.row[2] + b.row[2],
	}};
}


inline static vec3 vec3add3(vec3 a, vec3 b, vec3 c)
{
	return (vec3){{
		a.row[0] + b.row[0] + c.row[0],
		a.row[1] + b.row[1] + c.row[1],
		a.row[2] + b.row[2] + c.row[2],
	}};
}


inline static float vec3mag(vec3 a)
{
	return sqrt(a.row[0] * a.row[0] +
	            a.row[1] * a.row[1] +
	            a.row[2] * a.row[2]);
}


inline static vec3 vec3unit(vec3 a)
{
	return (vec3){{
		a.row[0] / vec3mag(a),
		a.row[1] / vec3mag(a),
		a.row[2] / vec3mag(a),
	}};
}


inline static float vec3dot(vec3 a, vec3 b)
{
	return a.row[0] * b.row[0] +
	       a.row[1] * b.row[1] +
	       a.row[2] * b.row[2];
}


inline static vec3 vec3cross(vec3 a, vec3 b)
{
	return (vec3){{
		a.row[1] * b.row[2] - a.row[2] * b.row[1],
		a.row[2] * b.row[0] - a.row[0] * b.row[2],
		a.row[0] * b.row[1] - a.row[1] * b.row[0],
	}};
}


inline static mat3 mat3scale(float c, mat3 m)
{
	return (mat3){{
		vec3scale(c, m.col[0]),
		vec3scale(c, m.col[1]),
		vec3scale(c, m.col[2]),
	}};
}


inline static mat3 mat3mul(mat3 a, mat3 b)
{
	return (mat3){{
		vec3add3(vec3scale(b.col[0].row[0], a.col[0]),
		         vec3scale(b.col[0].row[1], a.col[1]),
			 vec3scale(b.col[0].row[2], a.col[2])),

		vec3add3(vec3scale(b.col[1].row[0], a.col[0]),
		         vec3scale(b.col[1].row[1], a.col[1]),
			 vec3scale(b.col[1].row[2], a.col[2])),

		vec3add3(vec3scale(b.col[2].row[0], a.col[0]),
		         vec3scale(b.col[2].row[1], a.col[1]),
			 vec3scale(b.col[2].row[2], a.col[2])),
	}};
}


inline static quat quat_from_mat3(mat3 a)
{
	float w = 1 + a.col[0].row[0] + a.col[1].row[1] + a.col[2].row[2];
	float x = 1 + a.col[0].row[0] - a.col[1].row[1] - a.col[2].row[2];
	float y = 1 - a.col[0].row[0] + a.col[1].row[1] - a.col[2].row[2];
	float z = 1 - a.col[0].row[0] - a.col[1].row[1] + a.col[2].row[2];

	float xs = a.col[2].row[1] - a.col[1].row[2];
	float ys = a.col[0].row[2] - a.col[2].row[0];
	float zs = a.col[1].row[0] - a.col[0].row[1];

	return (quat){
		.w = maxf(0, w) / 2.0,
		.x = copysign(maxf(0, x) / 2.0, xs),
		.y = copysign(maxf(0, y) / 2.0, ys),
		.z = copysign(maxf(0, z) / 2.0, zs),
	};
}


inline static vec3 quat_to_euler(quat q)
{
	vec3 angles = {};

	/* roll (x-axis rotation) */
	float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angles.row[0] = atan2(sinr_cosp, cosr_cosp);

	/* pitch (y-axis rotation) */
	float sinp = 2 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1) {
		/* use 90 degrees if out of range */
		angles.row[1] = copysign(M_PI / 2, sinp);
	} else {
		angles.row[1] = asin(sinp);
	}

	/* yaw (z-axis rotation) */
	float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angles.row[2] = atan2(siny_cosp, cosy_cosp);

	return angles;
}


#endif				/* !_COMPONENT_SPATIAL_H */

/*!
 * \file hd_utils.h
 * \brief 
 *
 * Code by Joy.you
 * Contact yjcpui(at)gmail(dot)com
 *
 * The hd ecore
 * Copyright (c) 2013, 海达数云
 * All rights reserved.
 *
 */

#include <comm/hd_utils.h>
#include <math.h>

/**
 * \brief Compute the checksum (single-byte XOR).
 * \param data The address of the first data element in a sequence of bytes to be included in the sum
 * \param length The number of byte in the data sequence
 */
e_uint8 hd_compute_xor(const e_uint8 * const data, const e_uint32 length) {
	e_uint32 i;
	/* Compute the XOR by summing all of the bytes */
	e_uint8 checksum = 0;
	for (i = 0; i < length; i++) {
		checksum ^= data[i]; // NOTE: this is equivalent to simply summing all of the bytes
	}

	/* done */
	return checksum;
}

void hd_polar2xyz(float *x, float *y, float *z, double distance, double angle_h,
		double angle_v) {
	angle_v = (angle_v - (double) 90.0) * M_PI / 180.0;
	angle_h = (double) angle_h * M_PI / 180.0;
	(*x) = distance * 1e3 * cos(angle_v) * cos(angle_h) / 1e3;
	(*y) = distance * 1e3 * cos(angle_v) * sin(angle_h) / 1e3;
	(*z) = distance * 1e3 * sin(angle_v) / 1e3;
}

void hd_print_image(e_uint8* msg, int w,int h) {
	char c, l = 0;
	int i,j,count=0;
	for(i=0;i<h;i++)
	{
		l = 0; count=0;
		for(j=0;j<w;j++){
			c = *msg++;
			if ( (c&&l) || (!c&&!l) ) {
			    count++;
			} else {
				if (count > 0)
					DMSG((STDOUT, " %c[%d]", l? '*' : '0', count));
				l = c;
				count = 1;
			}
		}
		if (count > 0)
			DMSG((STDOUT, " %c[%d]", l? '*' : '0', count));
		DMSG((STDOUT, "\n"));
	}
}


char* hd_strncpy(char *dst, const char *src, size_t len) {
	char* ret=strncpy(dst, src, len);
	dst[len - 1] = '\0';
	return ret;
}

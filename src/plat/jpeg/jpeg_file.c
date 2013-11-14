#include <stdio.h>
#include <malloc.h>

#include <jpeg/jpeglib.h>
#include <jpg/hd_jpeg.h>

#define JPEG_QUALITY 100

e_int32 gray_to_jpeg_file(char *filename, unsigned char *bits, int width,
		int height) {
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	FILE * outfile; /* target file */
	JSAMPROW row_pointer[1]; /* pointer to JSAMPLE row[s] */
	int row_stride; /* physical row width in image buffer */

	if (!(filename && bits && width && height))
		return 0;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	if ((outfile = fopen(filename, "wb")) == NULL) {
		fprintf(stderr, "can't open %s/n", filename);
		return -1;
	}
	jpeg_stdio_dest(&cinfo, outfile);

	cinfo.image_width = width; // image width and height, in pixels
	cinfo.image_height = height;
	cinfo.input_components = 1; // # of color components per pixel
	cinfo.in_color_space = JCS_GRAYSCALE; //colorspace of input image
	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE); // limit to baseline-JPEG values
	jpeg_start_compress(&cinfo, TRUE);
	row_stride = width * 1; // JSAMPLEs per row in image_buffer
	while (cinfo.next_scanline < cinfo.image_height) {
//这里我做过修改，由于jpg文件的图像是倒的，所以改了一下读的顺序
		row_pointer[0] = &bits[cinfo.next_scanline * row_stride];
//row_pointer[0] = & bits[(cinfo.image_height - cinfo.next_scanline - 1) * row_stride];
		(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	jpeg_finish_compress(&cinfo);
	fclose(outfile);

	jpeg_destroy_compress(&cinfo);
	return 0;
}

e_int32 gray_to_jpeg_file_rotation(char *filename, unsigned char *bits,
		int width, int height) {
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	FILE * outfile; /* target file */
	JSAMPROW row_pointer[1]; /* pointer to JSAMPLE row[s] */
	int row_stride; /* physical row width in image buffer */
	int i;

	if (!(filename && bits && width && height))
		return 0;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	if ((outfile = fopen(filename, "wb")) == NULL) {
		fprintf(stderr, "can't open %s/n", filename);
		return -1;
	}
	jpeg_stdio_dest(&cinfo, outfile);

	cinfo.image_width = height; /* image width and height, in pixels */
	cinfo.image_height = width;
	cinfo.input_components = 1; /* # of color components per pixel */
	cinfo.in_color_space = JCS_GRAYSCALE; /* colorspace of input image */

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, JPEG_QUALITY,
			TRUE /* limit to baseline-JPEG values */);

	jpeg_start_compress(&cinfo, TRUE);

	row_stride = height; /* JSAMPLEs per row in image_buffer */

	unsigned char *row = (unsigned char *) malloc(row_stride);
	row_pointer[0] = row;
	while (cinfo.next_scanline < cinfo.image_height) {
		for (i = 0; i < row_stride; i++) { //get one row
			row[i] = bits[cinfo.next_scanline + i * width];
		}
		(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}
	free(row);

	jpeg_finish_compress(&cinfo);
	fclose(outfile);

	jpeg_destroy_compress(&cinfo);
	return 0;
}

#include <stdio.h>
#include <malloc.h>

#include <jpeg/jpeglib.h>
#include <jpg/hd_jpeg.h>

e_int32 gray_to_jpeg( e_uint32 uWidth, e_uint32 uHeight, unsigned char* pImg,
		e_uint32 iQuality, unsigned char **r_buf, e_uint32 *r_size){

	unsigned char*buf = NULL;
	unsigned long lsize = 0;

	// setup JPEG compression structure data
	struct jpeg_compress_struct jcInfo;
	struct jpeg_error_mgr jErr; // JPEG error handler
	jcInfo.err = jpeg_std_error(&jErr);

	// initialize JPEG compression object
	jpeg_create_compress( &jcInfo);

	// specify data destination is memory
	jpeg_mem_dest(&jcInfo, &buf, &lsize);

	// image format
	jcInfo.image_width = uWidth;
	jcInfo.image_height = uHeight;
	jcInfo.input_components = 1;
	jcInfo.in_color_space = JCS_GRAYSCALE;
	jcInfo.optimize_coding = TRUE;
	jcInfo.scale_num = 1;
	jcInfo.scale_denom = 2;

	// set default compression parameters
	jpeg_set_defaults(&jcInfo);

	// set image quality
	jpeg_set_quality(&jcInfo, iQuality, TRUE);

	// start compressor
	jpeg_start_compress(&jcInfo, TRUE);
	int iRowStride = jcInfo.image_width * jcInfo.input_components;
	while (jcInfo.next_scanline < jcInfo.image_height) {
		JSAMPROW pData = &(pImg[jcInfo.next_scanline * iRowStride]);
		jpeg_write_scanlines(&jcInfo, &pData, 1);
	}

	// finish compression
	jpeg_finish_compress(&jcInfo);

	// release JPEG compression object
	jpeg_destroy_compress(&jcInfo);

	(*r_buf) = buf;
	(*r_size) = lsize;

	return 1;
}

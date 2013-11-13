/*
 * jpeg_mem.c
 *
 *  Created on: Sep 25, 2013
 *      Author: uplusplus
 */

#include <stdio.h>
#include <malloc.h>

#include <jpeg/jpeglib.h>

#define JPEG_QUALITY 100

void JpegInitDestination(j_compress_ptr cinfo) {
}

static boolean JpegEmptyOutputBuffer(j_compress_ptr cinfo) {
	return TRUE;
}

static void JpegTermDestination(j_compress_ptr cinfo) {
	//    jpegDstDataLen = jpegDstBufferLen - jpegDstManager.free_in_buffer;
}

/**
 Raw Rgb Data converted to Jpeg data
 */
boolean JpegCompress(int w, int h, const char * rgb_data, int rgb_size,
		char * jpeg_data, int *jpeg_size) {
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	struct jpeg_destination_mgr jpegDstManager;
	int ret, y;
	unsigned char *srcBuf = (unsigned char*) malloc(w * 3);
	JSAMPROW rowPointer[1];
	rowPointer[0] = (JSAMPROW) srcBuf;
	int left_size;
	left_size = *jpeg_size;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	cinfo.image_width = w;
	cinfo.image_height = h;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;
	cinfo.raw_data_in = TRUE;
	jpeg_set_defaults(&cinfo);

	cinfo.dest = &jpegDstManager;

	jpegDstManager.next_output_byte = (unsigned char*) jpeg_data;
	jpegDstManager.free_in_buffer = left_size;
	jpegDstManager.init_destination = JpegInitDestination;
	jpegDstManager.empty_output_buffer = JpegEmptyOutputBuffer;
	jpegDstManager.term_destination = JpegTermDestination;

	//jpeg_set_quality(&cinfo, 20, TRUE);

	jpeg_start_compress(&cinfo, TRUE);
	for (y = 0; y < h; y++) {
		rowPointer[0] = (unsigned char*) (rgb_data + y * w * 3);
		ret = jpeg_write_scanlines(&cinfo, rowPointer, 1);
	}
	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);

	*jpeg_size = left_size - jpegDstManager.free_in_buffer;
//TODO:哪个是正确的？
	(*jpeg_size) =
			((void*) jpegDstManager.next_output_byte - (void*) jpeg_data);

	return TRUE;
}

void JpegInitSource(j_decompress_ptr cinfo) {

}

boolean JpegFillInputBuffer(j_decompress_ptr cinfo) {
	/*
	 jpegError = true;
	 jpegSrcManager.bytes_in_buffer = jpegBufferLen;
	 jpegSrcManager.next_input_byte = (JOCTET *)jpegBufferPtr;    */
	return TRUE;
}

void JpegSkipInputData(j_decompress_ptr cinfo, long num_bytes) {/*
 if (num_bytes < 0 || (size_t)num_bytes > jpegSrcManager.bytes_in_buffer) {
 jpegError = true;
 jpegSrcManager.bytes_in_buffer = jpegBufferLen;
 jpegSrcManager.next_input_byte = (JOCTET *)jpegBufferPtr;
 } else {
 jpegSrcManager.next_input_byte += (size_t) num_bytes;
 jpegSrcManager.bytes_in_buffer -= (size_t) num_bytes;
 }*/
}

void JpegTermSource(j_decompress_ptr cinfo) {
	/* No work necessary here. */
}

boolean JpegUnCompress(const char * jpeg_data, int jpeg_size, char *rgb_data,
		int rgb_size, int w, int h) {
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;
	struct jpeg_source_mgr jpegSrcManager;
	int ret, dy;
	JSAMPROW rowPointer[1];
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);

	jpegSrcManager.init_source = JpegInitSource;
	jpegSrcManager.fill_input_buffer = JpegFillInputBuffer;
	jpegSrcManager.skip_input_data = JpegSkipInputData;
	jpegSrcManager.resync_to_restart = jpeg_resync_to_restart;
	jpegSrcManager.term_source = JpegTermSource;
	jpegSrcManager.next_input_byte = (unsigned char*) jpeg_data;
	jpegSrcManager.bytes_in_buffer = jpeg_size;
	cinfo.src = &jpegSrcManager;

	jpeg_read_header(&cinfo, TRUE);
	cinfo.out_color_space = JCS_RGB;
	jpeg_start_decompress(&cinfo);
	if (cinfo.output_width != (unsigned int) w
			&& cinfo.output_height != (unsigned int) h) {
		jpeg_destroy_decompress(&cinfo);
		return FALSE;
	}
	for (dy = 0; cinfo.output_scanline < cinfo.output_height; dy++) {
		rowPointer[0] = (unsigned char *) (rgb_data + w * dy * 3);
		ret = jpeg_read_scanlines(&cinfo, rowPointer, 1);
	}
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);
	return TRUE;
}

int CompressJPEG(unsigned int uWidth, unsigned int uHeight, unsigned char* pImg,
		int iQuality, unsigned char **out_jpg, int *size) {

	// setup JPEG compression structure data
	jpeg_compress_struct jcInfo;
	jpeg_error_mgr jErr; // JPEG error handler
	jcInfo.err = jpeg_std_error(&jErr);

	// initialize JPEG compression object
	jpeg_create_compress( &jcInfo);

	// specify data destination is memory
	jpeg_mem_dest(&jcInfo, out_jpg, size);

	// image format
	jcInfo.image_width = uWidth;
	jcInfo.image_height = uHeight;
	jcInfo.input_components = 3;
	jcInfo.in_color_space = JCS_RGB;

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

	return 1;
}

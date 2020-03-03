/*
    This driver supports Chrontel CH7025/CH7026 TV/VGA Encoder.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

enum input_format{
	CH7026_RGB888,
	CH7026_RGB666,
	CH7026_RGB565,
	CH7026_RGB555,
	CH7026_DVO,
	CH7026_YCbCr422_8bit,
	CH7026_YCbCr422_10bit,
	CH7026_YCbCr444_8bit,
	CH7026_Consecutive_RGB666 = 9,
	CH7026_Consecutive_RGB565,
	CH7026_Consecutive_RGB555
};
enum output_format{
	CH7026_NTSC_M,
	CH7026_NTSC_J,
	CH7026_NTSC_443,
	CH7026_PAL_B,
	CH7026_PAL_M,
	CH7026_PAL_N,
	CH7026_PAL_Nc,
	CH7026_PAL_60,
	CH7026_VGA_OUT //we only have CVBS output on AV-D1
};
enum RGB_SWAP{
	CH7026_RGB_ORDER,
	CH7026_RBG_ORDER,	
	CH7026_GRB_ORDER,
	CH7026_GBR_ORDER,
	CH7026_BRG_ORDER,
	CH7026_BGR_ORDER,
};
struct input_stream_info{
	int xres;
	int yres;
	enum input_format iformat;
	enum output_format oformat;
	enum RGB_SWAP swap;
};
void ch7026_set_input_stream(struct input_stream_info *info);
void ch7026_enable(bool enable);

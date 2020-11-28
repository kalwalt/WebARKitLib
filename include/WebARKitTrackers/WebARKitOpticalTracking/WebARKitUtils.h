#ifndef WEBARKIT_UTILS_H
#define WEBARKIT_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jpeglib.h"
#include <setjmp.h>

typedef struct {
    unsigned char       *image;
    int           nc;
    int           xsize;
    int           ysize;
    float         dpi;
} JpegImageT;

struct my_error_mgr {
    struct jpeg_error_mgr pub;	/* "public" fields */
    jmp_buf setjmp_buffer;	/* for return to caller */
};
typedef struct my_error_mgr * my_error_ptr;

static unsigned char *jpgread  (FILE *fp, int *w, int *h, int *nc, float *dpi);
JpegImageT *ar2ReadJpegImage( const char *filename, const char *ext );
JpegImageT *ar2ReadJpegImage2( FILE *fp );

JpegImageT *ar2ReadJpegImage( const char *filename, const char *ext )
{
    FILE           *fp;
    JpegImageT  *jpegImage;
    size_t          len;
    char           *buf1;


    len = strlen(filename) + strlen(ext) + 1;
    //arMalloc(buf1, char, len + 1); // +1 for nul terminator.
		buf1 = (char*) malloc(sizeof(char)*(len + 1));
    sprintf(buf1, "%s.%s", filename, ext);
    fp = fopen(buf1, "rb");
    if( fp == NULL ) {
        //ARLOGe("Error: Unable to open file '%s' for reading.\n", buf1);
        std::cout << "Error: Unable to open file '%s' for reading.\n" << std::endl;
        free(buf1);
        return (NULL);
    }
    free(buf1);

    jpegImage = ar2ReadJpegImage2(fp);

    fclose(fp);
    return jpegImage;
}

JpegImageT *ar2ReadJpegImage2( FILE *fp )
{
    JpegImageT  *jpegImage;

    //arMalloc( jpegImage, JpegImageT, 1 );
    jpegImage = (JpegImageT*) malloc(sizeof(JpegImageT)*(1));
    jpegImage->image = jpgread(fp, &(jpegImage->xsize), &(jpegImage->ysize), &(jpegImage->nc), &(jpegImage->dpi));

    if( jpegImage->image == NULL ) {
        free( jpegImage );
        return NULL;
    }

    return jpegImage;
}

#define BUFFER_HEIGHT 5

static void my_error_exit (j_common_ptr cinfo)
{
    /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
    my_error_ptr myerr = (my_error_ptr) cinfo->err;

    /* Always display the message. */
    /* We could postpone this until after returning, if we chose. */
    //(*cinfo->err->output_message) (cinfo);

    /* Return control to the setjmp point */
    longjmp(myerr->setjmp_buffer, 1);
}

static unsigned char *jpgread (FILE *fp, int *w, int *h, int *nc, float *dpi)
{
    struct jpeg_decompress_struct    cinfo;
    struct my_error_mgr              jerr;
    unsigned char                    *pixels;
    unsigned char                    *buffer[BUFFER_HEIGHT];
    int                              bytes_per_line;
    int                              row;
    int                              i;
    int                              ret;

    /* Initialize the JPEG decompression object with default error handling. */
    memset(&cinfo, 0, sizeof(cinfo));

    /* We set up the normal JPEG error routines, then override error_exit. */
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_exit;
    /* Establish the setjmp return context for my_error_exit to use. */
    if (setjmp(jerr.setjmp_buffer)) {
        /* If we get here, the JPEG code has signaled an error.
         * We need to clean up the JPEG object, close the input file, and return.
         */
        jpeg_destroy_decompress(&cinfo);
        //ARLOGe("Error reading JPEG file.\n");
        std::cout << "Error reading JPEG file.\n" << std::endl;
        return NULL;
    }

    jpeg_create_decompress(&cinfo);

    /* Specify data source for decompression */
    jpeg_stdio_src(&cinfo, fp);

    /* Read file header, set default decompression parameters */
    ret = jpeg_read_header(&cinfo, TRUE);
    if( ret != 1 ) {
        //ARLOGe("Error reading JPEG file header.\n");
        std::cout << "Error reading JPEG file header.\n" << std::endl;
        jpeg_destroy_decompress(&cinfo);
        return NULL;
    }

    /* Start decompressor */
    (void) jpeg_start_decompress(&cinfo);

    /* Allocate image buffer */
    bytes_per_line = cinfo.num_components * cinfo.image_width;
    pixels = (unsigned char *)malloc(bytes_per_line  * cinfo.image_height);
    if (!pixels) {
        //ARLOGe("Out of memory!!\n");
        std::cout << "Out of memory!!\n" << std::endl;
        jpeg_destroy_decompress(&cinfo);
        return NULL;
    }

    row = 0;

    /* Process data */
    while (cinfo.output_scanline < cinfo.output_height) {
        for (i=0; i<BUFFER_HEIGHT; ++i) {
            /* read in "upside down" because opengl says the
             * texture origin is lower left
             */
            //int rrow = cinfo.output_height - row - 1;
            //buffer[i] = &pixels[bytes_per_line * (rrow - i)];
            buffer[i] = &pixels[bytes_per_line * (row + i)];
        }
        row += jpeg_read_scanlines(&cinfo, buffer, BUFFER_HEIGHT);
    }

    (void) jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    if (w) *w = cinfo.image_width;
    if (h) *h = cinfo.image_height;
    if (nc) *nc = cinfo.num_components;
    if (dpi) {
        if( cinfo.density_unit == 1 && cinfo.X_density == cinfo.Y_density ) {
            *dpi = (float)cinfo.X_density;
        } else if( cinfo.density_unit == 2 && cinfo.X_density == cinfo.Y_density ) {
            *dpi = (float)cinfo.X_density * 2.54f;
        } else if (cinfo.density_unit > 2 && cinfo.X_density == 0 && cinfo.Y_density == 0) { // Handle the case with some libjpeg versions where density in DPI is returned in the density_unit field.
            *dpi = (float)(cinfo.density_unit);
        } else {
            *dpi = 0.0f;
        }
    }

    return pixels;
}

#endif

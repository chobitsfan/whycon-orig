#ifndef CIMAGE_H
#define CIMAGE_H

#include <stdlib.h>
#include <string.h>

/**
@author Tom Krajnik
Single-channel (mono8) image. The pattern detector modifies the pixels
(it blanks already found patterns), so the image owns a copy of the data.
*/
class CRawImage
{
public:
  CRawImage(int wi,int he);
  ~CRawImage();

  //copy mono8 data with the given row stride (in bytes) into the image
  void copyFrom(const unsigned char *src,int step);

  int width;
  int height;
  int bpp;
  int size;
  unsigned char* data;
};

#endif

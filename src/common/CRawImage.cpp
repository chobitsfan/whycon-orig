#include "CRawImage.h"

CRawImage::CRawImage(int wi,int he)
{
	width = wi;
	height = he;
	bpp = 1;
	size = bpp*width*height;
	data = (unsigned char*)calloc(size,sizeof(unsigned char));
}

CRawImage::~CRawImage()
{
	free(data);
}

void CRawImage::copyFrom(const unsigned char *src,int step)
{
	if (step == width){
		memcpy(data,src,size);
	}else{
		for (int j = 0;j<height;j++) memcpy(&data[j*width],&src[j*step],width);
	}
}

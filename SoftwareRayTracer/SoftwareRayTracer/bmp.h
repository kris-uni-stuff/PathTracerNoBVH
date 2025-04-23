#pragma once

#include <stdio.h>
#include <windows.h>
#include <wingdi.h>

int savebitmap(const char* filename, float* pixelBuffer, int w, int h)
{
	char* charBuffer = (char*)malloc(sizeof(unsigned char) * w * h * 3);

	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			charBuffer[(y * w * 3) + (x * 3) + 0] = pixelBuffer[((h - y) * w * 3) + (x * 3) + 2];
			charBuffer[(y * w * 3) + (x * 3) + 1] = pixelBuffer[((h - y) * w * 3) + (x * 3) + 1];
			charBuffer[(y * w * 3) + (x * 3) + 2] = pixelBuffer[((h - y) * w * 3) + (x * 3) + 0];
		}
	}

	BITMAPINFOHEADER infoHdr;
	infoHdr.biSize = 40;
	infoHdr.biWidth = w;
	infoHdr.biHeight = h;
	infoHdr.biPlanes = 1;
	infoHdr.biBitCount = 24;
	infoHdr.biCompression = 0;
	infoHdr.biSizeImage = sizeof(unsigned char) * w * h * 3;
	infoHdr.biXPelsPerMeter = 0;
	infoHdr.biYPelsPerMeter = 0;
	infoHdr.biClrUsed = 0;
	infoHdr.biClrImportant = 0;

	BITMAPFILEHEADER fileHdr;
	fileHdr.bfType = 19778;
	fileHdr.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + (sizeof(unsigned char) * w * h * 3);
	fileHdr.bfReserved1 = 0;
	fileHdr.bfReserved2 = 0;
	fileHdr.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

	FILE* bitmapFile;

	errno_t err = fopen_s(&bitmapFile, filename, "wb");
	if (err != 0 || bitmapFile == NULL)
	{
		printf("savebitmap - open failed for %s\n", filename);
		return NULL;
	}

	fwrite(&fileHdr, sizeof(BITMAPFILEHEADER), 1, bitmapFile);

	fwrite(&infoHdr, sizeof(BITMAPINFOHEADER), 1, bitmapFile);

	fseek(bitmapFile, fileHdr.bfOffBits, SEEK_SET);

	int nBytes = infoHdr.biWidth * infoHdr.biHeight * 3;
	fwrite(charBuffer, sizeof(unsigned char), nBytes, bitmapFile);

	fclose(bitmapFile);

	free(charBuffer);

	printf("savebitmap - saved %s w=%d h=%d bits=%d\n", filename, infoHdr.biWidth, infoHdr.biHeight, infoHdr.biBitCount);
}

float linear_to_gamma(float linear_component)
{
	if (linear_component > 0)
	{
		return std::sqrt(linear_component);
	}

	return 0;
}

int savehdr(const char* filename, float* pixelBuffer, int w, int h)
{
	FILE* bitmapFile;

	errno_t err = fopen_s(&bitmapFile, filename, "wb");
	if (err != 0 || bitmapFile == NULL)
	{
		printf("savehdr - open failed for %s\n", filename);
		return NULL;
	}

	printf( "P3\n %d %d\n255\n", w, h);

	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			auto r = pixelBuffer[((h - y) * w * 3) + (x * 3) + 2];
			auto g = pixelBuffer[((h - y) * w * 3) + (x * 3) + 1];
			auto b = pixelBuffer[((h - y) * w * 3) + (x * 3) + 0];

			r = linear_to_gamma(r);
			g = linear_to_gamma(g);
			b = linear_to_gamma(b);

			//	static const interval intensity(0.000, 0.999);
			//	int rbyte = int(256 * intensity.clamp(r));
			//	int gbyte = int(256 * intensity.clamp(g));
			//	int bbyte = int(256 * intensity.clamp(b));

			printf("%d %d %d\n", r, g, b);
		}
	}

}

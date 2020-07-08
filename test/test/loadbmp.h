#pragma once
#if ! defined (_LOADBMP_H)
#define _LOADBMP_H

#if ! defined (EXTRA_NAME)
#define EXTRA_NAME "~EX."
#endif

#include <afx.h>
#include <iostream>
#include <windows.h>
#include <math.h>
#include "string"
using namespace std;

#define pi (double)3.14159265359

int nWidth;
int nHeight;
int nLen;
int nByteWidth;
BYTE *lpBackup;
BYTE *lpBitmap;
BYTE *lpBits;
CString FileName;
CString Front;
CString Rear;
CString c;

/*复数定义*/
typedef struct
{
	double re;
	double im;
}COMPLEX;

/*复数加运算*/
COMPLEX Add(COMPLEX c1, COMPLEX c2)
{
	COMPLEX c;
	c.re = c1.re + c2.re;
	c.im = c1.im + c2.im;
	return c;
}

/*复数减运算*/
COMPLEX Sub(COMPLEX c1, COMPLEX c2)
{
	COMPLEX c;
	c.re = c1.re - c2.re;
	c.im = c1.im - c2.im;
	return c;
}

/*复数乘运算*/
COMPLEX Mul(COMPLEX c1, COMPLEX c2)
{
	COMPLEX c;
	c.re = c1.re*c2.re - c1.im*c2.im;
	c.im = c1.re*c2.im + c2.re*c1.im;
	return c;
}
COMPLEX dive(COMPLEX c1, COMPLEX c2)
{
	COMPLEX c;
	c.re = (c1.re*c2.re + c1.im*c2.im) / (c2.re*c2.re + c2.im*c2.im);
	c.im = (-c1.re*c2.im + c2.re*c1.im) / (c2.re*c2.re + c2.im*c2.im);
	return c;


}


void GetPoints(BYTE *lpPoints)
{
	int x, y, p;
	for (y = 0; y<nHeight; y++)
	{
		for (x = 0; x<nWidth; x++)
		{
			p = x * 3 + y*nByteWidth;
			lpPoints[x + y*nWidth] = (BYTE)(0.299*(float)lpBits[p + 2] + 0.587*(float)lpBits[p + 1] + 0.114*(float)lpBits[p] + 0.1);
		}
	}
}

void PutPoints(BYTE *lpPoints)
{
	int x, y, p, p1;
	for (y = 0; y<nHeight; y++)
	{
		for (x = 0; x<nWidth; x++)
		{
			p = x * 3 + y*nByteWidth;
			p1 = x + y*nWidth;
			lpBits[p] = lpPoints[p1];
			lpBits[p + 1] = lpPoints[p1];
			lpBits[p + 2] = lpPoints[p1];
		}
	}
}

void CuttheName()
{
	int Position;
	Position = FileName.Find('.');
	Front = FileName.Left(Position);
	Rear = FileName.Mid(Position + 1);
}

void LoadBitmap()
{
	BITMAPINFOHEADER *pInfo;
	pInfo = (BITMAPINFOHEADER *)(lpBitmap + sizeof(BITMAPFILEHEADER));
	nWidth = pInfo->biWidth;
	nByteWidth = nWidth * 3;
	if (nByteWidth % 4) nByteWidth += 4 - (nByteWidth % 4);
	nHeight = pInfo->biHeight;
	if (pInfo->biBitCount != 24)
	{
		if (pInfo->biBitCount != 8)
		{
			cout << "Invalidation Bitmap";
			delete lpBitmap;
			lpBitmap = 0;
			return;
		}
		unsigned int PaletteSize = 1 << pInfo->biBitCount;
		if (pInfo->biClrUsed != 0 && pInfo->biClrUsed<PaletteSize) PaletteSize = pInfo->biClrUsed;
		lpBits = lpBitmap + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
		RGBQUAD *pPalette = (RGBQUAD *)lpBits;
		lpBits += sizeof(RGBQUAD)*PaletteSize;
		nLen = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + nByteWidth*nHeight;
		BYTE *lpTemp = lpBitmap;
		lpBitmap = new BYTE[nLen];
		BITMAPFILEHEADER bmh;
		BITMAPINFOHEADER bmi;
		bmh.bfType = 'B' + 'M' * 256;
		bmh.bfSize = nLen;
		bmh.bfReserved1 = 0;
		bmh.bfReserved2 = 0;
		bmh.bfOffBits = 54;
		bmi.biSize = sizeof(BITMAPINFOHEADER);
		bmi.biWidth = nWidth;
		bmi.biHeight = nHeight;
		bmi.biPlanes = 1;
		bmi.biBitCount = 24;
		bmi.biCompression = BI_RGB;
		bmi.biSizeImage = 0;
		bmi.biXPelsPerMeter = 0;
		bmi.biYPelsPerMeter = 0;
		bmi.biClrUsed = 0;
		bmi.biClrImportant = 0;
		int nBWidth = pInfo->biWidth;
		if (nBWidth % 4) nBWidth += 4 - (nBWidth % 4);
		memset(lpBitmap, 0, nLen);
		memcpy(lpBitmap, &bmh, sizeof(BITMAPFILEHEADER));
		memcpy(lpBitmap + sizeof(BITMAPFILEHEADER), &bmi, sizeof(BITMAPINFOHEADER));
		BYTE *lpBits2 = lpBitmap + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
		int x, y, p1, p2, Palette;
		for (y = 0; y<nHeight; y++)
		{
			for (x = 0; x<nWidth; x++)
			{
				p1 = y*nBWidth + x;
				p2 = y*nByteWidth + x * 3;
				if (lpBits[p1]<PaletteSize) Palette = lpBits[p1];
				else Palette = 0;
				lpBits2[p2] = pPalette[Palette].rgbBlue;
				lpBits2[p2 + 1] = pPalette[Palette].rgbGreen;
				lpBits2[p2 + 2] = pPalette[Palette].rgbRed;
			}
		}
		delete lpTemp;
	}
	lpBits = lpBitmap + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	if (lpBackup) delete lpBackup;
	lpBackup = new BYTE[nLen];
	memcpy(lpBackup, lpBitmap, nLen);
}

void OpenFile()
{
	CFile File;
	if (!File.Open(FileName, CFile::modeRead)) return;
	CuttheName();
	if (lpBitmap) delete lpBitmap;
	nLen = File.GetLength();
	lpBitmap = new BYTE[nLen];
	File.Read(lpBitmap, nLen);
	LoadBitmap();
}

void SaveAs(CString pp)
{
	CFile file;
	if (lpBitmap == 0) return;
	if (!file.Open(Front + pp + "." + Rear, CFile::modeWrite | CFile::modeCreate))
	{
		cout << "ERROR~~\n";
		return;
	}
	int nLen = nByteWidth*nHeight;
	BYTE *pMem = new BYTE[nLen + sizeof(BITMAPINFOHEADER)];
	BITMAPINFOHEADER *bmi = (BITMAPINFOHEADER *)pMem;
	bmi->biSize = sizeof(BITMAPINFOHEADER);
	bmi->biWidth = nWidth;
	bmi->biHeight = nHeight;
	bmi->biPlanes = 1;
	bmi->biBitCount = 24;
	bmi->biCompression = BI_RGB;
	bmi->biSizeImage = 0;
	bmi->biXPelsPerMeter = 0;
	bmi->biYPelsPerMeter = 0;
	bmi->biClrUsed = 0;
	bmi->biClrImportant = 0;
	BITMAPFILEHEADER bmh;
	bmh.bfType = 'B' + 'M' * 256;
	bmh.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + nLen;
	bmh.bfReserved1 = 0;
	bmh.bfReserved2 = 0;
	bmh.bfOffBits = 54;
	memcpy(pMem + sizeof(BITMAPINFOHEADER), lpBits, nLen);
	file.Write(&bmh, sizeof(BITMAPFILEHEADER));
	file.Write(pMem, nLen + sizeof(BITMAPINFOHEADER));
	file.Close();
}
void SaveAs(char pp)
{
	CFile file;
	if (lpBitmap == 0) return;
	if (!file.Open(Front + pp + "." + Rear, CFile::modeWrite | CFile::modeCreate))
	{
		cout << "ERROR~~\n";
		return;
	}
	int nLen = nByteWidth*nHeight;
	BYTE *pMem = new BYTE[nLen + sizeof(BITMAPINFOHEADER)];
	BITMAPINFOHEADER *bmi = (BITMAPINFOHEADER *)pMem;
	bmi->biSize = sizeof(BITMAPINFOHEADER);
	bmi->biWidth = nWidth;
	bmi->biHeight = nHeight;
	bmi->biPlanes = 1;
	bmi->biBitCount = 24;
	bmi->biCompression = BI_RGB;
	bmi->biSizeImage = 0;
	bmi->biXPelsPerMeter = 0;
	bmi->biYPelsPerMeter = 0;
	bmi->biClrUsed = 0;
	bmi->biClrImportant = 0;
	BITMAPFILEHEADER bmh;
	bmh.bfType = 'B' + 'M' * 256;
	bmh.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + nLen;
	bmh.bfReserved1 = 0;
	bmh.bfReserved2 = 0;
	bmh.bfOffBits = 54;
	memcpy(pMem + sizeof(BITMAPINFOHEADER), lpBits, nLen);
	file.Write(&bmh, sizeof(BITMAPFILEHEADER));
	file.Write(pMem, nLen + sizeof(BITMAPINFOHEADER));
	file.Close();
}

#endif


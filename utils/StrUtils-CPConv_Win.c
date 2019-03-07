// String Utilities: Character Codepage Conversion
// ----------------
// using Windows API

#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <ctype.h>
#include <wchar.h>
#include <Windows.h>

#ifdef _MSC_VER
#define strdup		_strdup
#define stricmp		_stricmp
#define strnicmp	_strnicmp
#endif

#include <stdtype.h>
#include "StrUtils.h"

//typedef struct _codepage_conversion CPCONV;
struct _codepage_conversion
{
	// codpage name strings
	char* cpsFrom;
	char* cpsTo;
	// codepage IDs
	UINT cpiFrom;
	UINT cpiTo;
};

static const char* REGKEY_CP_LIST = "SOFTWARE\\Classes\\MIME\\Database\\Charset";

#define UTF16_NE	12000	// UTF-16 LE ("native endian")
#define UTF16_OE	12001	// UTF-16 BE ("opposite endian")

static UINT GetCodepageFromStr(const char* codepageName)
{
	// catch a few encodings that Windows calls differently from iconv
	if (! stricmp(codepageName, "UTF-16LE"))
		codepageName = "unicode";
	else if (! stricmp(codepageName, "UTF-16BE"))
		codepageName = "unicodeFFFE";
	// go the quick route for "CPxxx"
	if (! strnicmp(codepageName, "CP", 2) && isdigit((unsigned char)codepageName[2]))
		return (UINT)atoi(&codepageName[2]);
	
	if (! stricmp(codepageName, "UTF-8"))
		return 65001;
	else if (! stricmp(codepageName, "unicode"))
		return 12000;
	else
		return 0;
}

UINT8 CPConv_Init(CPCONV** retCPC, const char* cpFrom, const char* cpTo)
{
	CPCONV* cpc;
	
	cpc = (CPCONV*)calloc(1, sizeof(CPCONV));
	if (cpc == NULL)
		return 0xFF;
	
	cpc->cpiFrom = GetCodepageFromStr(cpFrom);
	if (! cpc->cpiFrom)
		return 0x80;
	cpc->cpiTo = GetCodepageFromStr(cpTo);
	if (! cpc->cpiTo)
		return 0x81;
	cpc->cpsFrom = strdup(cpFrom);
	cpc->cpsTo = strdup(cpTo);
	
	*retCPC = cpc;
	return 0x00;
}

void CPConv_Deinit(CPCONV* cpc)
{
	free(cpc->cpsFrom);
	free(cpc->cpsTo);
	
	free(cpc);
	
	return;
}

UINT8 CPConv_StrConvert(CPCONV* cpc, size_t* outSize, char** outStr, size_t inSize, const char* inStr)
{
	int wcBufSize;
	wchar_t* wcBuf;
	const wchar_t* wcStr;
	int convChrs;
	
	if (inSize == 0)
		inSize = strlen(inStr);
	if (inSize == 0)
	{
		*outSize = 0;
		return 0x02;	// nothing to convert
	}

	if (cpc->cpiFrom == UTF16_NE)	// UTF-16, native endian
	{
		wcBufSize = inSize / sizeof(wchar_t);
		wcBuf = NULL;
		wcStr = (const wchar_t*)inStr;
	}
	else if (cpc->cpiFrom == UTF16_OE)	// UTF-16, opposite endian
	{
		size_t curChrPos;
		char* wcBufPtr;
		
		wcBufSize = inSize / sizeof(wchar_t);
		wcBuf = (wchar_t*)malloc(wcBufSize * sizeof(wchar_t));
		wcBufPtr = (char*)wcBuf;
		for (curChrPos = 0; curChrPos < inSize; curChrPos += 0x02)
		{
			wcBufPtr[curChrPos + 0x00] = inStr[curChrPos + 0x01];
			wcBufPtr[curChrPos + 0x01] = inStr[curChrPos + 0x00];
		}
		wcStr = wcBuf;
	}
	else
	{
		wcBufSize = MultiByteToWideChar(cpc->cpiFrom, 0x00, inStr, inSize, NULL, 0);
		if (wcBufSize < 0 || (wcBufSize == 0 && inSize > 0))
			return 0xFF;
		wcBuf = (wchar_t*)malloc(wcBufSize * sizeof(wchar_t));
		wcBufSize = MultiByteToWideChar(cpc->cpiFrom, 0x00, inStr, inSize, wcBuf, wcBufSize);
		if (wcBufSize < 0)
			wcBufSize = 0;
		wcStr = wcBuf;
	}
	
	if (wcBufSize > 0 && wcStr[wcBufSize - 1] == L'\0')
		wcBufSize --;	// remove trailing \0 character
	convChrs = WideCharToMultiByte(cpc->cpiTo, 0x00, wcStr, wcBufSize, NULL, 0, NULL, NULL);
	if (convChrs < 0)
	{
		free(wcBuf);
		*outSize = 0;
		return 0x80;	// conversion error
	}
	*outSize = (size_t)convChrs;
	*outStr = (char*)malloc((*outSize + 1) * sizeof(char));
	convChrs = WideCharToMultiByte(cpc->cpiTo, 0x00, wcStr, wcBufSize, *outStr, *outSize, NULL, NULL);
	*outSize = (convChrs >= 0) ? (size_t)convChrs : 0;
	(*outStr)[*outSize] = '\0';
	
	free(wcBuf);
	return (convChrs >= 0) ? 0x00 : 0x01;
}

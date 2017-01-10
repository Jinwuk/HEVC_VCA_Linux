#ifndef FOURCC_H_INCLUDED
#define FOURCC_H_INCLUDED


typedef unsigned int		FourCC_t;

inline FourCC_t FourCC(int a, int b, int c, int d)
{
	return FourCC_t (d << 24 | c << 16 | b << 8 | a);
}

static inline int GetBitsPerPixel(FourCC_t fourcc)
{
	switch ( fourcc )
	{
	case 0x56555941: // AYUV
	case 0x30313256: // V210
		return 32;
	case 0x32555949: // IYU2
		return 24;
	case 0x76757963: // cyuv
	case 0x56595549: // IUYV
	case 0x43594448: // HDYC
	case 0x564E5955: // UYNV
	case 0x59565955: // UYVY
	case 0x32323456: // V422
	case 0x32323459: // Y422
	case 0x32595559: // YUY2
	case 0x56595559: // YUYV
	case 0x564E5559: // YUNV
	case 0x55595659: // YVYU
	case 0x36315659: // YV16
	case 0x32595843: // CXY2
		return 16;
	case 0x31345949: // IY41
	case 0x31555949: // IYU1
	case 0x50313459: // Y41P
	case 0x31313459: // Y411
	case 0x54313459: // Y41T
	case 0x32315659: // YV12
	case 0x30323449: // I420
	case 0x56555949: // IYUV
	case 0x3231564E: // NV12
	case 0x3132564E: // NV21
	case 0x31434D49: // IMC1
	case 0x32434D49: // IMC2
	case 0x33434D49: // IMC3
	case 0x34434D49: // IMC4
	case 0x4C504C43: // CLPL
	case 0x31595843: // CXY1
		return 12;
	case 0x39555659: // YVU9
		return 9;
	case 0x524A4C43: // CLJR
	case 0x59455247: // GREY
	case 0x31313259: // Y211
	case 0x30303859: // Y800
		return 8;
// 	case 0x57415249: // IRAW
//	case 0x50565955: // UYVP
// 	case 0x35353656: // V655
// 	case 0x59555956: // VYUY
//	case 0x50565559: // YUVP
//	case 0x39565559: // YUV9
//	case 0x39304649: // IF09
//	case 0x42313459: // Y41B
// 	case 0x30303859: // Y800
//		break;
	default:
		return 0;
	}
}

// four cc samples
/*
AYUV : 0x56555941 // AYUV
CLJR : 0x524A4C43 // CLJR
cyuv : 0x76757963 // cyuv
GREY : 0x59455247 // GREY
IRAW : 0x57415249 // IRAW
IUYV : 0x56595549 // IUYV
IY41 : 0x31345949 // IY41
IYU1 : 0x31555949 // IYU1
IYU2 : 0x32555949 // IYU2
HDYC : 0x43594448 // HDYC
UYNV : 0x564E5955 // UYNV
UYVP : 0x50565955 // UYVP
UYVY : 0x59565955 // UYVY
V210 : 0x30313256 // V210
V422 : 0x32323456 // V422
V655 : 0x35353656 // V655
VYUY : 0x59555956 // VYUY
Y422 : 0x32323459 // Y422
YUY2 : 0x32595559 // YUY2
YUYV : 0x56595559 // YUYV
YUNV : 0x564E5559 // YUNV
YVYU : 0x55595659 // YVYU
Y411 : 0x31313459 // Y411
Y211 : 0x31313259 // Y211
Y41T : 0x54313459 // Y41T
Y42T : 0x54323459 // Y42T
YUVP : 0x50565559 // YUVP
Y800 : 0x30303859 // Y800
YVU9 : 0x39555659 // YVU9
YUV9 : 0x39565559 // YUV9
IF09 : 0x39304649 // IF09
YV16 : 0x36315659 // YV16
YV12 : 0x32315659 // YV12
I420 : 0x30323449 // I420
IYUV : 0x56555949 // IYUV
NV12 : 0x3231564E // NV12
NV21 : 0x3132564E // NV21
IMC1 : 0x31434D49 // IMC1
IMC2 : 0x32434D49 // IMC2
IMC3 : 0x33434D49 // IMC3
IMC4 : 0x34434D49 // IMC4
CLPL : 0x4C504C43 // CLPL
Y41B : 0x42313459 // Y41B
Y800 : 0x30303859 // Y800
CXY1 : 0x31595843 // CXY1
CXY2 : 0x32595843 // CXY2
*/
#endif
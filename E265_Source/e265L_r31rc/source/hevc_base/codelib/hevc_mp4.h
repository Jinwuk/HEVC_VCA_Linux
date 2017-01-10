#ifndef _MP4_H_INCLUDED_
#define _MP4_H_INCLUDED_

#include <hevc_file.h>
#include <hevc_template.h>
#include <hevc_bitstream.h>
#include <hevc_smarts.h>
#include <list>

namespace hevc {
namespace mp4 {

typedef enum HDLRTYPE
{
	VIDEO = 0,
	SOUND,
	HINT
} hdlr_type_t;

typedef enum BOXTYPE
{
	TYPE_UNKNOWN = 0xFFFFFFFF,
	FTYP = 0x66747970,
	MOOV = 0x6d6f6f76,
	IODS = 0x696f6473,
	MVHD = 0x6d766864,
	TRAK = 0x7472616b,
	TKHD = 0x746b6864,
	MDIA = 0x6d646961,
	MDHD = 0x6d646864,
	HDLR = 0x68646c72,
	MINF = 0x6d696e66,
	VMHD = 0x766d6864,
	SMHD = 0x736d6864,
	DINF = 0x64696e66,
	STBL = 0x7374626c,
	STSD = 0x73747364,
	STTS = 0x73747473,
	CTTS = 0x63747473,
	STSC = 0x73747363,
	STSZ = 0x7374737a,
	STZ2 = 0x73747a32,
	STCO = 0x7374636f,
	CO64 = 0x636f3634,
	STSS = 0x73747373,
	URL0 = 0x75726c20,
	URN0 = 0x75726e20,
	DREF = 0x64726566,
	UUID = 0x75756964,
	AVCC = 0x61766343,
	BTRT = 0x62747274,
	MDAT = 0x6d646174,
	FREE = 0x66726565,
} box_type_t;

//typedef u32		box_type_t;

typedef enum {
	Box_len_32,
	Box_len_64,
} box_len_t;

typedef enum {
	mp4_err_fileopen = -2,
	mp4_err_unknown = -1,
	mp4_err_noerror = 0,
	mp4_err_invalid_bitstream_size = 1,
	mpr_err_invalid_data_field=2
} mp4_error_t;

static inline u32 box_type(const char* type)
{
	if (type == NULL)
		return 0;

	unsigned long ret = 0;
	for (int i = 0; i < 4; i++)
		ret |= type[i] << ((4-i-1)*8);
	return ret;
}

box_type_t detect_type(io::CFileAccess* pfMp4);
__stdstr GetUtf8StringFromBs(util::CBitStreamReader* pBs);

const TCHAR*	GetBoxTypeText( box_type_t box_type );

class Box;

class Box_Factory// : public design::CAbstractFactoryB<Box, util::CBitStreamReader*>
{
public:
	static Box* Create(util::CBitStreamReader* pBs);
	static Box* Create(box_type_t boxtype);
};

class Box_FactoryEx : public Box_Factory
{
public:
	static Box* Create(util::CBitStreamReader* pBs);
	static Box* Create(box_type_t boxtype);
protected:
private:
};

class CMp4FileReader : public hevc::io::CFileAccess
{
public:
	CMp4FileReader( );
	CMp4FileReader( const TCHAR* szFileName );

	util::CBitStreamReader*	GetCurrentBox();

protected:
private:
	util::CSafeBuffer	m_buffer;
	util::CBitStreamReader	m_bs;
};

#define BOX_CTOR(cls, type)	\
	cls(util::CBitStreamReader* pBs) : Box( pBs ) {}	\
	cls() : Box( type ) {} 
#define FBOX_CTOR(cls, type) \
	cls(util::CBitStreamReader* pBs) : FullBox( pBs ) {} \
	cls() : FullBox( type ) {}

class Box
{
public:
	// Decode mode ctor
	Box( util::CBitStreamReader* pBs ); 
	// Encode mode ctor
	Box( const char* szType );		//--
	Box( box_type_t type );
	virtual ~Box();

	u64			GetSize();
	void		SetSize(u64 size);	//--
	box_len_t	GetSizeBits();
	u32			GetType();
	void		SetType( box_type_t type );
	u8*			GetUserType() { return user_type; }
	void		SetUserType( const u8* pUserTypeUUID );

	void		AddChild( Box* pBox );

	mp4_error_t Decode();
	mp4_error_t	Encode( util::CBitStreamWriter* pBs );

	mp4_error_t GetLastError() { return m_error; }

	size_t	GetChildCount()		{ return m_childs.size(); }
	Box*	GetChild(size_t idx)
	{
		if ( idx >= GetChildCount() )
			return 0;
		return m_childs[idx].get();
	}

	virtual		u64			GetBodySize()	{ return body_size; }
	util::CBitStreamWriter*		GetWriter() { return m_pBsW; }

	u64			GetEncodeSize();
protected:
	virtual		mp4_error_t DecodeBody()	{ return mp4_err_noerror; };
	virtual		mp4_error_t EncodeBody()	{ return mp4_err_noerror; };

	mp4_error_t DecodeChild();
	mp4_error_t EncodeChild();

	virtual		u64			GetEncodeBodySize();
	u64			GetChildSize();

	u32		size;
	u32		type;
	u64		largesize;
	u8		user_type[17];
	u64		body_size;

	mp4_error_t		m_error;
	util::CBitStreamReader	*m_pBsR;
	util::CBitStreamWriter	*m_pBsW;

	std::vector<std::shared_ptr<Box>>		m_childs;

private:
	// disable default ctor
	Box( void ) {};
	// disable copy ctor
	Box(const Box& that) {};
};

class FullBox : public Box
{
protected:
	FullBox(util::CBitStreamReader* pBs);
	FullBox(const char* szType);
public:
	u8	GetVersion()		{ return version; }
	void SetVersion(u8 n)	{ version = n; }
	u32	GetFlags()			{ return flags; }
	void SetFlags(u32 n)	{ flags = n; }
protected:

	virtual		mp4_error_t DecodeBody();
	virtual		mp4_error_t EncodeBody();
	virtual		u64			GetEncodeBodySize()
	{
		return 12;
	}
	
	u8	version;
	u32	flags;
private:
};

static inline bool IsBoxError(Box* pBox) {
	if ( ! pBox ) return true;

	return pBox->GetLastError() != mp4_err_noerror;
}

// 'ftyp'
class FileTypeBox : public Box
{
public:
	BOX_CTOR(FileTypeBox, "ftyp");

	u32		GetMajorBrand()				{ return major_brand; }
	void	SetMajorBrand(u32 brand)	{ major_brand = brand; }
	u32		GetMinorVersion()				{ return minor_version; }
	void	SetMinorVersion(u32 version)	{ minor_version = version; }
	std::vector<u32>	GetCompatibleBrands()				{ return compatible_brands; }
	void	SetCompatibleBrands(std::vector<u32> brands)	{ compatible_brands = brands; }
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual mp4_error_t EncodeBody( void );

	virtual		u64			GetEncodeBodySize()
	{
		return __super::GetEncodeBodySize() + 8 + (4 * compatible_brands.size());
	}

	u32		major_brand;
	u32		minor_version;
	std::vector<u32>	compatible_brands;
private:
};

// 'mvhd'
class MovieHeaderBox : public FullBox
{
public:
	FBOX_CTOR(MovieHeaderBox, "mvhd");

	u64 GetCreationTime()		{ return creation_time; }
	u64 GetModificationTime()	{ return modification_time; }
	u32 GetTimeScale()			{ return timescale; }
	u64 GetDuration()			{ return duration; }

	n32	GetRate()		{ return rate; }
	n16 GetVolume()		{ return volume; }

	n32* GetMatrix()		{ return &matrix[0]; }
	u32* GetPreDefined()	{ return &pre_defined[0]; }
	u32 GetNextTrackId()	{ return next_track_ID; }

protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u64 creation_time;
	u64 modification_time;
	u32 timescale;
	u64 duration;

	n32 rate;
	n16 volume;

	u16	reserved1;
	u32 reserved2[2];
	n32 matrix[9];
	u32 pre_defined[6];
	u32 next_track_ID;
};

class TrackHeaderBox : public FullBox
{
public:
	FBOX_CTOR(TrackHeaderBox, "tkhd");
	
	u64		GetCreationTime()		{ return creation_time; }
	u64		GetModificationTime()	{ return modification_time; }
	u32		GetTrackId()			{ return track_ID; }
	u32		GetDuration()			{ return duration; }
	n16		GetLayer()				{ return layer; }
	n16		GetAlternateGroup()		{ return alternate_group; }
	n16		GetVolume()				{ return volume; }
	n32*	GetMatrix()				{ return &matrix[0]; }
	u32		GetWidth()				{ return width; }
	u32		GetHeight()				{ return height; }
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	// version==1
	u64 creation_time;
	u64 modification_time;
	u32 track_ID;
	u32 reserved1; // = 0;
	// version==0
/*
	unsigned int(32) creation_time;
	unsigned int(32) modification_time;
	unsigned int(32) track_ID;
	const unsigned int(32) reserved = 0;
*/
	u32 duration;

	u32 reserved2[2]; // = 0;
	n16 layer; // = 0
	n16 alternate_group; //  = 0;
	n16 volume; // {if track_is_audio 0x0100 else 0};
	u16 reserved3; // = 0;
	n32 matrix[9]; // = { 0x00010000,0,0,0,0x00010000,0,0,0,0x40000000 }; 	// unity matrix
	u32 width;
	u32 height;
private:
};

class EditListBox : public FullBox
{
public:
	FBOX_CTOR(EditListBox, "elst");

	u64	GetSegmentDuration()	{ segment_duration; }
	u64	GetMediaTime()			{ media_time; }
	u64	GetMediaRateInteger()	{ media_rate_integer; }
	u64	GetMediaRateFraction()	{ media_rate_fraction; }
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u64 segment_duration;
	n64 media_time;
	n16 media_rate_integer;
	n16 media_rate_fraction; // = 0
private:
};

// 'mdhd'
class MediaHeaderBox : public FullBox
{
public:
	FBOX_CTOR(MediaHeaderBox, "mdhd");
	u64 GetCreationTime()		{ return creation_time; }
	u64 GetModificationTime()	{ return modification_time; }
	u32 GetTimeScale()			{ return timescale; }
	u64 GetDuration()			{ return duration; }
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u64 creation_time;
	u64 modification_time;
	u32 timescale;
	u64 duration;

	u8  language[3];
	u16 pre_defined; // = 0
private:
};

// 'hdlr'
class HandlerBox : public FullBox
{
public:
	FBOX_CTOR(HandlerBox, "hdlr");

	u32			GetPreDefined()			{ return pre_defined; }
	void		SetPreDefined(u32 n)	{ pre_defined = n; }
	u32			GetHandlerType()		{ return handler_type; }
	void		SetHandlerType(u32 n)	{ handler_type = n; }
	__stdstr	GetName()				{ return name; }
	void		SetName(__stdstr nm);
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual mp4_error_t EncodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u32		pre_defined;
	u32		handler_type;
	u32		reserved[3];
	__stdstr name; 
private:
};

// 'vmhd' 
class VideoMediaHeaderBox : public FullBox
{
public:
	FBOX_CTOR(VideoMediaHeaderBox, "vmhd");
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u16 graphicsmode; // = 0
	u16 opcolor[3]; // = { 0, 0, 0 }
private:
};

// 'url '
class DataEntryUrlBox : public FullBox
{
public:
	FBOX_CTOR(DataEntryUrlBox, "url ");

	__stdstr GetLocation() { return location; }
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	__stdstr location;
};

// 'urn '
class DataEntryUrnBox : public FullBox 
{
public:
	FBOX_CTOR(DataEntryUrnBox, "urn ");
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	__stdstr name;
	__stdstr location;
};

// 'dref'
class DataReferenceBox : public FullBox
{
public:
	FBOX_CTOR(DataReferenceBox, "dref");
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u32 entry_count;
};

// abstract class
class SampleEntry : public Box
{
public:
	BOX_CTOR(SampleEntry, "");

	u16 GetDataReferenceIndex() { return data_reference_index; }
protected:
	virtual mp4_error_t DecodeBody( void );
//	u8 reserved[6];	// = 0
	u16 data_reference_index;
private:
};

class VisualSampleEntry : public SampleEntry
{
public:
protected:
	u16 pre_defined1; // = 0;
	//u16 reserved = 0;
	u32 pre_defined2[3]; // = 0;
	u16 width;
	u16 height;
	u32 horizresolution; // = 0x00480000; // 72 dpi
	u32 vertresolution; // = 0x00480000; // 72 dpi
	//u32 reserved = 0;
	u16 frame_count; // = 1;
	char compressorname[33]; // fixed 32 bytes
	u16 depth; // = 0x0018;
	n16 pre_defined3; // = -1;
//	CleanApertureBox clap; // optional
//	PixelAspectRatioBox pasp; // optional
};

class AudioSampleEntry : public SampleEntry
{
public:
//	BOX_CTOR(AudioSampleEntry);
protected:
	//const u32[2] reserved = 0;
	u16 channelcount; // = 2;
	u16 samplesize; // = 16;
	u16 pre_defined; // = 0;
	//const u16 reserved = 0 ;
	u32 samplerate; // = { default samplerate of media}<<16;
};

class SampleDescriptionBox : public FullBox
{
public:
	FBOX_CTOR(SampleDescriptionBox, "");
protected:
	int i ;
	u32 entry_count;
// 	for (i = 1 ; i <= entry_count ; i++)
// 	{
// 		switch (handler_type)
// 		{
// 			case ¡®soun¡¯: // for audio tracks
// 			AudioSampleEntry();
// 			break;
// 			case ¡®vide¡¯: // for video tracks
// 			VisualSampleEntry();
// 			break;
// 			case ¡®hint¡¯: // Hint track
// 			HintSampleEntry();
// 			break;
// 			case ¡®meta¡¯: // Metadata track
// 			MetadataSampleEntry();
// 			break; }
// 		}
// 	}
};

} // namespace mp4
} // namespace hevc

#endif // _MP4_H_INCLUDED_
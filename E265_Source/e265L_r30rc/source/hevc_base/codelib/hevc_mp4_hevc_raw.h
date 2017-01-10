#include "./hevc_mp4.h"

#pragma once

namespace hevc {
namespace mp4 {

/*
enum {
	STIF = SKIP,
	STFM,
	STSO,
	TIST,
};
*/

enum {
	STIF = 0x73746966,
	STFM = 0x7374666d,
	STSO = 0x7374736f,
	TIST = 0x74697374,
};

static const u32 HEVC_4K_RAW_DATA = box_type("4krw");

const TCHAR*	GetBoxTypeTextEx( box_type_t box_type );


// 'stif'
class StreamInfoBox : public Box
{
public:
	BOX_CTOR(StreamInfoBox, "stif");

	u32		GetStreamCount()		{ return stream_count; }
	void	SetStreamCount(u32 cnt) { stream_count = cnt; }
	u32		GetSampleCount()		{ return sample_count; }
	void	SetSampleCount(u32 cnt) { sample_count = cnt; }
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual mp4_error_t EncodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u32		stream_count;
	u32		sample_count;
};

// 'stfm'
class StreamFormatBox : public Box
{
public:
	BOX_CTOR(StreamFormatBox, "stfm");

	u32		GetStreamId()		{ return stream_id; }
	u32		GetStreamType()		{ return stream_type; }
	u8		GetSampleBits()		{ return sample_bits; }
	u8		GetChannels()		{ return channels; }
	u16		GetSampleRate()		{ return sample_rate; }
	u16		GetWidth()			{ return width; }
	u16		GetHeight()			{ return height; }
	u8		GetBitsPerPixel()	{ return bits_per_pixel; }
	u8		GetBitDepth()		{ return bit_depth; }
	u32		GetFourCC()			{ return fourCC; }

	void	SetStreamId( u32 n )	{ stream_id = n; }
	void	SetStreamType( u32 n )	{ stream_type = n; }
	void	SetSampleBits( u8 n )	{ sample_bits = n; }
	void	SetChannels( u8 n )		{ channels = n; }
	void	SetSampleRate( u16 n )	{ sample_rate = n; }
	void	SetWidth( u16 n )		{ width = n; }
	void	SetHeight( u16 n )		{ height = n; }
	void	SetBitsPerPixel( u8 n )	{ bits_per_pixel = n; }
	void	SetBitDepth( u8 n )		{ bit_depth = n; }
	void	SetFourCC( u32 n )		{ fourCC = n; }

	static const u32	stream_type_pcma = 0x70636d61;	// pcm audio - 'pcma'
	static const u32	stream_type_yuvr = 0x79757672;	// yuv raw - 'yuvr'
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual mp4_error_t EncodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u32		stream_id;
	u32		stream_type;

	// when stream_type = 'pcma'
	u8		sample_bits;
	u8		channels;
	u16		sample_rate;

	// stream_type = 'yuvr'
	u16		width;
	u16		height;
	u8		bits_per_pixel;
	u8		bit_depth;
	u32		fourCC;
private:
};

class TimestampBox : public Box
{
public:
	BOX_CTOR(TimestampBox, "tist");

	u64		GetTist()			{ return tist; }
	void	SetTist( u64 n )	{ tist = n; }
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual mp4_error_t EncodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u64		tist;
private:
};

class SampleOffset
{
public:
	u8		sample_index;
	u64		offset;
	u64		tist;
};

// 'stso'
class StreamSampleOffsetBox : public Box
{
public:
	BOX_CTOR(StreamSampleOffsetBox, "stso");

	u32		GetStreamId()			{ return stream_id; }
	void	SetStreamId(u32 n)		{ stream_id = n; }
	u32		GetSampleCount()		{ return sample_count; }
	void	SetSampleCount(u32 n)	{ sample_count = n; }

	std::vector<SampleOffset>*	GetOffset() { return &offset; }
	void	SetOffset(std::vector<SampleOffset>& off) 
	{
		offset = off;
	}
	void	AddOffset(SampleOffset& ofst)
	{
		offset.push_back(ofst);
	}
protected:
	virtual mp4_error_t DecodeBody( void );
	virtual mp4_error_t	EncodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u32		stream_id;
	u32		sample_count;
	std::vector<SampleOffset>	offset;
};

// 'mdat' -- for HEVC RAW extension
class RawMediaDataBox : public Box
{
public:
	BOX_CTOR(RawMediaDataBox, "mdat");

	void	SetAudioOffset( std::vector<SampleOffset> * pOffset )
	{
		a_offset = *pOffset;
	}
	void	SetVideoOffset( std::vector<SampleOffset> * pOffset )
	{
		v_offset = *pOffset;
	}
	void	SetMediaDataSize(u64 n)
	{
		media_data_size = n;
	}
protected:
	virtual mp4_error_t	EncodeBody( void );
	virtual mp4_error_t DecodeBody( void );
	virtual	u64			GetEncodeBodySize();

	u64		media_data_size;
	std::vector<SampleOffset>	a_offset;
	std::vector<SampleOffset>	v_offset;
};

} // namespace mp4
} // namespace hevc

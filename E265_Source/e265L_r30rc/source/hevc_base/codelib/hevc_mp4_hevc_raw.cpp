#include "./hevc_mp4_hevc_raw.h"
#include <tchar.h>

namespace hevc {
namespace mp4 {

const TCHAR*	GetBoxTypeTextEx( box_type_t box_type )
{
	switch ( u32(box_type) )
	{
	case STIF:
		return _T("Stream information box");
	case STFM:
		return _T("Stream format box");
	case STSO:
		return _T("Stream sample offset box");
	case TIST:
		return _T("Timestamp box");
	default:
		return GetBoxTypeText(box_type);
	}
}


//-----------------------------------------------------------------------
Box* Box_FactoryEx::Create(util::CBitStreamReader* pBs)
{
	size_t remains = pBs->GetRemainBytes();
	if ( remains < 8 ) return 0;

	u64 size = (u32)(pBs->Peek(64) >> 32);
	box_type_t type = (box_type_t)(u32)pBs->Peek(64);

	if ( type != MDAT )
	{
		Box* pBox = Box_Factory::Create( pBs );

		if ( pBox ) return pBox;
	}

	switch ( u32(type) )
	{
	case STIF:
		return new StreamInfoBox( pBs );
	case STFM:
		return new StreamFormatBox( pBs );
	case STSO:
		return new StreamSampleOffsetBox( pBs );
	case TIST:
		return new TimestampBox( pBs );
	case MDAT:
		return new RawMediaDataBox( pBs );
	}

	return 0;
}

Box* Box_FactoryEx::Create(box_type_t boxtype)
{
	if ( boxtype != MDAT )
	{
		Box* pBox = Box_Factory::Create( boxtype );

		if ( pBox ) return pBox;
	}

	switch ( u32(boxtype) )
	{
	case STIF:
		return new StreamInfoBox();
	case STFM:
		return new StreamFormatBox();
	case STSO:
		return new StreamSampleOffsetBox();
	case TIST:
		return new TimestampBox();
	case MDAT:
		return new RawMediaDataBox();
	}

	return 0;
}

//-----------------------------------------------------------------------
mp4_error_t StreamInfoBox::DecodeBody( void )
{
	if ( m_pBsR->GetTotalBytes() < 8 )
		return mp4_err_invalid_bitstream_size;

	stream_count = (u32)m_pBsR->Get(32);
	sample_count = (u32)m_pBsR->Get(32);

	return mp4_err_noerror;
}

mp4_error_t StreamInfoBox::EncodeBody( void )
{
	mp4_error_t err = __super::EncodeBody();
	if ( err != mp4_err_noerror )
		return err;

	m_pBsW->Set( stream_count, 32 );
	m_pBsW->Set( sample_count, 32 );

	return mp4_err_noerror;
}

u64			StreamInfoBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + 8;
}

//-----------------------------------------------------------------------
mp4_error_t StreamFormatBox::DecodeBody( void )
{
	if ( m_pBsR->GetTotalBytes() < 8 )
		return mp4_err_invalid_bitstream_size;

	stream_id = (u32)m_pBsR->Get(32);
	stream_type = (u32)m_pBsR->Get(32);

	if ( StreamFormatBox::stream_type_pcma == stream_type )
	{
		sample_bits = (u8)m_pBsR->Get(8);
		channels = (u8)m_pBsR->Get(8);
		sample_rate = (u16)m_pBsR->Get(16);
	}
	else if ( StreamFormatBox::stream_type_yuvr == stream_type )
	{
		width = (u16)m_pBsR->Get(16);
		height = (u16)m_pBsR->Get(16);
		bits_per_pixel = (u8)m_pBsR->Get(8);
		bit_depth = (u8)m_pBsR->Get(8);
		m_pBsR->Throw(16);
		fourCC = (u32)m_pBsR->Get(32);
	}
	else 
		return mpr_err_invalid_data_field;

	return mp4_err_noerror;
}

mp4_error_t StreamFormatBox::EncodeBody( void )
{
	mp4_error_t err = __super::EncodeBody();
	if ( err != mp4_err_noerror )
		return err;

	m_pBsW->Set( stream_id, 32 );
	m_pBsW->Set( stream_type, 32 );

	if ( stream_type == StreamFormatBox::stream_type_pcma )
	{
		m_pBsW->Set( sample_bits, 8 );
		m_pBsW->Set( channels, 8 );
		m_pBsW->Set( sample_rate, 16 );
	}
	else if ( stream_type == StreamFormatBox::stream_type_yuvr )
	{
		m_pBsW->Set( width, 16 );
		m_pBsW->Set( height, 16 );
		m_pBsW->Set( bits_per_pixel, 8 );
		m_pBsW->Set( bit_depth, 8 );
		m_pBsW->Set( 0, 16 );
		m_pBsW->Set( fourCC, 32 );
	}
	else
		return mpr_err_invalid_data_field;

	return mp4_err_noerror;
}

u64	StreamFormatBox::GetEncodeBodySize()
{
	if ( StreamFormatBox::stream_type_pcma == stream_type )
		return __super::GetEncodeBodySize() + 12;
	else 
		return __super::GetEncodeBodySize() + 20;
}

//-----------------------------------------------------------------------
mp4_error_t TimestampBox::DecodeBody( void )
{
	if ( m_pBsR->GetTotalBytes() < 8 )
		return mp4_err_invalid_bitstream_size;

	tist = m_pBsR->Get(64);

	return mp4_err_noerror;
}

mp4_error_t TimestampBox::EncodeBody( void )
{
	mp4_error_t err = __super::EncodeBody();
	if ( err != mp4_err_noerror )
		return err;

	m_pBsW->Set( tist, 64 );

	return mp4_err_noerror;
}

u64			TimestampBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + 8;
}

//-----------------------------------------------------------------------
mp4_error_t StreamSampleOffsetBox::DecodeBody( void )
{
	if ( m_pBsR->GetTotalBytes() < 8 )
		return mp4_err_invalid_bitstream_size;

	stream_id = (u32)m_pBsR->Get(32);
	sample_count = (u32)m_pBsR->Get(32);

	//for (size_t ii = 0; ii < )
	for ( u32 ii = 0; ii < sample_count; ii++ )
	{
		SampleOffset off;
		off.sample_index = (u8)m_pBsR->Get(8);
		m_pBsR->Throw(24);
		off.offset = m_pBsR->Get(64);
		off.tist	= m_pBsR->Get(64);

		offset.push_back(off);
	}

	return mp4_err_noerror;
}

mp4_error_t	StreamSampleOffsetBox::EncodeBody( void )
{
	mp4_error_t err = __super::EncodeBody();
	if ( err != mp4_err_noerror )
		return err;

	m_pBsW->Set( stream_id, 32 );
	m_pBsW->Set( sample_count, 32 );

	for (size_t ii = 0; ii < offset.size(); ii++ )
	{
		m_pBsW->Set( offset[ii].sample_index, 8 );
		m_pBsW->Set( 0, 24 );
		m_pBsW->Set( offset[ii].offset, 64);
		m_pBsW->Set( offset[ii].tist, 64 );
	}

	return mp4_err_noerror;
}

u64			StreamSampleOffsetBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + 8 + (offset.size() * 20);
}

//-----------------------------------------------------------------------
// constant nums - for test
#define VIDEO_SAMPLE_SIZE			(24883200)					// 3840 * 2160 * 24 (bitsperpixel) / 8
#define AUDIO_SAMPLE_SIZE			(51200)						// (48000 * 32 * 2) / 60 (fps)

mp4_error_t	RawMediaDataBox::EncodeBody( void )
{
	mp4_error_t err = __super::EncodeBody();
	if ( err != mp4_err_noerror )
		return err;

	// test video data
	for ( size_t ii = 0; ii < v_offset.size(); ii++ )
	{
		if ((ii%2)==0)
			memset(m_pBsW->GetCurrentPtr(), 'V', VIDEO_SAMPLE_SIZE);
		else
			memset(m_pBsW->GetCurrentPtr(), 'v', VIDEO_SAMPLE_SIZE);

		m_pBsW->Skip(VIDEO_SAMPLE_SIZE*8);
	}

	// test audio data
	for ( size_t ii = 0; ii < a_offset.size(); ii++ )
	{
		if ((ii%2)==0)
			memset(m_pBsW->GetCurrentPtr(), 'A', AUDIO_SAMPLE_SIZE);
		else
			memset(m_pBsW->GetCurrentPtr(), 'a', AUDIO_SAMPLE_SIZE);

		m_pBsW->Skip(AUDIO_SAMPLE_SIZE*8);
	}

	return mp4_err_noerror;
}

mp4_error_t RawMediaDataBox::DecodeBody( void )
{
	m_pBsR->Throw( (GetSize() - 8) *8 );

	return mp4_err_noerror;
}

u64			RawMediaDataBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + media_data_size;
}

} // namespace mp4
} // namespace hevc

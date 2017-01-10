#include "./hevc_mp4.h"
#include <hevc_charHelper.h>
#include <tchar.h>

namespace hevc {
namespace mp4 {

/*
static const u32 type_repo[] = {
	box_type("ftyp"),
	box_type("moov"),
	box_type("iods"),
	box_type("mvhd"),
	box_type("trak"),
	box_type("tkhd"),
	box_type("mdia"),
	box_type("mdhd"),
	box_type("hdlr"),
	box_type("minf"),
	box_type("vmhd"),
	box_type("smhd"),
	box_type("dinf"),
	box_type("stbl"),
	box_type("stsd"),
	box_type("stts"),
	box_type("ctts"),
	box_type("stsc"),
	box_type("stsz"),
	box_type("stz2"),
	box_type("stco"),
	box_type("co64"),
	box_type("stss"),
	box_type("url "),
	box_type("urn "),
	box_type("dref"),
	box_type("uuid"),
	box_type("avcC"),
	box_type("btrt"),
	box_type("mdat"),
	box_type("free"),
};
*/

inline u64	GetBsGetSize(const unsigned char* pStart, const unsigned char* pEnd)
{
	return u64(pEnd - pStart);
}

#define START_CHECK_BS_GET_SIZE(pBs) \
	const unsigned char * __pStart = (const unsigned char *)pBs->GetCurrentPtr(); \

#define CHECK_BS_GET_SIZE(pBs) GetBsGetSize(__pStart, (const unsigned char *)pBs->GetCurrentPtr())

u64		detect_box_size(io::CFileAccess* pfMp4)
{
	n64 peek_data_len =0;
	if ( !pfMp4 ) return 0;

	u8	pRead[16];
	u64 ret;

	// size
	u32 size = (u32)pfMp4->Read( pRead, 4 );
	peek_data_len -= 4;

	// type
	(box_type_t)pfMp4->Read( pRead, 4 );
	peek_data_len -= 4;

	// largesize
	if ( 1 == size )
	{
		u64 large = pfMp4->Read( pRead, 8 );
		peek_data_len -= 8;

		ret = large;
	}
	else if ( 0 == size )
	{
		ret = pfMp4->GetFileRemainBytes() + 8;
	}
	else
	{
		ret = size;
	}

	return ret;
}

bool	detect_box_size_and_type(io::CFileAccess* pfMp4, u64 &box_size, box_type_t &_box_type )
{
	n64 peek_data_len =0;
	if ( !pfMp4 ) return 0;

	u8	pRead[16];
	bool f = true;
	// size
	if ( 8 != (u32)pfMp4->Read( pRead, 8 ) )
		return false;
	peek_data_len -= 8;

	// type
	u32 size = util::p2ul(&pRead[0]);
	_box_type = (box_type_t)util::p2ul(&pRead[4]);

	// largesize
	if ( 1 == size )
	{
		u64 large = pfMp4->Read( pRead, 8 );
		peek_data_len -= 8;

		if ( large > 0 )
			box_size = large;
		else
		{
			box_size = 0;
			f = false;
		}
	}
	else if ( 0 == size )
	{
		box_size = pfMp4->GetFileRemainBytes() + 8;
	}
	else
	{
		box_size = size;
	}

	pfMp4->Seek( peek_data_len, hevc::io::CFileAccess::Seek_Cur );

	return f;
}

box_type_t detect_type(io::CFileAccess* pfMp4)
{
	int ret = (int)pfMp4->Seek(4, io::CFileAccess::Seek_Cur);
	//int ret = fseek(fmp4, 4, SEEK_CUR);

	char type[5] = {0, };
	pfMp4->Read(type, 4);

	return (box_type_t)box_type(type);
// 	if (ret == 0)
// 		return SKIP;
// 	u32 nBoxType = box_type(type);
// 
// 	pfMp4->Seek(-8, io::CFileAccess::Seek_Cur);
// 
// 	for (int i = 0; i < SKIP; i++)
// 	{
// 		if (type_repo[i] == nBoxType)
// 			return (BOXTYPE)i;
// 	}
// 
// 	return SKIP;
}

box_type_t detect_type( util::CBitStreamReader *bs )
{
	if ( ! bs ) return TYPE_UNKNOWN;

	if ( bs->GetRemainBytes() < 8 ) return TYPE_UNKNOWN;

	u32 nBoxType = (u32)bs->Peek(64);

	return (box_type_t)nBoxType;

// 	for (int i = 0; i < SKIP; i++)
// 	{
// 		if (type_repo[i] == nBoxType)
// 			return (BOXTYPE)i;
// 	}

//	return (box_type_t)SKIP;
}

box_type_t detect_type( u32 type )
{
	return box_type_t(type);
// 	for (int i = 0; i < SKIP; i++)
// 	{
// 		if (type_repo[i] == type)
// 			return (BOXTYPE)i;
// 	}

//	return (box_type_t)SKIP;
}

const TCHAR*	GetBoxTypeText( box_type_t box_type )
{
	switch ( box_type )
	{
	case FTYP:
		return _T("File type box");
	case MOOV:
		return _T("Movie box");
	case IODS:
		return _T("IOD box");
	case MVHD:
		return _T("Movie Header box");
	case TRAK:
		return _T("Track box");
	case TKHD:
		return _T("Track header box");
	case MDIA:
		return _T("Media box");
	case MDHD:
		return _T("Media header box");
	case HDLR:
		return _T("Handler box");
	case MINF:
		return _T("Media information box");
	case VMHD:
		return _T("Video media header box");
	case SMHD:
		return _T("Sound media header box");
	case DINF:
		return _T("Data information box");
	case STBL:
		return _T("Sample table box");
	case STSD:
		return _T("Sample description box");
	case STTS:
		return _T("Decoding time to sample box");
	case CTTS:
		return _T("Composition time to sample box");
	case STSC:
		return _T("Sample to chunk box");
	case STSZ:
		return _T("Sample size box");
	case STZ2:
		return _T("Compact sample size box");
	case STCO:
		return _T("Chunk offset box");
	case CO64:
		return _T("Chunk large offset box");
	case STSS:
		return _T("Sync sample box");
	case URL0:
		return _T("Data entry url box");
	case URN0:
		return _T("Data entry urn box");
	case DREF:
		return _T("Data reference box");
	case UUID:
		return _T("UUID box");
	case AVCC:
		return _T("AVC decoding config box");
	case BTRT:
		return _T("BTRT box");
	case MDAT:
		return _T("Media data box");
	case FREE:
		return _T("Free space box");
	default:
		return _T("Unknown");
	}
}

__stdstr GetUtf8StringFromBs(util::CBitStreamReader* pBs)
{
	if ( !pBs || pBs->GetRemainBytes() < 1 )
	{
		return _T("");
	}

	unsigned char* p = (unsigned char*)pBs->GetCurrentPtr();
	if ( !p )
	{
		return _T("");
	}

	util::CCharHelper str8(p);
	pBs->Throw((str8.GetByteLen8()+1)*8);

	return __stdstr(str8);
}

//-----------------------------------------------------------------------
Box* Box_Factory::Create(util::CBitStreamReader* pBs)
{
	size_t remains = pBs->GetRemainBytes();
	if ( remains < 8 ) return 0;

	u64 size = (u32)(pBs->Peek(64) >> 32);
	box_type_t type = (box_type_t)(u32)pBs->Peek(64);

	switch ( type )
	{
	case FTYP:
		return new FileTypeBox( pBs );
	case MVHD:
		return new MovieHeaderBox( pBs );
	case TKHD:
		return new TrackHeaderBox( pBs );
	case MDHD:
		return new MediaHeaderBox( pBs );
	case HDLR:
		return new HandlerBox( pBs );
	case MINF:
	case VMHD:
	case SMHD:
	case DINF:
	case STBL:
	case STSD:
	case STTS:
	case CTTS:
	case STSC:
	case STSZ:
	case STZ2:
	case STCO:
	case CO64:
	case STSS:
	case URL0:
	case URN0:
	case DREF:
	case UUID:
	case AVCC:
	case IODS:
	// container boxes
	case MDIA:
	case MOOV:
	case TRAK:
	case BTRT:
		return new Box( pBs );
	default:
		return NULL;
	}
}

Box* Box_Factory::Create(box_type_t boxtype)
{
	switch ( detect_type(boxtype) )
	{
	case FTYP:
		return new FileTypeBox();
	case MVHD:
		return new MovieHeaderBox();
	case TKHD:
		return new TrackHeaderBox();
	case MDHD:
		return new MediaHeaderBox();
	case HDLR:
		return new HandlerBox();
	case MINF:
	case VMHD:
	case SMHD:
	case DINF:
	case STBL:
	case STSD:
	case STTS:
	case CTTS:
	case STSC:
	case STSZ:
	case STZ2:
	case STCO:
	case CO64:
	case STSS:
	case URL0:
	case URN0:
	case DREF:
	case UUID:
	case AVCC:
	case IODS:
	case MOOV:
	case BTRT:
	case TRAK:
	case MDIA:
		return new Box( boxtype );
	default:
		return NULL;
	}
}
//-----------------------------------------------------------------------
#define DEFAULT_MP4_BOX_BUFFER_SIZE				(1024*1024*5)

CMp4FileReader::CMp4FileReader( )
{
	m_buffer.Create(DEFAULT_MP4_BOX_BUFFER_SIZE);
}

CMp4FileReader::CMp4FileReader( const TCHAR* szFileName )
: hevc::io::CFileAccess( szFileName, io::CFileAccess::OpenRead, io::CFileAccess::FileOpenExisting, io::CFileAccess::ShareNone )
{
	m_buffer.Create(DEFAULT_MP4_BOX_BUFFER_SIZE);
}

util::CBitStreamReader*	CMp4FileReader::GetCurrentBox()
{
	u64	box_size;
	box_type_t box_type;
	if ( ! detect_box_size_and_type(this, box_size, box_type) )
		return 0;

	if ( 0 == box_size )
	{
		return NULL;
	}

	if ( box_type == MDAT )
	{
		// 'mdat'의 경우 헤더부분만 분석
		this->Read( m_buffer.GetBuffer(), (u32)8 );
		m_bs.SetPtr( m_buffer.GetBuffer(), box_size*8 );
		Seek((n64)box_size-8, io::CFileAccess::Seek_Cur);
	}
	else
	{
		if ( box_size >= m_buffer.GetLength() )
		{
			if ( m_buffer.Create(box_size) != box_size )
			{
				return NULL;
			}
		}

		if ( this->Read( m_buffer.GetBuffer(), (u32)box_size ) <= 0 )
			return NULL;

		m_bs.SetPtr( m_buffer.GetBuffer(), box_size * 8 );
	}

	return &m_bs;
}

//-----------------------------------------------------------------------
Box::Box( util::CBitStreamReader* pBs )
{
	m_pBsR = pBs;
	m_pBsW = 0;
	m_error = mp4_err_noerror;
	body_size = 0;
}

Box::Box( const char* szType )
{
	m_pBsR = 0;
	m_pBsW = 0;
	m_error = mp4_err_noerror;
	body_size = 0;
	type = box_type(szType);
}

Box::Box( box_type_t tp )
{
	m_pBsR = 0;
	m_pBsW = 0;
	m_error = mp4_err_noerror;
	body_size = 0;
	type = tp;
}

Box::~Box()
{
//	TRACE(_T("Box::~Box()\n"));
}

u64 Box::GetSize()
{
	if ( 1 >= size )
		return largesize;
	else
		return size;
}

void Box::SetSize(u64 sz)
{
	if ( size > _MAX_DWORD )
	{
		largesize = size;
		size = 1;
	}
	else
	{
		size = (u32)sz;
		largesize = 0;
	}
}

box_len_t	Box::GetSizeBits()
{
	if ( 1 >= size )
		return Box_len_64;
	else
		return Box_len_32;
}

u32 Box::GetType()
{
	if ( type == UUID ) {
		return 0xFFFFFFFF;
	}

	return type;
}

void 	Box::SetType( box_type_t box_type )
{
	type = box_type;
}

void Box::SetUserType( const u8* pUserTypeUUID )
{
	type = UUID;

	memset(user_type, 0, 17);
	memcpy(user_type, pUserTypeUUID, 16);
}

void	Box::AddChild( Box* pBox )
{
	m_childs.push_back(std::shared_ptr<Box>(pBox));
}

mp4_error_t Box::Decode()
{
	size_t remains = m_pBsR->GetRemainBytes();
	if ( remains < 8 ) return mp4::mp4_err_invalid_bitstream_size;

	size = (u32)m_pBsR->Get(32);
	type = (u32)m_pBsR->Get(32);
	remains -= 8;

	START_CHECK_BS_GET_SIZE(m_pBsR);

	// large size
	if ( 1 == size )
	{
		if ( remains < 8 )
			m_error = mp4_err_invalid_bitstream_size;
		else
			largesize = m_pBsR->Get(64);
	}
	// size to the eof
	else if ( 0 == size )
	{
		largesize = m_pBsR->GetRemainBytes() + 8;
	}
	else if ( 8 > size )
	{
		m_error = mp4_err_invalid_bitstream_size;
	}

	if ( UUID == type )
	{
		if ( m_pBsR->GetRemainBytes() < 16 )
			m_error = mp4_err_invalid_bitstream_size;
		else
		{
			memset( user_type, 0, 17 );
			memcpy( user_type, m_pBsR->GetCurrentPtr(), 16 );
			m_pBsR->Throw(16*8);
		}
	}

	if ( mp4_err_noerror == m_error )
		m_error = DecodeBody();

	body_size = CHECK_BS_GET_SIZE(m_pBsR);

	m_error = DecodeChild();
	
	return m_error;
}

mp4_error_t Box::Encode(  util::CBitStreamWriter* pBs )
{
	m_pBsW = pBs;

	size_t remains = m_pBsW->GetRemainBytes();
	if ( remains < 8 )
	{
		m_error = mp4::mp4_err_invalid_bitstream_size;
	}

	SetSize( GetEncodeSize() );

	if ( m_error == mp4_err_noerror )
	{
		if ( GetSizeBits() == Box_len_32 )
		{
			m_pBsW->Set(size, 32);
			m_pBsW->Set(type, 32);
		}
		else
		{
			m_pBsW->Set(1, 32);
			m_pBsW->Set(type, 32);
			m_pBsW->Set(largesize, 64);
		}

		if ( UUID == type )
		{
			m_pBsW->SetBytes(user_type, 16);
		}

		m_error = EncodeBody();
	}

	m_error = EncodeChild();

	return m_error;
}

mp4_error_t Box::DecodeChild()
{
	do {
		if ( body_size + GetChildSize() + 8 >= size )
			return mp4_err_noerror;
		if ( m_pBsR->GetRemainBytes() < 1 )
			break;

		Box* pChild = Box_FactoryEx::Create(m_pBsR);

		if ( !pChild )
		{
			// skip
			size = (u32)m_pBsR->Get(32);
			m_pBsR->Throw(size-4);
		}
		else
		{
			pChild->Decode();
			m_childs.push_back(std::shared_ptr<Box>(pChild));
		}

	} while (true);

	return mp4_err_noerror;
}

mp4_error_t Box::EncodeChild()
{
	for ( size_t ii = 0; ii < m_childs.size(); ii++ )
	{
		m_childs[ii]->Encode( m_pBsW );
	}

	return mp4_err_noerror;
}

u64		Box::GetEncodeSize()
{
	u64 enc_size = GetEncodeBodySize();
	body_size = enc_size;

	if ((body_size % 2))
	{
		int aa = 0;
	}

	for ( size_t ii = 0; ii < m_childs.size(); ii++ )
	{
		enc_size += m_childs[ii]->GetEncodeSize();
	}

	if ( enc_size > _MAX_DWORD )
		enc_size += 8;

	if ((body_size % 2))
	{
		int aa = 0;
	}

	if ( GetType() == UUID )
		enc_size += 16;

	return enc_size;
}

u64		Box::GetEncodeBodySize()
{
	return 8;
}

u64		Box::GetChildSize()
{
	u64 sz = 0;
	for (size_t ii =0; ii < m_childs.size(); ii++)
	{
		sz += m_childs[ii]->GetSize();
	}

	return sz;
}

//-----------------------------------------------------------------------
FullBox::FullBox(util::CBitStreamReader* pBs)
: Box( pBs )
{

}

FullBox::FullBox(const char* szType)
: Box( szType )
{

}


//-----------------------------------------------------------------------
mp4_error_t FullBox::DecodeBody()
{
	if ( m_pBsR->GetRemainBytes() < 4 )
		return mp4_err_invalid_bitstream_size;
	version = (u8)m_pBsR->Get(8);
	flags = (u32)m_pBsR->Get(24);

	return mp4_err_noerror;
}

mp4_error_t FullBox::EncodeBody()
{
	m_pBsW->Set(version, 8);
	m_pBsW->Set(flags, 24);

	return mp4_err_noerror;
}

//-----------------------------------------------------------------------
mp4_error_t FileTypeBox::DecodeBody( )
{
	if ( (m_pBsR->GetRemainBytes() % 4) != 0 || m_pBsR->GetRemainBytes() < 8 ) return mp4_err_invalid_bitstream_size;

	major_brand = (u32)m_pBsR->Get(32);
	minor_version = (u32)m_pBsR->Get(32);

	while ( m_pBsR->GetRemainBits() > 0 ) 
	{
		u32 brand = (u32)m_pBsR->Get(32);
		compatible_brands.push_back(brand);
	}

	return mp4_err_noerror;
}

mp4_error_t FileTypeBox::EncodeBody( void )
{
	if ( (m_pBsW->GetRemainBytes() % 4) != 0 ||
		  m_pBsW->GetRemainBytes() < 8 + (compatible_brands.size() * 4) )
		  return mp4_err_invalid_bitstream_size;

	m_pBsW->Set(major_brand, 32);
	m_pBsW->Set(minor_version, 32);

	for ( size_t ii = 0; ii < compatible_brands.size(); ii++ )
		m_pBsW->Set(compatible_brands[ii], 32);

	return mp4_err_noerror;
}

//-----------------------------------------------------------------------
mp4_error_t MovieHeaderBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	if ( GetVersion() == 1 )
	{
		if ( m_pBsR->GetRemainBytes() != 20 + 80 )
			return mp4_err_invalid_bitstream_size;

		creation_time = m_pBsR->Get(64);
		modification_time = m_pBsR->Get(64);
		timescale = (u32)m_pBsR->Get(32);
		duration = m_pBsR->Get(64);
	}
	else if ( GetVersion() == 0 )
	{
		if ( m_pBsR->GetRemainBytes() != 16 + 80 )
			return mp4_err_invalid_bitstream_size;

		creation_time = m_pBsR->Get(32);
		modification_time = m_pBsR->Get(32);
		timescale = (u32)m_pBsR->Get(32);
		duration = m_pBsR->Get(32);
	}

	rate = (n32)m_pBsR->Get(32);
	volume = (n16)m_pBsR->Get(16);
	reserved1 = (u16)m_pBsR->Get(16);
	for (int ii = 0; ii < 2; ii++)
	{
		reserved2[ii] = (u32)m_pBsR->Get(32);
	}
	for (int ii = 0; ii < 9; ii++)
	{
		matrix[ii] = (u32)m_pBsR->Get(32);
	}
	for (int ii = 0; ii < 6; ii++)
	{
		pre_defined[ii] = (u32)m_pBsR->Get(32);
	}
	next_track_ID = (u32)m_pBsR->Get(32);

	return mp4_err_noerror;
}

u64	MovieHeaderBox::GetEncodeBodySize()
{
	u64 bdsz = __super::GetEncodeBodySize();

	if ( GetVersion() == 1 )
		bdsz += 28;
	else 
		bdsz += 16;

	bdsz += 80;

	return GetVersion() == 1 ? bdsz + 108 : 96;
}

//-----------------------------------------------------------------------
mp4_error_t TrackHeaderBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	if ( GetVersion() == 1 )
	{
		if ( m_pBsR->GetRemainBytes() != 20 )
			return mp4_err_invalid_bitstream_size;

		creation_time		= m_pBsR->Get(64);
		modification_time	= m_pBsR->Get(64);
		track_ID			= (u32)m_pBsR->Get(32);
		reserved1			= (u32)m_pBsR->Get(32);
		duration			= (u32)m_pBsR->Get(64);
	}
	else if ( GetVersion() == 0 )
	{
		if ( m_pBsR->GetRemainBytes() != 12 )
			return mp4_err_invalid_bitstream_size;

		creation_time		= (u32)m_pBsR->Get(32);
		modification_time	= (u32)m_pBsR->Get(32);
		track_ID			= (u32)m_pBsR->Get(32);
		reserved1			= (u32)m_pBsR->Get(32);
		duration			= (u32)m_pBsR->Get(32);
	}

	for (int ii = 0; ii < 2; ii++ )
	{
		reserved2[ii] = (u32)m_pBsR->Get(32);
	}
	layer = (n16)m_pBsR->Get(16);
	alternate_group = (n16)m_pBsR->Get(16);
	volume = (n16)m_pBsR->Get(16);
	reserved3 = (u16)m_pBsR->Get(16);
	for (int ii = 0; ii < 9; ii++)
	{
		matrix[ii] = (u32)m_pBsR->Get(32);
	}
	width = (u32)m_pBsR->Get(32);
	height = (u32)m_pBsR->Get(32);

	return mp4_err_noerror;
}

u64		TrackHeaderBox::GetEncodeBodySize()
{
	u64 bdsz = __super::GetEncodeBodySize();

	return GetVersion() == 1 ? bdsz + 92 : bdsz + 80;
}

//-----------------------------------------------------------------------
mp4_error_t EditListBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	if ( GetVersion() == 1 )
	{
		if ( m_pBsR->GetRemainBytes() != 20 )
			return mp4_err_invalid_bitstream_size;

		segment_duration = m_pBsR->Get(64);
		media_time = (n64)m_pBsR->Get(64);
	}
	else if ( GetVersion() == 0 )
	{
		if ( m_pBsR->GetRemainBytes() != 12 )
			return mp4_err_invalid_bitstream_size;

		segment_duration = m_pBsR->Get(32);
		media_time = (n64)m_pBsR->Get(32);
	}

	media_rate_integer = (n16)m_pBsR->Get(16);
	media_rate_fraction = (n16)m_pBsR->Get(16);

	return mp4_err_noerror;
}

u64			EditListBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + GetVersion() == 0 ? 20 : 12;
}

//-----------------------------------------------------------------------
mp4_error_t MediaHeaderBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	if ( GetVersion() == 1 )
	{
		if ( m_pBsR->GetRemainBytes() != 20 + 4 )
			return mp4_err_invalid_bitstream_size;

		creation_time = m_pBsR->Get(64);
		modification_time = m_pBsR->Get(64);
		timescale = (u32)m_pBsR->Get(32);
		duration = m_pBsR->Get(64);
	}
	else if ( GetVersion() == 0 )
	{
		if ( m_pBsR->GetRemainBytes() != 16 + 4 )
			return mp4_err_invalid_bitstream_size;

		creation_time = m_pBsR->Get(32);
		modification_time = m_pBsR->Get(32);
		timescale = (u32)m_pBsR->Get(32);
		duration = m_pBsR->Get(32);
	}

	m_pBsR->Get(1);
	for (int ii = 0; ii < 3; ii++)
	{
		language[ii] = (u8)m_pBsR->Get(5);
	}

	pre_defined = (u16)m_pBsR->Get(16);

	return mp4_err_noerror;
}

u64	MediaHeaderBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + GetVersion() == 0 ? 32 : 20;
}

//-----------------------------------------------------------------------
void HandlerBox::SetName(__stdstr nm)
{
	this->name = nm;
}

mp4_error_t HandlerBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	if ( m_pBsR->GetRemainBytes() < 20 )
		return mp4_err_invalid_bitstream_size;

	pre_defined = (u32)m_pBsR->Get(32);
	handler_type = (u32)m_pBsR->Get(32);
	for (int ii = 0; ii < 3; ii++)
	{
		reserved[ii] = (u32)m_pBsR->Get(32);
	}
	name = GetUtf8StringFromBs(m_pBsR);

	return mp4_err_noerror;
}

mp4_error_t HandlerBox::EncodeBody( void )
{
	mp4_error_t err = __super::EncodeBody();
	if ( err != mp4_err_noerror )
		return err;

	util::CCharHelper strName(name.c_str());

	if ( m_pBsW->GetRemainBytes() < 20 + strName.GetByteLen8() )
		return mp4_err_invalid_bitstream_size;

	m_pBsW->Set( pre_defined, 32 );
	m_pBsW->Set( handler_type , 32 );
	m_pBsW->Set( 0, 32 );
	m_pBsW->Set( 0, 32 );
	m_pBsW->Set( 0, 32 );

	m_pBsW->SetBytes(strName.GetUTF8Ptr(), strName.GetByteLen8());

	return mp4_err_noerror;
}

u64	HandlerBox::GetEncodeBodySize()
{
	util::CCharHelper str(name.c_str());
	return __super::GetEncodeBodySize() + str.GetByteLen8() + 20;
}

//-----------------------------------------------------------------------
mp4_error_t VideoMediaHeaderBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	if ( m_pBsR->GetRemainBytes() != 8 ) 
		return mp4_err_invalid_bitstream_size;

	graphicsmode = (u16)m_pBsR->Get(16);
	for (int ii = 0; ii < 3; ii++)
		opcolor[ii] = (u16)m_pBsR->Get(16);

	return mp4_err_noerror;
}

u64	VideoMediaHeaderBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + 8;
}

//-----------------------------------------------------------------------
mp4_error_t DataEntryUrlBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	location = GetUtf8StringFromBs(m_pBsR);

	return mp4_err_noerror;
}

u64	DataEntryUrlBox::GetEncodeBodySize()
{
	util::CCharHelper str(location.c_str());
	return __super::GetEncodeBodySize() + str.GetByteLen8();
}

//-----------------------------------------------------------------------
mp4_error_t DataEntryUrnBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	name = GetUtf8StringFromBs( m_pBsR );
	location = GetUtf8StringFromBs( m_pBsR );

	return mp4_err_noerror;
}

u64	DataEntryUrnBox::GetEncodeBodySize()
{
	util::CCharHelper str1(name.c_str());
	util::CCharHelper str2(location.c_str());
	return __super::GetEncodeBodySize() + str1.GetByteLen8() + str2.GetByteLen8();
}

//-----------------------------------------------------------------------
mp4_error_t DataReferenceBox::DecodeBody( void )
{
	if ( FullBox::DecodeBody() != mp4_err_noerror )
		return GetLastError();

	if ( m_pBsR->GetRemainBytes() != 4 ) 
		return mp4_err_invalid_bitstream_size;

	entry_count = (u32)m_pBsR->Get(32);

	return mp4_err_noerror;
}

u64	DataReferenceBox::GetEncodeBodySize()
{
	return __super::GetEncodeBodySize() + 4;
}

//-----------------------------------------------------------------------
mp4_error_t SampleEntry::DecodeBody( void )
{
	if ( m_pBsR->GetTotalBytes() != 8 )
		return mp4_err_invalid_bitstream_size;

	m_pBsR->Throw(8*6); // reserved
	data_reference_index = (u16)m_pBsR->Get(16);

	return mp4_err_noerror;
}

} // namespace mp4
} // namespace hevc


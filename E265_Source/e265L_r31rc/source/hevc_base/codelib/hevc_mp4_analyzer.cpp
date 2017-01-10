#include "./hevc_mp4_analyzer.h"

namespace hevc {
namespace mp4
{

//-----------------------------------------------------------------------
CMp4Parser::CMp4Parser()
{

}

CMp4Parser::~CMp4Parser()
{

}

void	CMp4Parser::Clear()
{
	m_pFtyp = 0;

	m_lstBox.clear();
}

Box*		CMp4Parser::Parse( util::CBitStreamReader* pBs )
{
	Box* pBox = GenerateBox( pBs );

	return pBox;
}

bool CMp4Parser::GetBoxInfo( util::CBitStreamReader* pBs, mp4_box_offset_info* pBoxInfo )
{
	size_t remains = pBs->GetRemainBytes();
	if ( remains < 8 ) return false;

// 	pBoxInfo->size = (u32)pBs->Get(32);
// 	pBoxInfo->type = (u32)pBs->Get(32);	

	return true;
}

Box*	CMp4Parser::GenerateBox( util::CBitStreamReader* pBs )
{
	return Box_Factory::Create( pBs );
/*
	u64 data = pBs->Get(64);
	u32 size = u32(data >> 32);
	u32 type = u32(data);
	switch ( type )
	{
	case FTYP:
		m_pFtyp = new FileTypeBox(pBs);
		m_lstBox.push_back( std::shared_ptr<Box>(m_pFtyp) );
		return m_pFtyp;
	case MOOV:
	case IODS:
	case MVHD:
	case TRAK:
	case TKHD:
	case MDIA:
	case MDHD:
	case HDLR:
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
	case BTRT:
	default:
		return NULL;
	}*/
}

//-----------------------------------------------------------------------
CMp4Analyzer::CMp4Analyzer()
: m_file(0)
, m_pullFileSize(0)
, m_pullCurrent(0)
{

}

CMp4Analyzer::~CMp4Analyzer()
{

}

n32 CMp4Analyzer::Init( LPCTSTR szFileName, u64* pSize, u64* pCurrent )
{
	Clear();
	m_file.SetFileName( szFileName );
	if ( m_file.Open(io::CFileAccess::OpenRead, io::CFileAccess::FileOpenExisting, io::CFileAccess::ShareRead) 
		!= hevc::NoError )
	{
		return mp4_err_fileopen;
	}

	m_pullFileSize = pSize;
	m_pullCurrent = pCurrent;

	return mp4_err_noerror;
}

void CMp4Analyzer::SetNotifyHwnd(HWND hWnd)
{
	m_hNotifyHwnd = hWnd;
}

void CMp4Analyzer::Clear()
{
	m_file.Close();
	m_parser.Clear();
	m_boxes.clear();
}

size_t	CMp4Analyzer::GetBoxCount()
{
	return m_boxes.size();
}

const mp4_box_offset_info* CMp4Analyzer::GetBoxInfo( size_t idx )
{
	if ( idx >= GetBoxCount() ) return 0;

	return &m_boxes[idx];
}

static void Box2BoxInfo( Box* pBox, mp4_box_offset_info* pBoxInfo, u64 ullCurrent )
{
	pBoxInfo->offset = ullCurrent;
	pBoxInfo->size = pBox->GetSize();
	pBoxInfo->type = pBox->GetType();
	ullCurrent += pBox->GetBodySize();
	for (size_t ii = 0; ii < pBox->GetChildCount(); ii++)
	{
		mp4_box_offset_info childBoxInfo;
		Box2BoxInfo(pBox->GetChild(ii), &childBoxInfo, ullCurrent);
	}
}

DWORD	CMp4Analyzer::_Run()
{
	util::CBitStreamReader* pBs;

	m_file.Seek(0,  hevc::io::CFileAccess::Seek_Begin);

	*m_pullFileSize = m_file.GetFileSize();
	*m_pullCurrent = 0;

	Box* pBox;
	u64 sz;
	
	while ( !m_bStop )
	{
		mp4_box_offset_info box_info;

		pBs = m_file.GetCurrentBox();
		
		if ( ! pBs )
			break;

		sz = pBs->GetRemainBytes();

		pBox = m_parser.Parse( pBs );

		if ( pBox && pBox->Decode() == mp4_err_noerror )
		{
			box_info.offset = *m_pullCurrent;
			box_info.size = pBox->GetSize();
			box_info.type = pBox->GetType();
			// get child box
			//box_info.pBox.reset(pBox);
			m_boxes.push_back(box_info);

// 			TRACE(_T("BOX : %s, size : %u, offset : %u\n"), GetBoxTypeText((box_type_t)pBox->GetType())
// 				, pBox->GetSize(), box_info.offset);
		}

		*m_pullCurrent += sz;
		//m_parser.Parse(pBs);
	}

	m_boxes.clear();

//	::SendMessage(m_hNotifyHwnd, WM_ANALYZE_FINISHED, 0, (LPARAM)this);
	return 0;
}

} // namespace mp4
} // namespace hevc

#include "./hevc_mp4.h"
#include <hevc_template.h>
#include <hevc_thread.h>

namespace hevc {
namespace mp4 {

//-----------------------------------------------------------------------
typedef struct tagmp4boxinfo{
	tagmp4boxinfo(){}
	u64		offset;
	u64		size;
	u32		type;
	std::vector<tagmp4boxinfo> childs;
} mp4_box_offset_info;

//-----------------------------------------------------------------------
class CMp4Parser
{
public:
	CMp4Parser();
	virtual ~CMp4Parser();

	void	Clear();

	Box*	Parse( util::CBitStreamReader* pBs );
	bool	GetBoxInfo( util::CBitStreamReader* pBs, mp4_box_offset_info* pBoxInfo );

protected:
	Box*	GenerateBox( util::CBitStreamReader* pBs );
	
	FileTypeBox			*m_pFtyp;
	Box_Factory			m_factory;

	std::list<std::shared_ptr<Box>>		m_lstBox;
};

//-----------------------------------------------------------------------
class CMp4Analyzer : public workflow::CThread
{
public:
	CMp4Analyzer();
	virtual ~CMp4Analyzer();

	n32 Init( LPCTSTR szFileName, u64* pSize, u64* pCurrent );
	void SetNotifyHwnd( HWND hWnd );

	void Clear();

	size_t	GetBoxCount();
	const mp4_box_offset_info* GetBoxInfo( size_t idx );

protected:
	CMp4FileReader	m_file;
	CMp4Parser		m_parser;
	HWND			m_hNotifyHwnd;

	virtual		DWORD	_Run();

	u64		*m_pullFileSize;
	u64		*m_pullCurrent;

	std::vector<mp4_box_offset_info>	m_boxes;
};

/*
static inline CMp4Analyzer* GetAnalyzer()
{
	return CMp4Analyzer::GetInstance();
}
*/

} // namespace mp4
} // namespace hevc

#include "prop_util.h"
#include "./hevc_enc_util.h"

#include <fstream>
#include <istream>
#include <ostream>
#include <algorithm>
using namespace std;
#define delimiter			':'
#define comment_mark		'#'
static inline bool is_valid_pnv( string s )
{
	if (ltrim(s).size() < 1 ) return false;
	if (s[0] == comment_mark) return false;
	if (s.find_first_of(delimiter) < 0) return false;
	return true;
}

static inline string get_prop( string s )
{
	if (!is_valid_pnv(s)) return "";
	return trim( s.substr( 0, s.find_first_of(delimiter) ) );
}

static inline string get_value( string s )
{
	if (!is_valid_pnv(s)) return "";
	return ltrim( s.substr(s.find_first_of(delimiter) + 1, s.size() - s.find_first_of(delimiter)) );
}

bool	CPropUtil::Load( std::string strFileName )
{
	m_props.clear();

	fstream fs;
	fs.open( strFileName.c_str(), std::ios_base::in );

	if ( fs.bad() || fs.fail() ) return false;

	istream& is = fs;

	char szBuffer[2048];

	__stdstr strProp;
	__stdstr strVal;
	__stdstr strTemp;

	while ( ! is.eof() )
	{
		is.getline(szBuffer, 2048);

		strTemp = szBuffer;

		strProp = get_prop(strTemp);
		strVal = get_value(strTemp);

		if ( !strProp.empty() )
		{
			m_props[strProp] = strVal;
		}
	}

	return true;
}

void	CPropUtil::Clear()
{
	m_props.clear();
}

bool	CPropUtil::GetLong( std::string strProp, n32 & nValue )
{
	if ( m_props.count(strProp) < 1 ) return false;

	nValue = atol( m_props[strProp].c_str() );

	return true;
}

bool	CPropUtil::GetString( std::string strProp, std::string &strRet )
{
	if ( m_props.count(strProp) < 1 ) return false;

	strRet = m_props[strProp];

	return true;
}

bool	CPropUtil::GetReal( std::string strProp, r64 & rValue )
{
	if ( m_props.count(strProp) < 1 ) return false;

	rValue = atof( m_props[strProp].c_str() );

	return true;
}

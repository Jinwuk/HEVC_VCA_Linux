/**
	----------------------------------------------------------------------------------
	@file		prop_util.h
	@date		2013/11/19
	@author		wsseo
	@brief		
	@endcode
	----------------------------------------------------------------------------------
	@section MODIFYINFO 수정내용
	@endcode
*/

#ifndef _PROP_UTIL_H_INCLUDED_
#define _PROP_UTIL_H_INCLUDED_

#include <hevc_base.h>
#include <map>
using namespace hevc;

class CPropUtil {
public :
	bool	Load( std::string strFileName );
	void	Clear();
	bool	GetLong( std::string strProp, n32 & nValue );
	bool	GetString( std::string strProp, std::string &strRet );
	bool	GetReal( std::string strProp, r64 & rValue );
protected:
	typedef std::map<__stdstr, __stdstr>		t_prop_key_n_val;
	typedef t_prop_key_n_val::iterator			itrProp;

	t_prop_key_n_val	m_props;
};

#endif // define _PROP_UTIL_H_INCLUDED_
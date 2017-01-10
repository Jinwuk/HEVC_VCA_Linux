#ifndef _HEVC_PROCESS_GROUP_H_INCLUDED_
#define _HEVC_PROCESS_GROUP_H_INCLUDED_

#include <windows.h>
#include <memory>
#include <process.h>

template <class T>
class CSingleTon2
{
private:
	static std::auto_ptr<T>	g_pInstance;
public:
	static T*	GetInstance(void)
	{
		if (NULL == g_pInstance.get())
		{
			std::auto_ptr<T> pTemp (new T);
			g_pInstance = pTemp;
		}
		return g_pInstance.get();
	}

	static void ReleaseInstance()
	{
		if ( GetInstance() != NULL )
		{
			delete g_pInstance.get();
			g_pInstance.release();
		}
	}
};

template <class T> std::auto_ptr<T>	CSingleTon2<T>::g_pInstance;

#define FRIEND_FOR_SINGLETON_CDTOR(_myclass_)	friend class CSingleTon2<_myclass_>; \
	friend class std::auto_ptr<_myclass_>;

class CProcessGroupMgr : public CSingleTon2<CProcessGroupMgr>
{
	FRIEND_FOR_SINGLETON_CDTOR(CProcessGroupMgr);
public:
	bool					m_bUseProcessGroup;
	unsigned short			m_nProcessGroup;
	unsigned long long		m_nAffinityMask;
};

inline CProcessGroupMgr * GetProcGrpMgr()
{
	return CProcessGroupMgr::GetInstance();
}

inline bool _SetThreadAffinity(void* hThread)
{
	if ( NULL == hThread || INVALID_HANDLE_VALUE == hThread )
		return false;

	GROUP_AFFINITY ga;
	ZeroMemory(&ga, sizeof(ga));
	ga.Group = GetProcGrpMgr()->m_nProcessGroup;
	ga.Mask = GetProcGrpMgr()->m_nAffinityMask;

	if ( ! SetThreadGroupAffinity( hThread, &ga, 0 ) )
		return false;
	return true;
}

#endif // _HEVC_PROCESS_GROUP_H_INCLUDED_
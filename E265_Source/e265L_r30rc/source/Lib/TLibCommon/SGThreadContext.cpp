#include <stdio.h>
#include "SGThreadContext.h"

#if SG_THREAD_POOLING
typedef struct {
	bool		bUseProcessGroup;
	USHORT		usProcessGroup;
	ULONGLONG	ullAffinityMask;
} sg_process_option_t;

static sg_process_option_t g_sProcOption = { false, 0, 0 };

void		SG_SetProcessOption( bool bUseProcGroup, USHORT usProcessGroup, ULONGLONG ullAffinityMask )
{
	g_sProcOption.bUseProcessGroup = bUseProcGroup;
	g_sProcOption.usProcessGroup = usProcessGroup;
	g_sProcOption.ullAffinityMask = ullAffinityMask;
}

bool		SG_UsingProcGroup()
{
	return g_sProcOption.bUseProcessGroup;
}

USHORT		SG_GetProcessGroup()
{
	return g_sProcOption.usProcessGroup;
}

ULONGLONG	SG_GetAffinityMask()
{
	return g_sProcOption.ullAffinityMask;
}

void		SG_SetThreadAffinity(HANDLE hThread)
{
	if ( ! SG_UsingProcGroup() ) return;

	GROUP_AFFINITY ga;
	memset(&ga, 0, sizeof(ga));
	ga.Group = SG_GetProcessGroup();
	ga.Mask = SG_GetAffinityMask();
	if ( ! SetThreadGroupAffinity( hThread, &ga, NULL ) )
	{
	}
}

#endif // SG_THREAD_POOLING
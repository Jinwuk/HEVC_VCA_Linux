#include "./hevc_iocp.h"

#include <hevc_charHelper.h>
#include <hevc_log.h>
#include <hevc_file.h>

namespace hevc {
namespace net {

CIocpWorker::CIocpWorker( HANDLE hComplPort )
: m_hComplPort( hComplPort )
{

}

CIocpWorker::~CIocpWorker()
{
}

DWORD CIocpWorker::_Run()
{
	DWORD cbTransferred;
	SOCKET client_sock;
	SOCKETINFO *ptr;

	int retval;

	while ( !m_bStop )
	{
		retval = GetQueuedCompletionStatus(m_hComplPort, &cbTransferred,
			(PULONG_PTR)&client_sock, (LPOVERLAPPED *)&ptr, INFINITE);

		if ( client_sock == INVALID_SOCKET || 0 == ptr )
		{
			break;
		}
		CIocpSocket* pSocket = ptr->pAcceptSocket;

		if ( Iocp_output == ptr->eIoType )
		{
			ptr->nSendBytes = cbTransferred;
			pSocket->AddSended( cbTransferred );
			pSocket->OnSend();
		}
		else if ( Iocp_input == ptr->eIoType ) 
		{
			// check overlapped I/O result
			if ( cbTransferred == 0 )
			{
				// peer closed
				if(retval == 0)
				{
					DWORD dwTransffered, dwFlags;
					if ( ! WSAGetOverlappedResult(pSocket->GetSafeHandle(), ptr, &dwTransffered, FALSE, &dwFlags) )
					{
						// pSocket->OnClose();
						// OnClose() 는 OnReceive() 함수에서 호출되는 Recv() 함수에서 
						// WSARecv() 실패시 호출.
					}
				}

				log::Debug(_T("peer closed. [%d], total recv : %u, elapsed : %u ms, recv rate : %0.3f"),
					pSocket->GetPort(), pSocket->GetTotalReceived(), pSocket->GetElapsed(), pSocket->GetRecvBps() );
				continue;
			}
			else
			{
				pSocket->SetReceiveByte( cbTransferred );
				pSocket->OnReceive();
			}
		}
	}
	return 0;
}

CIocpSocket::CIocpSocket()
: m_sck(INVALID_SOCKET)
, m_hComplPort(NULL)
, m_ullRecv(0)
, m_ullSend(0)
{
}

CIocpSocket::~CIocpSocket()
{

}

SOCKET	CIocpSocket::GetSafeHandle()
{
	return m_sck;
}

void	CIocpSocket::Close()
{
	util::SYNC(m_sync);
	if ( INVALID_SOCKET != m_sck )
	{
		closesocket(m_sck);
		m_sck = INVALID_SOCKET;
		delete m_pSockInfo;
		m_pSockInfo = 0;
		m_ullRecv = 0;
		m_ullSend = 0;
	}
}

BOOL	CIocpSocket::Accept(SOCKET hSocket, SOCKADDR_IN* pAddr, HANDLE hComplPort)
{
	util::SYNC(m_sync);

	m_sck			= hSocket;
	m_port			= pAddr->sin_port;
	m_addr			= pAddr->sin_addr.s_addr;
	m_hComplPort	= hComplPort;

	HANDLE hResult = CreateIoCompletionPort(
						(HANDLE)m_sck, 
						m_hComplPort, 
						(DWORD)m_sck, 0);

	if(hResult == NULL) return FALSE;

	m_pSockInfo = new SOCKETINFO;

	Recv();

	return TRUE;
}

BOOL	CIocpSocket::Connect(const SOCKADDR* lpSockAddr, int nSockAddrLen, HANDLE hComplPort)
{
	if ( INVALID_SOCKET != m_sck || NULL != m_hComplPort )
		return FALSE; 

	m_hComplPort = hComplPort;

	if ( INVALID_SOCKET != m_sck )
	{
		return FALSE;
	}

	m_sck = WSASocket(AF_INET, SOCK_STREAM, IPPROTO_IP, NULL, 0, WSA_FLAG_OVERLAPPED);

	if ( INVALID_SOCKET == m_sck )
	{
		return FALSE;
	}
	
	if ( WSAConnect( m_sck, lpSockAddr, nSockAddrLen, 0, 0, 0, 0) == SOCKET_ERROR )
	{
		return FALSE;
	}

	HANDLE hResult = CreateIoCompletionPort(
		(HANDLE)m_sck, 
		m_hComplPort, 
		(DWORD)m_sck, 0);

	m_pSockInfo = new SOCKETINFO;

	Recv();

	return TRUE;
}

BOOL	CIocpSocket::Connect(LPCTSTR lpszHostAddress, UINT nHostPort, HANDLE hComplPort)
{
	util::CCharHelper str(lpszHostAddress);
	SOCKADDR_IN k_Addr;
	k_Addr.sin_family      = AF_INET;
	k_Addr.sin_addr.s_addr = inet_addr((const char*)str);
	k_Addr.sin_port        = htons(nHostPort);

	return Connect((SOCKADDR*)&k_Addr, sizeof(SOCKADDR_IN), hComplPort);
}

BOOL	CIocpSocket::Recv()
{
	ZeroMemory(m_pSockInfo, sizeof(OVERLAPPED));
	m_pSockInfo->nRecvBytes		= 0;
	m_pSockInfo->nSendBytes		= 0;
	m_pSockInfo->wsaBuf.buf		= m_pBuf;
	m_pSockInfo->wsaBuf.len		= DEF_BUF_SIZE;
	m_pSockInfo->pAcceptSocket	= this;
	m_pSockInfo->eIoType		= Iocp_input;

	DWORD dwRecvBytes;
	DWORD flags = 0;
	int retval = WSARecv(m_sck, 
		&(m_pSockInfo->wsaBuf), 
		1, 
		&dwRecvBytes, 
		&flags, 
		m_pSockInfo, 
		NULL);

	if( retval == SOCKET_ERROR )
	{
		if(WSAGetLastError() != ERROR_IO_PENDING)
		{
			log::LogWin32Error(WSAGetLastError(), _T("Recv() error"));
			Close();

			OnClose();
			return FALSE;
		}
	}

	return TRUE;
}

BOOL	CIocpSocket::RecvIndirect(void* pBuf, DWORD dwBufLen)
{
	ZeroMemory(m_pSockInfo, sizeof(OVERLAPPED));
	m_pSockInfo->nRecvBytes		= 0;
	m_pSockInfo->nSendBytes		= 0;
	m_pSockInfo->wsaBuf.buf		= (CHAR*)pBuf;
	m_pSockInfo->wsaBuf.len		= dwBufLen;
	m_pSockInfo->pAcceptSocket	= this;
	m_pSockInfo->eIoType		= Iocp_input;

	DWORD dwRecvBytes;
	DWORD flags = 0;
	int retval = WSARecv(m_sck, 
		&(m_pSockInfo->wsaBuf), 
		1, 
		&dwRecvBytes, 
		&flags, 
		m_pSockInfo, 
		NULL);

	if( retval == SOCKET_ERROR )
	{
		if(WSAGetLastError() != ERROR_IO_PENDING)
		{
			log::LogWin32Error(WSAGetLastError(), _T("Recv() error"));
			Close();

			OnClose();
			return FALSE;
		}
	}

	return TRUE;
}

BOOL	CIocpSocket::Send(const void* pData, LPDWORD INOUT pdwData)
{
	ZeroMemory(m_pSockInfo, sizeof(OVERLAPPED));

	if ( *pdwData > DEF_BUF_SIZE )
		return FALSE;

	memcpy_s(m_pBuf, DEF_BUF_SIZE, pData, *pdwData);

	m_pSockInfo->nRecvBytes		= 0;
	m_pSockInfo->nSendBytes		= 0;
	m_pSockInfo->wsaBuf.buf		= m_pBuf;
	m_pSockInfo->wsaBuf.len		= *pdwData;
	m_pSockInfo->pAcceptSocket	= this;
	m_pSockInfo->eIoType		= Iocp_output;

	DWORD flags = 0;
	int retval = WSASend(m_sck, &(m_pSockInfo->wsaBuf), 1, pdwData, flags, m_pSockInfo, NULL);

	if( retval == SOCKET_ERROR )
	{
		if(WSAGetLastError() != ERROR_IO_PENDING)
		{
			return FALSE;
		}
	}
	return TRUE;
}

BOOL	CIocpSocket::SendIndirect(const void* pData, LPDWORD INOUT pdwData)
{
	ZeroMemory(m_pSockInfo, sizeof(OVERLAPPED));
	m_pSockInfo->nRecvBytes		= 0;
	m_pSockInfo->nSendBytes		= 0;
	m_pSockInfo->wsaBuf.buf		= (char*)pData;
	m_pSockInfo->wsaBuf.len		= *pdwData;
	m_pSockInfo->pAcceptSocket	= this;
	m_pSockInfo->eIoType		= Iocp_output;

	DWORD flags = 0;
	int retval = WSASend(m_sck, &(m_pSockInfo->wsaBuf), 1, pdwData, flags, m_pSockInfo, NULL);

	if( retval == SOCKET_ERROR )
	{
		if(WSAGetLastError() != ERROR_IO_PENDING)
		{
			return FALSE;
		}
	}
	return TRUE;
}

void	CIocpSocket::OnReceive()
{
	if ( m_pSockInfo )
	{
//		log::Debug(_T("%d bytes received"), m_pSockInfo->nRecvBytes);
	}
	Recv();
}

void	CIocpSocket::OnSend()
{
	Recv();
}

void	CIocpSocket::OnClose()
{
}

void	CIocpSocket::SetReceiveByte( u32 nReceive )
{
	if ( m_pSockInfo )
	{
		if ( m_ullRecv == 0 )
		{
			log::Debug( _T("first received") );
			m_nTickFirstReceived = GetTickCount();
		}
		m_pSockInfo->nRecvBytes = nReceive;
		m_ullRecv += nReceive;
	}
}

double	CIocpSocket::GetRecvBps()
{
	m_nElapsed = GetTickCount() - m_nTickFirstReceived; // total ms
	double rate = ((double)m_ullRecv*8.0) / ((double)m_nElapsed*1000.0);

	return rate;
}

CIocp::CIocp()
: m_hComplPort(NULL)
, m_nWorkerCnt(4)
{

}

CIocp::~CIocp()
{

}

bool CIocp::Open()
{
	Close();

	m_hComplPort = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 0);
	if(m_hComplPort == NULL) return false;

	CreateWorkers();

	return true;
}

void CIocp::Close()
{
	if ( m_hComplPort )
	{
		CloseHandle(m_hComplPort);
		m_hComplPort = 0;
	}

	StopWorkers();
}

void	CIocp::CreateWorkers()
{
	for ( u32 ii = 0; ii < m_nWorkerCnt; ii++ )
	{
		CIocpWorker* pSocket = new CIocpWorker(m_hComplPort);

		m_lstWorker.push_back(std::shared_ptr<CIocpWorker>( pSocket ));

		pSocket->_Start();
	}
}

void	CIocp::StopWorkers()
{
	for ( auto itr = m_lstWorker.begin(); itr != m_lstWorker.end(); itr++ )
	{
		PostQueuedCompletionStatus(m_hComplPort, 0, INVALID_SOCKET, 0);
	}

	for ( auto itr = m_lstWorker.begin(); itr != m_lstWorker.end(); itr++ )
	{
		//		(*itr)->_StopWait();
		WaitForSingleObject((*itr)->GetSafeHandle(), INFINITE);
	}

	m_lstWorker.clear();
}

CListenSocket::CListenSocket()
: m_sck(INVALID_SOCKET)
, m_port(0)
, m_addr(0)
{
	
}

CListenSocket::~CListenSocket()
{

}

void	CListenSocket::SetOption(u16 nPort, u32 nAddr)
{
	m_port = nPort;
	m_addr = nAddr;
}

u16		CListenSocket::GetPort()
{
	return m_port;
}

u32		CListenSocket::GetAddress()
{
	return m_addr;
}

void	CListenSocket::SetWorkerCount(u32 nCount)
{
	if ( IsThreadRunning() )
	{
		return;
	}

	m_nWorkerCnt = nCount;
}

bool	CListenSocket::PreStart()
{
	m_sck = socket(AF_INET, SOCK_STREAM, 0);
	if(m_sck == INVALID_SOCKET) false;

	SOCKADDR_IN serveraddr;
	ZeroMemory(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(m_port);
	serveraddr.sin_addr.s_addr = htonl(m_addr);
	int retval = bind(m_sck, (SOCKADDR *)&serveraddr, sizeof(serveraddr));

	if(retval == SOCKET_ERROR) return false; //err_quit("bind()");

	return CIocp::Open();
}

void	CListenSocket::PreStop()
{
	closesocket(m_sck);
	CIocp::Close();
	m_hComplPort = NULL;
	StopWorkers();
}

CIocpSocket* CListenSocket::GenerateSocket()
{
	return new CIocpSocket();
}

DWORD	CListenSocket::_Run()
{
	if ( !m_bPrepareStart || INVALID_SOCKET == m_sck )
		return 1;

	int retval = listen(m_sck, SOMAXCONN);
	if (retval == SOCKET_ERROR) return 1;

	SOCKADDR_IN clientaddr;
	int addrlen;
	SOCKET client_sock;
	while(1)
	{
		addrlen = sizeof(clientaddr);

		client_sock = WSAAccept(m_sck, (SOCKADDR *)&clientaddr, &addrlen, 0, 0);
		if( INVALID_SOCKET == client_sock )
		{
			DWORD dwErr = GetLastError();
			//err_display("accept()");
			break;
		}
		log::Debug( "client connected from=%s:%d.", 
			inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port) );

		CIocpSocket* pAccept = GenerateSocket();

		if ( NULL == pAccept )
		{
			closesocket(client_sock);
		}

		if ( ! pAccept->Accept( client_sock, &clientaddr, m_hComplPort ) )
		{
			delete pAccept;
		}
		else
		{
			m_lstSockets.push_back( std::shared_ptr<CIocpSocket>(pAccept) );
		}

	}

	return 0;
}

} // namespace net
} // namespace hevc

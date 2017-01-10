#pragma once

#pragma warning(disable:4005) // _WINSOCKAPI_ 재정의 warning 방지

#include <hevc_base.h>
#include <hevc_thread.h>
#include <hevc_sync.h>
#include <winsock2.h>
#include <map>
#include <memory>
#pragma comment(lib, "ws2_32")


namespace hevc {
namespace net {

#define DEF_BUF_SIZE			8192

class CIocpSocket;

typedef enum {
	Iocp_input,
	Iocp_output,
} io_type_t;

struct SOCKETINFO : public OVERLAPPED
{
	int				nRecvBytes;
	int				nSendBytes;
	WSABUF			wsaBuf;
	CIocpSocket*	pAcceptSocket;
	io_type_t		eIoType;
};

class CIocpWorker : public workflow::CThread
{
public:
	CIocpWorker( HANDLE hComplPort );
	~CIocpWorker();

protected:
private:
	virtual DWORD _Run();

	HANDLE			m_hComplPort;
};

class CIocpSocket
{
public:
	CIocpSocket();
	virtual ~CIocpSocket();

	void	Close();

	virtual BOOL	Accept(SOCKET hSocket, SOCKADDR_IN* pAddr, HANDLE hComplPort);
	BOOL	Connect(const SOCKADDR* lpSockAddr, int nSockAddrLen, HANDLE hComplPort);
	BOOL	Connect(LPCTSTR lpszHostAddress, UINT nHostPort, HANDLE hComplPort);

	BOOL	Recv();
	BOOL	RecvIndirect(void* pBuf, DWORD dwBufLen);
	BOOL	Send(const void* pData, LPDWORD INOUT pdwData);
	BOOL	SendIndirect(const void* pData, LPDWORD INOUT pdwData);
	virtual void	OnReceive();
	virtual void	OnSend();
	virtual void	OnClose();

	SOCKETINFO*		GetSocketInfo() { return m_pSockInfo; }
	SOCKET	GetSafeHandle();

	u32		GetAddr() { return m_addr; }
	u16		GetPort() { return m_port; }

	void	SetReceiveByte( u32 nReceive );
	// for debugging
	u64		GetTotalReceived()	{ return m_ullRecv; }
	u64		GetTotalSended()	{ return m_ullSend; }
	void	AddSended(u64 nSended )	{ m_ullSend += nSended; }
	u32		GetElapsed()		{ return m_nElapsed; }
	double	GetRecvBps();
protected:

	u16		m_port;
	u32		m_addr;

	u64		m_ullRecv;
	u64		m_ullSend;
	u32		m_nTickFirstReceived;
	u32		m_nElapsed;

	HANDLE			m_hComplPort;
	SOCKET			m_sck;
	SOCKETINFO		*m_pSockInfo;
	util::CSyncLock	m_sync;

	char			m_pBuf[DEF_BUF_SIZE];
};

class CIocp
{
public:
	CIocp();
	~CIocp();

	bool	Open();
	void	Close();

	HANDLE	GetComplPort() { return m_hComplPort; }
protected:
	void	CreateWorkers();
	void	StopWorkers();

	HANDLE	m_hComplPort;
	u32		m_nWorkerCnt;

	std::list<std::shared_ptr<CIocpWorker>>		m_lstWorker;
	std::list<std::shared_ptr<CIocpSocket>>		m_lstSockets;
};

class CListenSocket : public workflow::CThread, CIocp
{
public:
	CListenSocket();
	~CListenSocket();

	void	SetOption(u16 nPort, u32 nAddr = ADDR_ANY);
	u16		GetPort();
	u32		GetAddress();
	void	SetWorkerCount(u32 nCount);

protected:
private:
	virtual CIocpSocket*	GenerateSocket();

	virtual	bool	PreStart();
	virtual void	PreStop();
	virtual DWORD	_Run();

	SOCKET	m_sck;
	u16		m_port;
	u32		m_addr;
};

} // namespace net
} // namespace hevc


#ifndef HEVC_TEMPLATE_H
#define HEVC_TEMPLATE_H

#include <memory>
#include <map>
#include <vector>
using std::auto_ptr;
using std::map;

namespace hevc {
namespace design {

//---------------------------------------------------------------------
template <class T>
class CSingleTon
{
private:
	static auto_ptr<T>	g_pInstance;
public:
	static T*	GetInstance(void)
	{
		if (NULL == g_pInstance.get())
		{
			auto_ptr<T> pTemp (new T);
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

template <class T> auto_ptr<T>	CSingleTon<T>::g_pInstance;

#define FRIEND_FOR_SINGLETON_CDTOR(_myclass_)	friend class hevc::design::CSingleTon<_myclass_>; \
											friend class std::auto_ptr<_myclass_>;

//---------------------------------------------------------------------
template <typename T, typename Key> class CMultiton
{
public:    
	static void Release()
	{
		g_pInstances.clear();
	}

	static T* GetInstance(const Key& key)
	{
		typename std::map<Key, auto_ptr<T>>::iterator it = g_pInstances.find(key);

		if (it != g_pInstances.end()) {
			return (it->second.get());
		}

		T* instance = new T;
		g_pInstances[key].reset( instance );
		return instance;
	}

	static T& GetReference(const Key& key)
	{
		return *GetInstance(key);
	}

	static const Key GetKeyByInstance( const T* pInst )
	{
		for (typename std::map<Key, auto_ptr<T>>::iterator it = g_pInstances.begin(); it != g_pInstances.end(); ++it)
		{
			if ( it->second.get() == pInst )
			{
				return it->first;
			}
		}
	}

	static T* GetInstanceByIndex( size_t idx )
	{
		if ( m_instances.size() <= idx )
			return 0;
		typename std::map<Key, std::auto_ptr<T>>::iterator it = g_pInstances.begin();
		it += idx;
		if ( it != g_pInstances.end() ) 
			return it->second.get();
		else 
			return 0;
	}

	static size_t GetSize()
	{
		return g_pInstances.size();
	}

protected:
	CMultiton() {}
	virtual ~CMultiton() { Release(); }

private:
	CMultiton(const CMultiton&) {}
	CMultiton& operator= (const CMultiton&) { return *this; }

	static std::map<Key, auto_ptr<T>> g_pInstances;
};

template <typename T, typename Key> std::map<Key, std::auto_ptr<T>> CMultiton<T, Key>::g_pInstances;

//---------------------------------------------------------------------
template <typename T>
class Deleter {
public:
	Deleter(T* pointer) : pointer_(pointer) { }
	Deleter(const Deleter& deleter) {
		Deleter* d = const_cast<Deleter*>(&deleter);
		pointer_ = d->pointer_;
		d->pointer_ = 0;
	}
	~Deleter() { delete pointer_; }
	operator T*	() { return pointer_; }
	T* pointer_;
};

template <class _type>
void	DeletePtrVector(std::vector<_type> &vct)
{
	for (size_t ii = 0; ii < vct.size(); ii++)
	{
		delete vct.at(ii);
	}

	vct.clear();
};

template <typename E, typename _type>
void	DeletePtrMap(std::map<E, _type> &mp)
{
	std::map<E, _type>::iterator itr = mp.begin();

	for ( ; itr != mp.end(); itr++ )
	{
		delete itr->second;
	}

	mp.clear();
}

//---------------------------------------------------------------------
template <typename T, typename E>
class CAbstractFactory
{
public:
	CAbstractFactory() {};
	virtual ~CAbstractFactory()
	{
		m_vctRef.clear();
	};

	void	Init() {
		m_vctRef.clear();
	}
	T*	Create(E en)
	{
		T* pTemp = AddNew(en);
		return pTemp;
	}
	void	Remove(T* pRef)
	{
		std::vector<std::tr1::shared_ptr<T>>::iterator itr = 
			m_vctRef.begin();
		for ( ; itr != m_vctRef.end(); itr++)
		{
			if ( itr == pRef )
			{
				vector::erase(itr);
			}
		}
	}
protected:
	std::vector<std::tr1::shared_ptr<T>>	m_vctRef;
	virtual T*	Generate(E) = 0;
	T* AddNew(E en) {
		T* pTemp = Generate(en);
		std::tr1::shared_ptr<T> p(pTemp);
		m_vctRef.push_back(p);
		return pTemp;
	}
};

//---------------------------------------------------------------------
template <typename T, typename E>
class CAbstractFactoryB
{
public:
	CAbstractFactoryB() {};
	virtual ~CAbstractFactoryB()
	{
	};

	T*	Create(E en)
	{
		T* pTemp = Generate(en);
		return pTemp;
	}
protected:
	virtual T*	Generate(E) = 0;
};
// factory support index key
template <typename _Type, typename _Key>
class CAbstractMapper
{
public:
	CAbstractMapper() {};
	virtual ~CAbstractMapper()
	{
		m_mapRef.clear();
	};
	void	Init() {
		m_mapRef.clear();
	}
	void	ReleaseAll() 
	{
		m_mapRef.clear();
	}
	_Type*	Create(const _Key& key)
	{
		_Type* pTemp = AddNew(key);
		return pTemp;
	}
	_Type*	Get(const _Key& key)
	{
		if (m_mapRef.count(key) > 0)
		{
			return m_mapRef[key].get();
		}
		return 0;
	}
	_Type* GetByIdx(const size_t idx)
	{
		if ( idx >= m_mapRef.size() ) return 0;

		t_map::iterator itr = m_mapRef.begin();

		itr += idx;

		return itr->first;
	}
	size_t GetSize()
	{
		return m_mapRef.size();
	}
	const _Key GetKey( const _Type* myPtr )
	{
		t_map::iterator itr = m_mapRef.begin();

		for ( ; itr != m_mapRef.end(); itr++ )
		{
			if ( itr->second.get() == myPtr )
				return itr->first;
		}
	}
protected:
	typedef std::map<_Key, std::shared_ptr<_Type>> t_map;
	t_map	m_mapRef;
	virtual _Type*	Generate(const _Key& key) = 0;
private:
	_Type* AddNew(const _Key& key) {
		_Type* pTemp = Generate(key);

		if (pTemp)
		{
			if (m_mapRef.count(key) > 0)
			{
				m_mapRef.erase(key);
			}

			m_mapRef[key] = std::tr1::shared_ptr<_Type>(pTemp);
		}

		return pTemp;
	}
};

// factory support index key and custom creator info
template <typename _Type, typename _Key, typename _Info>
class CAbstractMapperEx 
{
public:
	CAbstractMapperEx() {};
	virtual ~CAbstractMapperEx()
	{
		m_mapRef.clear();
	};

	void	Init() {
		m_mapRef.clear();
	}
	void	ReleaseAll() 
	{
		m_mapRef.clear();
	}
	_Type*	Create(const _Key& key, const _Info& info)
	{
		_Type* pTemp = AddNew(key, info);
		return pTemp;
	}
	_Type*	Get(const _Key& key)
	{
		if (m_mapRef.count(key) > 0)
		{
			return m_mapRef[key].first.get();
		}
		return 0;
	}
	_Type* GetByIdx(const size_t idx)
	{
		if ( idx >= m_mapRef.size() ) return 0;

		std::map<_Key, _MyEntry>::iterator itr = m_mapRef.begin();

		itr += idx;

		return itr->second.first;
	}
	size_t GetSize()
	{
		return m_mapRef.size();
	}
	const _Key GetKey( const _Type* myPtr )
	{
		std::map<_Key, _MyEntry>::iterator itr = m_mapRef.begin();

		for ( ; itr != m_mapRef.end(); itr++ )
		{
			if ( itr->second.first->get() == myPtr )
				return itr->first;
		}
	}
protected:
	typedef std::pair<std::shared_ptr<_Type>, _Info>		_MyEntry;
	std::map<_Key, _MyEntry>	m_mapRef;
	virtual _Type*	Generate(const _Key& key, const _Info& info) = 0;
private:
	_Type* AddNew(const _Key& key, const _Info& info) {
		_Type* pTemp = Generate(key, info);

		if (pTemp)
		{
			if (m_mapRef.count(key) > 0)
			{
				m_mapRef.erase(key);
			}

			m_mapRef[key] = _MyEntry(std::tr1::shared_ptr<_Type>(pTemp), info);
		}

		return pTemp;
	}
};

/*

//---------------------------------------------------------------------
template <typename Manager, typename Entry, typename Key, typename Info>
class CManager
{
public:
	static Manager*	GetManager( const Info& info )
	{
		if (NULL == g_pInstance.get())
		{
			auto_ptr<Manager> pTemp ( new Manager );
			g_pManager = pTemp;
		}
		return g_pManager.get();
	}

	static void Release()
	{
		if ( GetInstance() != NULL )
		{
			delete g_pManager.get();
			g_pManager.release();
		}

		g_pEntryInstances.clear();
	}

	static Entry* GetEntry( const Key& key )
	{
		typename std::map<Key, std::auto_ptr<Entry>>::iterator itr = g_pEntryInstances.find(key);

		if (itr != g_pEntryInstances.end()) {
			return (itr->second.get());
		}

		Entry* instance = new Entry;
		g_pEntryInstances[key].reset( instance );
		return instance;
	}

	static const Key GetKeyByEntry( const Entry* pInst )
	{
		for (typename std::map<Key, auto_ptr<Entry>>::iterator it = g_pEntryInstances.begin(); it != g_pEntryInstances.end(); ++it)
		{
			if ( it->second.get() == pInst )
			{
				return it->first;
			}
		}
	}

	static Entry* GetEntryByIndex( size_t idx )
	{
		if ( m_instances.size() <= idx )
			return 0;
		typename std::map<Key, std::auto_ptr<Entry>>::iterator it = g_pInstances.begin();
		it += idx;
		if ( it != g_pInstances.end() ) 
			return it->second.get();
		else 
			return 0;
	}

	static size_t GetSize()
	{
		return g_pInstances.size();
	}

private:
	static std::auto_ptr<Manager>				g_pManager;
	static std::map<Key, std::auto_ptr<Entry>>	g_pEntryInstances;
};

template <typename Manager, typename Entry, typename Key, typename Info> auto_ptr<Manager>	CManager<Manager, Entry, Key, Info>::g_pManager;
template <typename Manager, typename Entry, typename Key, typename Info> std::map<Key, std::auto_ptr<Entry>> CManager<Manager, Entry, Key, Info>::g_pEntryInstances;
*/
} // namespace Util
} // namespace Framework

#endif // HEVC_TEMPLATE_H
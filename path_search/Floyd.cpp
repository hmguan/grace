#include "Floyd.h"
#include <vector>
#include "Calculate.hpp"
#include "log.h"
#include "DataInfo.h"
#if _WIN32
#include <io.h>
#include <Windows.h>
#include <sys/stat.h> 
#else
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#endif

Floyd::Floyd()
{
	stInvalidWop.edge_id = 0;
	stInvalidWop.wop_id = 0;
	stInvalidWop.direction = PathSearch_FORWARD;
	stInvalidWop.angle_type = PathSearch_CONST_ANGLE;
	stInvalidWop.angle = 0;
	stInvalidWop.distance = INVALID_DISTANCE;
}

Floyd::~Floyd()
{
	for (int i = 0; i < m_WopDistancesLength; i++)
	{
		delete[] m_WopDistances[i];
	}
	delete[] m_WopDistances;

	for (int i = 0; i < m_WopPathsLength; i++)
	{
		delete[] m_WopPaths[i];
	}
	delete[] m_WopPaths;
}

// private
int Floyd::InsertPath( std::vector<unsigned int>&  vctWopPath )
{
	// 至少有2个收尾wop才能进行搜索
	if( vctWopPath.size() < 2 ) {
		return -2;//debug test
	}

	bool bInsert = true;
	while( bInsert ) {
		bInsert = false;
		std::vector<unsigned int>::iterator  iter = vctWopPath.begin( );
		for( size_t i = 0; i < vctWopPath.size( ) - 1; i++ ) {

			unsigned int iStartWopIndex = 0;
			unsigned int iEndWopIndex = 0;
			
			unsigned int Value = m_WopPaths[vctWopPath[i]][vctWopPath[i + 1]];
			if( Value < m_vctWops.size() && Value >= 0 && Value != vctWopPath[i] && Value != vctWopPath[i + 1] ) {
				vctWopPath.insert( iter + i + 1, Value );
				bInsert = true;
				break;
			}
		}
	}
	return 0;
}

int Floyd::GetStartAndEndWop( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	                          int& startWopIndex, int& endWopIndex,
	                          float& fDistanceStart, float& fDistanceEnd )
{
	if( m_vctWops.size() == 0 ) {
		return -1;
	}
	startWopIndex = -1;
	endWopIndex = -1;

	for( size_t wopIndex = 0; wopIndex < m_vctWops.size(); wopIndex++ ) {
		// 起始wop的部分长度
		if( m_vctWops[wopIndex].edge_id == stStartUpl.edge_id && m_vctWops[wopIndex].wop_id == stStartUpl.wop_id ) {
			if( m_vctWops[wopIndex].direction == PathSearch_FORWARD ) {
				fDistanceStart = m_vctWops[wopIndex].distance * ( 1 - stStartUpl.fPercent );
			} else {
				fDistanceStart = m_vctWops[wopIndex].distance * stStartUpl.fPercent;
			}
			startWopIndex = wopIndex;
		}
		// 结束wop的部分长度
		if( m_vctWops[wopIndex].edge_id == stEndUpl.edge_id && m_vctWops[wopIndex].wop_id == stEndUpl.wop_id ) {
			if( m_vctWops[wopIndex].direction == PathSearch_FORWARD ) {
				fDistanceEnd = m_vctWops[wopIndex].distance * stEndUpl.fPercent;
			} else {
				fDistanceEnd = m_vctWops[wopIndex].distance * ( 1 - stEndUpl.fPercent );
			}
			endWopIndex = wopIndex;
		}
	}

	if( startWopIndex < 0 || endWopIndex < 0 ) {
		return -1;
	}
	return 0;
}

int Floyd::IsSameWop( const PathSearch_Wop stWop1, const PathSearch_Wop stWop2 )
{
	if( stWop1.wop_id != stWop2.wop_id || stWop1.edge_id != stWop2.edge_id ) {
		return -1;
	}
	return 0;
}

int Floyd::SameWopPathSearch( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl, 
	                          const unsigned int iSameWopIndex,
							  const float fDistanceStart,
							  const float fDistanceEnd,
							  std::vector<unsigned int>&  vctWopPath, float& fDistance )
{
	if( SameWopInsideSearch( stStartUpl, stEndUpl, iSameWopIndex, vctWopPath, fDistance ) < 0 ) {
		if( SameWopOutsideSearch( stStartUpl, stEndUpl, iSameWopIndex, fDistanceStart, fDistanceEnd, vctWopPath, fDistance ) < 0 ) {
			return -1;
		}		
	}
	return 0;
}

int Floyd::SameWopOutsideSearch( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	                             const unsigned int iSameWopIndex,
								 const float fDistanceStart,
								 const float fDistanceEnd,
	                             std::vector<unsigned int>&  vctWopPath, float& fDistance )
{

	std::vector<unsigned int> vctStartAbleWop;
	std::vector<unsigned int> vctEndAbleWop;
	for( auto iter = m_mapLinks.begin( ); m_mapLinks.end( ) != iter; iter++ ) {
		if( iter->second.link_from.wop_id == m_vctWops[iSameWopIndex].wop_id
			&& iter->second.link_from.edge_id == m_vctWops[iSameWopIndex].edge_id ) {
			for( size_t wopIndex = 0; wopIndex < m_vctWops.size(); wopIndex++ ) {
				if( m_vctWops[wopIndex].wop_id == iter->second.link_to.wop_id
					&& m_vctWops[wopIndex].edge_id == iter->second.link_to.edge_id ) {
					vctStartAbleWop.push_back( wopIndex );
				}
			}
		}
		if( iter->second.link_to.wop_id == m_vctWops[iSameWopIndex].wop_id
			&& iter->second.link_to.edge_id == m_vctWops[iSameWopIndex].edge_id ) {
			for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) {
				if( m_vctWops[wopIndex].wop_id == iter->second.link_from.wop_id
					&& m_vctWops[wopIndex].edge_id == iter->second.link_from.edge_id ) {
					vctEndAbleWop.push_back( wopIndex );
				}
			}
		}
	}

	if( vctStartAbleWop.size( ) == 0 || vctEndAbleWop.size( ) == 0 ) {
		return -1;
	}

	float min_distance = INVALID_DISTANCE;
	float dist = 0;
	unsigned int iMinStartWopIndex;
	unsigned int iMinEndWopIndex;
	for( size_t start_wop = 0; start_wop < vctStartAbleWop.size(); start_wop++ ) {
		for( size_t end_wop = 0; end_wop < vctEndAbleWop.size( ); end_wop++) {

			int iStartWopIndex = vctStartAbleWop[start_wop];
			int iEndWopIndex = vctEndAbleWop[end_wop];
			if( iStartWopIndex != iEndWopIndex ) {

				dist = m_WopDistances[iStartWopIndex][iEndWopIndex]
					+ m_vctWops[iStartWopIndex].distance
					+ m_vctWops[iEndWopIndex].distance
					+ m_WopDistances[iSameWopIndex][iStartWopIndex]
					+ m_WopDistances[iEndWopIndex][iSameWopIndex];
			} else {
				dist = m_vctWops[iStartWopIndex].distance					  
					+ m_WopDistances[iSameWopIndex][iStartWopIndex]
					+ m_WopDistances[iEndWopIndex][iSameWopIndex];
			}

			if( dist < min_distance ) {
				min_distance = dist;
				iMinStartWopIndex = iStartWopIndex;
				iMinEndWopIndex = iEndWopIndex;
			}
		}
	}

	if( min_distance >= INVALID_DISTANCE ) {
		return -1;
	}

	vctWopPath.push_back( iSameWopIndex );
	if( iMinStartWopIndex == iMinEndWopIndex ) {

		vctWopPath.push_back( iMinStartWopIndex );

	} else {
		vctWopPath.push_back( iMinStartWopIndex );
		vctWopPath.push_back( iMinEndWopIndex );
	}
	vctWopPath.push_back( iSameWopIndex );

	InsertPath( vctWopPath );

	fDistance = min_distance + fDistanceStart + fDistanceEnd;
	if( fDistance >= INVALID_DISTANCE ) {
		return -1;
	}
	return 0;
}

int Floyd::SameWopInsideSearch( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	                            const unsigned int iSameWopIndex,
	                            std::vector<unsigned int>&  vctWopPath, float& fDistance )
{
	if( m_vctWops[iSameWopIndex].direction == PathSearch_FORWARD ) {
		if( stStartUpl.fPercent <= stEndUpl.fPercent ) {
			fDistance = m_vctWops[iSameWopIndex].distance * ( stEndUpl.fPercent - stStartUpl.fPercent );
			vctWopPath.push_back( iSameWopIndex );
			return 0;
		} 
	} else {
		if( stStartUpl.fPercent >= stEndUpl.fPercent ) {
			fDistance = m_vctWops[iSameWopIndex].distance * ( stStartUpl.fPercent - stEndUpl.fPercent );
			vctWopPath.push_back( iSameWopIndex );
			return 0;
		}
	}

	return -1;
}

// public
int Floyd::GetFloydId()
{
	return m_iFloydId;
}

int Floyd::SetFloydId(int iFloydId)
{
	m_iFloydId = iFloydId;
	return 0;
}

int Floyd::SetSearchType(unsigned int iSearchType)
{
	m_iSearchType = iSearchType;
	return 0;
}

int Floyd::LoadData( const PathSearch_LayoutData* pLayoutData )
{
	if( !pLayoutData ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData is NULL.";
		return -1;
	}
	
	if( m_mapWopType.size() > 0 ) {
		m_mapWopType.clear();
	}

	for( auto& woptype : pLayoutData->wopList ) {
		PathSearch_Wop wop_type = woptype;
		m_mapWopType[wop_type.wop_id] = wop_type;
	}

	if( m_mapWopType.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData->wopList is empty.";
		return -1;
	}

	if( m_mapEdges.size() > 0 ) {
		m_mapEdges.clear();
	}
	if( m_vctWops.size( ) > 0 ) {
		m_vctWops.clear( );
	}
	if( m_mapLinks.size() > 0 ) {
		m_mapLinks.clear();
	}

	for(auto& edgeInfo : pLayoutData->edgeList ) {
		PathSearch_EdgeInfo edge = edgeInfo;
		m_mapEdges[edge.id] = edge;

		for( auto& wop_id : edgeInfo.wop_list ) {
			PathSearch_Wop wop;
			wop.wop_id = wop_id;
			wop.edge_id = edgeInfo.id;
			wop.angle_type = m_mapWopType[wop_id].angle_type;
			wop.angle = m_mapWopType[wop_id].angle;
			wop.direction = m_mapWopType[wop_id].direction;
			wop.distance = edgeInfo.length;

			m_vctWops.push_back( wop );
		}
	}
	int iLength = m_vctWops.size();
	if (iLength != m_WopDistancesLength)
	{
		if (m_WopDistancesLength > 0)
		{//释放原来的二维数组空间
			for (int i = 0; i < m_WopDistancesLength; i++)
			{
				delete[] m_WopDistances[i];
			}
			delete[] m_WopDistances;
		}

		//申请新的二维数组空间
		m_WopDistances = new float*[iLength];
		for (int i = 0; i < iLength; i++)
		{
			m_WopDistances[i] = new float[iLength];
		}

		m_WopDistancesLength = iLength;
	}

	if( m_mapEdges.size( ) == 0 || m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData->edgeList is empty. or pLayoutData->edgeList.wop_list is empty.";
		return -1;
	}

	for( size_t startWopIndex = 0; startWopIndex < m_vctWops.size( ); startWopIndex++ ) {

		for( size_t endWopIndex = 0; endWopIndex < m_vctWops.size( ); endWopIndex++ ) {

			if( startWopIndex == endWopIndex ) {
				m_WopDistances[startWopIndex][endWopIndex] = 0;
			} else {
				m_WopDistances[startWopIndex][endWopIndex] = INVALID_DISTANCE;
			}
		}
	}

	for( auto& link : pLayoutData->linkList ) {
		if( link.convert_cost < 0 ) {
			loerror("path_search") << "PathSearch_LoadData: link.convert_cost < 0.";
			return -1;
		}
		PathSearch_Link Link = link;
		m_mapLinks[Link.link_id] = Link;

		unsigned int startWopIndex = 0;
		unsigned int endWopIndex = 0;
		for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) {
			if( Link.link_from.edge_id == m_vctWops[wopIndex].edge_id
				&& Link.link_from.wop_id == m_vctWops[wopIndex].wop_id ) {
				startWopIndex = wopIndex;
			}
			if( Link.link_to.edge_id == m_vctWops[wopIndex].edge_id
				&& Link.link_to.wop_id == m_vctWops[wopIndex].wop_id ) {
				endWopIndex = wopIndex;
			}
		}
		m_WopDistances[startWopIndex][endWopIndex] = ( float )Link.convert_cost;
	}

	if( m_mapLinks.size() == 0 ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData->linkList is empty.";
		return -1;
	}

	return 0;
}

int Floyd::SetWops( const PathSearch_Wop* pWops, const unsigned int iLength )
{
	if( pWops == nullptr || iLength == 0 ) {
		loerror("path_search") << "PathSearch_SetWops: pWops is NULL.or iLength == 0.";
		return -1;
	}

	/*
	if( m_WopDistances.size( ) > 0 ) {
		m_WopDistances.clear( );
	}
	*/

	//optimize
	if (iLength != m_WopDistancesLength)
	{
		if (m_WopDistancesLength > 0)
		{//释放原来的二维数组空间
			for (int i = 0; i < m_WopDistancesLength; i++)
			{
				delete[] m_WopDistances[i];
			}
			delete[] m_WopDistances;
		}

		//申请新的二维数组空间
		m_WopDistances = new float*[iLength];
		for (int i = 0; i < iLength; i++)
		{
			m_WopDistances[i] = new float[iLength];
		}

		m_WopDistancesLength = iLength;
	}

	if( m_vctWops.size( ) > 0 ) {
		m_vctWops.clear( );
	}

	if( m_mapWopType.size() > 0 ) {
		m_mapWopType.clear();
	}

	for( size_t startWopIndex = 0; startWopIndex < iLength; startWopIndex++ ) {
		PathSearch_Wop wop = pWops[startWopIndex];
		m_vctWops.push_back( wop );

		m_mapWopType[wop.wop_id] = wop;

		for( size_t endWopIndex = 0; endWopIndex < iLength; endWopIndex++ ) {

			if( startWopIndex == endWopIndex ) {
				m_WopDistances[startWopIndex][endWopIndex] = 0;
			} else {
				m_WopDistances[startWopIndex][endWopIndex] = INVALID_DISTANCE;
			}
		}
	}
	return 0;
}

int Floyd::SetLinks( const PathSearch_Link* pLinks, const unsigned int iLength )
{
	if( pLinks == nullptr || iLength == 0 ) {
		loerror("path_search") << "PathSearch_SetLinks: pLinks is NULL.or iLength == 0.";
		return -1;
	}
	if( m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_SetLinks: wops number is 0.";
		return -1;
	}
	if( m_mapLinks.size( ) > 0 ) {
		m_mapLinks.clear( );
	}
	for( size_t i = 0; i < iLength; i++ ) {

		unsigned int link_id = pLinks[i].link_id;
		PathSearch_Link link = pLinks[i];
		m_mapLinks[link_id] = link;
		if( link.convert_cost < 0 ) {
			loerror("path_search") << "PathSearch_SetLinks: link.convert_cost < 0.";
			return -1;
		}

		unsigned int startWopIndex = 0;
		unsigned int endWopIndex = 0;

		for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) {
			if( link.link_from.edge_id == m_vctWops[wopIndex].edge_id
				&& link.link_from.wop_id == m_vctWops[wopIndex].wop_id ) {
				startWopIndex = wopIndex;
			}
			if( link.link_to.edge_id == m_vctWops[wopIndex].edge_id
				&& link.link_to.wop_id == m_vctWops[wopIndex].wop_id ) {
				endWopIndex = wopIndex;
			}
		}

		m_WopDistances[startWopIndex][endWopIndex] = ( float )link.convert_cost;
	}
	return 0;
}

int Floyd::SetEdges( const PathSearch_EdgeInfo* pEdges, const unsigned int iLength )
{
	if( pEdges == nullptr || iLength == 0 ) {
		loerror("path_search") << "PathSearch_SetEdges: pEdges is NULL. or iLength == 0.";
		return -1;
	}

	if( m_mapEdges.size( ) > 0 ) {
		m_mapEdges.clear( );
	}
	for( size_t i = 0; i < iLength; i++ ) {
		PathSearch_EdgeInfo edge;
		edge = pEdges[i];
		m_mapEdges[edge.id] = edge;
	}
	return 0;
}

int Floyd::StartFloydPath(FinishLevelCB FinishLevelCB, void* pUsr)
{
	/*
	if (m_WopPaths.size() > 0) {
		m_WopPaths.clear();
	}
	*/

	//optimize
	if (m_WopPathsLength != m_WopDistancesLength)
	{
		if (m_WopPathsLength > 0)
		{//释放原来的二维数组空间
			for (int i = 0; i < m_WopPathsLength; i++)
			{
				delete[] m_WopPaths[i];
			}
			delete[] m_WopPaths;
		}

		//申请新的二维数组空间
		m_WopPaths = new int*[m_WopDistancesLength];
		for (int i = 0; i < m_WopDistancesLength; i++)
		{
			m_WopPaths[i] = new int[m_WopDistancesLength];
		}

		m_WopPathsLength = m_WopDistancesLength;
	}
	loinfo("path_search") << "wops size=" << m_vctWops.size();
	unsigned int iWopsNum = m_vctWops.size();
	for (size_t startWopIndex = 0; startWopIndex < iWopsNum; startWopIndex++) {
		for (size_t endWopIndex = 0; endWopIndex < iWopsNum; endWopIndex++) {
			m_WopPaths[startWopIndex][endWopIndex] = INVALID_PATH_INDEX;
		}
	}
	int iLevel = 0;
	for (size_t pathWopIndex = 0; pathWopIndex < iWopsNum; pathWopIndex++) {
		for (size_t fromWopIndex = 0; fromWopIndex < iWopsNum; fromWopIndex++) {
			for (size_t toWopIndex = 0; toWopIndex < iWopsNum; toWopIndex++) {
				if (m_WopDistances[fromWopIndex][toWopIndex] > m_WopDistances[fromWopIndex][pathWopIndex] + m_WopDistances[pathWopIndex][toWopIndex] + m_vctWops[pathWopIndex].distance) {
					m_WopDistances[fromWopIndex][toWopIndex] = m_WopDistances[fromWopIndex][pathWopIndex] + m_WopDistances[pathWopIndex][toWopIndex] + m_vctWops[pathWopIndex].distance;
					m_WopPaths[fromWopIndex][toWopIndex] = pathWopIndex;
				}
			}
		}
		iLevel++;
		int iPercent = ((iLevel)* 100) / (iWopsNum);
		FinishLevelCB(iPercent, pUsr);
	}
	return 0;
}
int Floyd::WriteFloydData(float**& MatrixWopDistances, int**& MatrixWopPaths, FloydDate*date)
{
	if (m_WopDistancesLength == 0 || m_vctWops.size() == 0) {
		loerror("path_search") << "PathSearch_StartFolyd: wops number is 0.";
		return -1;
	}
	int i = 0;
	for (size_t startIndex = 0; startIndex < m_WopDistancesLength; startIndex++) {
		for (size_t endIndex = 0; endIndex < m_WopDistancesLength; endIndex++) {
			date[i].iStartIndex = startIndex;
			date[i].iEndIndex = endIndex;
			date[i].iPathIndex = MatrixWopPaths[startIndex][endIndex];
			date[i].fDistance = MatrixWopDistances[startIndex][endIndex];
			i++;
		}
	}
	return 0;
}

int Floyd::ReadFLoydData(FloydDate*date, int iCount, float**& MatrixWopDistances, int**& MatrixWopPaths)
{
	double table[5];
	//int iCount = m_vctWops.size()*m_vctWops.size();

	//optimize
	int length = m_vctWops.size();
	if (m_WopDistancesLength != length)
	{
		if (m_WopDistancesLength > 0)
		{//释放原来的二维数组空间
			for (int i = 0; i < m_WopDistancesLength; i++)
			{
				delete[] m_WopDistances[i];
			}
			delete[] m_WopDistances;
		}

		//申请新的二维数组空间
		m_WopDistances = new float*[length];
		for (int i = 0; i < length; i++)
		{
			m_WopDistances[i] = new float[length];
		}

		m_WopDistancesLength = length;
	}
	if (m_WopPathsLength != length)
	{
		if (m_WopPathsLength > 0)
		{//释放原来的二维数组空间
			for (int i = 0; i < m_WopPathsLength; i++)
			{
				delete[] m_WopPaths[i];
			}
			delete[] m_WopPaths;
		}

		//申请新的二维数组空间
		m_WopPaths = new int*[length];
		for (int i = 0; i < length; i++)
		{
			m_WopPaths[i] = new int[length];
		}

		m_WopPathsLength = length;
	}

	for (int j = 0; j < iCount; j++) 
	{
		table[0] = date[j].iStartIndex;
		table[1] = date[j].iEndIndex;
		table[2] = date[j].iPathIndex;
		table[3] = date[j].fDistance;
	
		unsigned int startIndex = table[0];
		unsigned int endIndex = table[1];

		MatrixWopPaths[startIndex][endIndex] = table[2];
		MatrixWopDistances[startIndex][endIndex] = table[3];
	}

return 0;
}

int Floyd::StartFolyd(const char* pFloydPath, FinishLevelCB FinishLevelCB, void* pUsr)
{
	if( m_WopDistancesLength == 0 || m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_StartFolyd: wops number is 0.";
		return -1;
	}
	DataInfo datainfo;
	int iCount = m_vctWops.size()*m_vctWops.size();
	//FloydDate *floyd_date = new FloydDate[iCount];
	//FloydDate *floyd_date=new FloydDate[iCount];

#if _WIN32
	//获取文件属性，在Floyd数据.dat文件size大于零，且修改时间不比地图.xml文件晚的情况下才读取.dat文件，否则就重新生成.dat文件
	bool WhetherReadDatFile;
	struct _stat buf;
	int result = _stat(pFloydPath, &buf);
	WhetherReadDatFile = (result == 0);
	if (WhetherReadDatFile)
	{
		WhetherReadDatFile = (buf.st_size > 0);//文件大小需要大于零
		if (WhetherReadDatFile)
		{//判断.dat文件修改时间不比地图.xml文件晚
			std::string file_path(pFloydPath);
			int last_pos = file_path.find_last_of('.');
			std::string path = file_path.substr(0, last_pos + 1);
			std::string new_path = path + "xml";//根据dat文件路径，生成map文件路径
			struct _stat buf_map;
			result = _stat(new_path.c_str(), &buf_map);
			WhetherReadDatFile = (result == 0);
			if (WhetherReadDatFile)
			{
				WhetherReadDatFile = (buf.st_mtime >= buf_map.st_mtime);
			}
		}
	}
	if (WhetherReadDatFile && _access(pFloydPath, 0) == 0){
#else
	if (access( pFloydPath, F_OK) == 0) {
#endif
	//	#if _WIN32
		FILE *fp = nullptr;
		fp = fopen(pFloydPath, "rb+");
		if (fp == NULL) {
			return -1;
		}
		long size;
		fseek(fp, 0, SEEK_END);
		size = ftell(fp);
		fseek(fp, 0, SEEK_SET);
		DataInfo datainfo;
		int iCount = m_vctWops.size()*m_vctWops.size();
		FloydDate *rfloyd_date;
		char*ptr;
		if (iCount <= 2000){
			try{
				rfloyd_date = new FloydDate[iCount];
				FloydDate * pori = rfloyd_date;
				ptr = new char[size];
				fread(ptr, size, 1, fp);
				rfloyd_date = (FloydDate*)ptr;
				ReadFLoydData(rfloyd_date, iCount, m_WopDistances, m_WopPaths);
				delete[] rfloyd_date;
				delete[] pori;
			}
			catch (...){
				loerror("path_search") << "new rFloydDate[] failed";
			}
		}
		else{
			int inum = iCount / 2000000;
			int iover = iCount % 2000000;
			for (int i = 0; i < inum; i++){
				try{
					rfloyd_date = new FloydDate[2000000];
					FloydDate * pori = rfloyd_date;
					ptr = new char[2000000 * sizeof(FloydDate)];
					fread(ptr, 2000000 * sizeof(FloydDate), 1, fp);
					rfloyd_date = (FloydDate*)ptr;
					fseek(fp, 0, SEEK_CUR);
					ReadFLoydData(rfloyd_date, 2000000, m_WopDistances, m_WopPaths);
					delete[] rfloyd_date;
					delete[] pori;
				}
				catch (...){
					loerror("path_search") << "new rFloydDate[] or ptr failed";
				}
			}
			try{
				rfloyd_date = new FloydDate[iover];
				FloydDate * pori = rfloyd_date;
				ptr = new char[iover * sizeof(FloydDate)];
				fread(ptr, iover * sizeof(FloydDate), 1, fp);
				rfloyd_date = (FloydDate*)ptr;
				ReadFLoydData(rfloyd_date,iover, m_WopDistances, m_WopPaths);
				delete[] rfloyd_date;
				delete[] pori;
			}
			catch (...){
				loerror("path_search") << "new rFloydDate[] or ptr failed";
			}
		}

		fclose(fp);

		/*#else
			FILE *fp=nullptr;
			fp = fopen(pFloydPath,"rb+");
			if (fp == NULL) {
				return -1;
			}
			long size;
			fseek(fp, 0, SEEK_END); 
			size = ftell(fp);
			fseek(fp, 0, SEEK_SET);
			char*ptr= new char[size];
			fread(ptr, size,1,fp);
			floyd_date = (FloydDate*)ptr;
			if (floyd_date == NULL){
				delete[] floyd_date;
				return -1;
			}
			ReadFLoydData(floyd_date, m_WopDistances, m_WopPaths);
			fclose(fp);
		#endif	*/	
	}
	else{
		FloydDate *wfloyd_date;
		FILE *fp = nullptr;
		fp = fopen(pFloydPath, "ab+");
		if (fp == NULL) {
			loerror("path_search") << "open dat failed";
			return -1;
		}
		//#if _WIN32

		StartFloydPath(FinishLevelCB, pUsr);
        loinfo("path_search") << "m_mapWopDistances.size" << m_WopDistancesLength * m_WopDistancesLength;
        loinfo("path_search") << "m_vctWops.size" << m_vctWops.size();
        loinfo("path_search") << "icount=" << iCount;
			
		if (iCount <= 2000000){
			wfloyd_date = new FloydDate[iCount];
			WriteFloydData(m_WopDistances, m_WopPaths, wfloyd_date);
			fwrite(wfloyd_date, sizeof(FloydDate)*iCount, 1, fp);
			delete[] wfloyd_date;
		}
		else {
			for (size_t startIndex = 0; startIndex < m_WopDistancesLength; startIndex++){
				try{
					wfloyd_date = new FloydDate[m_WopDistancesLength];
					int i = 0;
					for (size_t endIndex = 0; endIndex <m_WopDistancesLength; endIndex++) {
						wfloyd_date[i].iStartIndex = startIndex;
						wfloyd_date[i].iEndIndex = endIndex;
						wfloyd_date[i].iPathIndex = m_WopPaths[startIndex][endIndex];
						wfloyd_date[i].fDistance = m_WopDistances[startIndex][endIndex];
						i++;
					}
					fwrite(wfloyd_date, sizeof(FloydDate)*m_WopDistancesLength, 1, fp);
					loinfo("path_search") << "write dat file OK";
					delete[] wfloyd_date;
				}
				catch (...){
					loerror("path_search") << "new wFloydDate[] failed";
				}
			}
				
		}
		fclose(fp);
		/*#else
			FILE *fp=nullptr;
			fp = fopen(pFloydPath,"wb+");
			if (fp == NULL) {
				return -1;
			}
			StartFloydPath(FinishLevelCB, pUsr);
			if (floyd_date == nullptr){
                loerror("path_search") << "floyd_date nullptr.";
				delete[] floyd_date;
				return -1;
			}
			WriteFloydData(m_WopDistances, m_WopPaths, floyd_date);
			fwrite(floyd_date, sizeof(FloydDate)*iCount,1,fp);	
            loinfo("path_search") << "write dat file OK";			
			fclose(fp);
		#endif*/
	}
	
	//delete[] floyd_date;
	return 0;
}

int Floyd::GetPath( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance )
{
	if( m_WopPathsLength == 0 || m_WopDistancesLength == 0 || m_vctWops.size( ) == 0 ) {
		return -1;
	}

	int iStartWopIndex = 0;
	int iEndWopIndex = 0;
	float fDistanceStart = 0;
	float fDistanceEnd = 0;

	if( GetStartAndEndWop( stStartUpl, stEndUpl, iStartWopIndex, iEndWopIndex, fDistanceStart, fDistanceEnd ) < 0 ) {
		return -1;
	}

	*fDistance = m_WopDistances[iStartWopIndex][iEndWopIndex] + fDistanceStart + fDistanceEnd;
	if( *fDistance >= INVALID_DISTANCE ) {
		return -1;
	}

	PathSearch_Wop 	stStartWop = m_vctWops[iStartWopIndex];
	PathSearch_Wop  stEndWop = m_vctWops[iEndWopIndex];
	std::vector<unsigned int /*wop_index*/>  vctWopPath;
	// 不同wop
	if( IsSameWop( stStartWop, stEndWop ) < 0 ) {

		vctWopPath.push_back( iStartWopIndex );
		vctWopPath.push_back( iEndWopIndex );
		InsertPath( vctWopPath );
	}
	// 相同的wop
	else if( IsSameWop( stStartWop, stEndWop ) == 0 ) {
		if( SameWopPathSearch( stStartUpl, stEndUpl, iStartWopIndex, fDistanceStart, fDistanceEnd, vctWopPath, *fDistance ) < 0 ) {
			return -1;
		}
	}

	if( vctWopPath.size( ) > 0 ) {
		*path = new PathSearch_Wop[vctWopPath.size( )];
	}

	int offset = 0;
	for( auto& wopIndex : vctWopPath ) {
		PathSearch_Wop wop = m_vctWops[wopIndex];
		memcpy( *path + offset, &wop, sizeof( PathSearch_Wop ) );
		offset++;
	}
	*iLength = vctWopPath.size( );

	return 0;
}

int Floyd::GetPathByUpl( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance )
{
	if (!path){
		return -1;
	}
	*path = NULL;
	if( m_mapWopType.size( ) == 0 || m_mapEdges.size( ) == 0 || m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_GetPath: wops number is 0. or edges number is 0.";
		return -1;
	}
	// 首先通过给定UPL获取改upl上有效的带wop的upl列表
	std::vector<PathSearch_UPL> vctStartUpl;
	std::vector<PathSearch_UPL>	vctEndUpl;
	DataCalculate::Upl2UplList( stStartUpl, m_mapWopType, m_mapEdges, vctStartUpl );
	DataCalculate::Upl2UplList( stEndUpl, m_mapWopType, m_mapEdges, vctEndUpl );

	// 需要判断起始和结束upl是否已经指定了wop
	bool bExistStartWop = false;
	bool bExistEndWop = false;
	for( auto& wop : m_vctWops ) {
		if( stStartUpl.edge_id == wop.edge_id && stStartUpl.wop_id == wop.wop_id ) {
			bExistStartWop = true;
		}
		if( stEndUpl.edge_id == wop.edge_id && stEndUpl.wop_id == wop.wop_id ) {
			bExistEndWop = true;
		}
	}

	if( vctStartUpl.size() == 0 || vctEndUpl.size() == 0 ) {
		loerror("path_search") << "PathSearch_GetPath: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}

	// 如果已经指定wop,则需要判断该upl使用指定的wop是否有效，如果没有指定wop,则需要找出该upl能使用的有效的wop，组成upl列表
	if( bExistStartWop == true ) {
		bool bFindStartWop = false;
		for( auto& start_wop : vctStartUpl ) {
			if( start_wop.wop_id == stStartUpl.wop_id && start_wop.edge_id == stStartUpl.edge_id ) {
				vctStartUpl.clear();
				vctStartUpl.push_back( stStartUpl );
				bFindStartWop = true;
				break;
			}
		}

		if( bFindStartWop == false ) {
			loerror("path_search") << "PathSearch_GetPath: start Upl's specified wop is not valid.";
			return -1;
		}
	} 

	if( bExistEndWop == true ) {
		bool bFindEndWop = false;
		for( auto& end_wop : vctEndUpl ) {
			if( end_wop.wop_id == stEndUpl.wop_id && end_wop.edge_id == stEndUpl.edge_id ) {
				vctEndUpl.clear( );
				vctEndUpl.push_back( stEndUpl );
				bFindEndWop = true;
				break;
			}
		}

		if( bFindEndWop == false ) {
			loerror("path_search") << "PathSearch_GetPath: end Upl's specified wop is not valid.";
			return -1;
		}
	} 

	// 能够使用的带wop的起始UPL都存在
	if( vctStartUpl.size() == 0 || vctEndUpl.size() == 0 ) {
		loerror("path_search") << "PathSearch_GetPath: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}

	PathSearch_Wop* tmp_path = nullptr;
	int tmp_length = 0;
	float tmp_distance = 0;
	float min_distance = INVALID_DISTANCE;
	PathSearch_UPL	best_startUpl;
	PathSearch_UPL  best_endUpl;
	// 遍历，找到最优的带wop的起始和结束upl
	for( auto& startUpl : vctStartUpl ) {
		for( auto& endUpl : vctEndUpl ) {		
			loinfo("path_search") << "startUpl " << startUpl.wop_id << "endUpl " << endUpl.wop_id;
			if( GetPath( startUpl, endUpl, &tmp_path, &tmp_length, &tmp_distance ) < 0 ) {
				//Releasepath( tmp_path );
				break;
			}
			if (tmp_path){
				Releasepath( tmp_path );
			}
			loinfo("path_search") << "startUpl " << startUpl.wop_id << "endUpl " << endUpl.wop_id << "dis" << tmp_distance;
			tmp_path = nullptr;
			if( tmp_distance + tmp_length*1 < min_distance ) {
				min_distance = tmp_distance + tmp_length * 1;
				best_startUpl = startUpl;
				best_endUpl = endUpl;
			}
		}
	}

	// 最优的最短距离
	if( min_distance >= INVALID_DISTANCE ) {
		loerror("path_search") << "PathSearch_GetPath: path is not valid.";
		return -1;
	}

	// 将最优的路径反馈
	if( GetPath( best_startUpl, best_endUpl, path, iLength, fDistance ) < 0 ) {
		loerror("path_search") << "PathSearch_GetPath: path is not valid.";
		return -1;
	}
	return 0;
}

int Floyd::GetPathByPos( const PathSearch_Position stStartPos, const PathSearch_Position stEndPos, PathSearch_Wop** path, int* iLength, float* fDistance )
{
	if( m_mapWopType.size( ) == 0 || m_mapEdges.size( ) == 0 || m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_GetPathByPos: wops number is 0. or edges number is 0.";
		return -1;
	}
	// 根据点，找到有效的可搜路的带wop的upl列表
	std::vector<PathSearch_UPL> vctStartUpl;
	std::vector<PathSearch_UPL>	vctEndUpl;
	PS_position_t pos_start;
	PS_position_t pos_end;
	pos_start.x_ = stStartPos.x;
	pos_start.y_ = stStartPos.y;
	pos_start.angle_ = stStartPos.angle;
	pos_end.x_ = stEndPos.x;
	pos_end.y_ = stEndPos.y;
	pos_end.angle_ = stEndPos.angle;
	DataCalculate::Point2UplList( pos_start, m_mapWopType, m_mapEdges, vctStartUpl );
	DataCalculate::Point2UplList( pos_end, m_mapWopType, m_mapEdges, vctEndUpl );

	// 存在有效的起始和结束的带wop的upl
	if( vctStartUpl.size() == 0 || vctEndUpl.size() == 0 ) {
		loerror("path_search") << "PathSearch_GetPathByPos: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}
	
	PathSearch_Wop* tmp_path = nullptr;
	int tmp_length = 0;
	float tmp_distance = 0;
	float min_distance = INVALID_DISTANCE;
	PathSearch_UPL	best_startUpl;
	PathSearch_UPL  best_endUpl;
	// 遍历，找到最优的带wop的起始和结束upl
	for( auto& startUpl : vctStartUpl ) {
		for( auto& endUpl : vctEndUpl ) {
			if( GetPath( startUpl, endUpl, &tmp_path, &tmp_length, &tmp_distance ) < 0 ) {
				//Releasepath( tmp_path );
				break;
			}
			if (tmp_path){
				Releasepath(tmp_path);
			}
			tmp_path = nullptr;
			if( tmp_distance < min_distance ) {
				min_distance = tmp_distance;
				best_startUpl = startUpl;
				best_endUpl = endUpl;
			}		
		}
	}

	// 最优的最短距离
	if( min_distance >= INVALID_DISTANCE ) {
		loerror("path_search") << "PathSearch_GetPathByPos: path is not valid.";
		return -1;
	}
  
	// 将最优的路径反馈
	if( GetPath( best_startUpl, best_endUpl, path, iLength, fDistance ) < 0 ) {
		loerror("path_search") << "PathSearch_GetPathByPos: path is not valid.";
		return -1;
	}
	
	return 0;
}

int Floyd::GetPathBySPosAndEUpl( const PathSearch_Position stStartPos, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance )
{  
	if( m_mapWopType.size( ) == 0 || m_mapEdges.size( ) == 0 || m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_GetPathBySPosAndEUpl: wops number is 0. or edges number is 0.";
		return -1;
	}

	std::vector<PathSearch_UPL> vctStartUpl;
	std::vector<PathSearch_UPL>	vctEndUpl;

	// 根据点，找到有效的可搜路的带wop的起始upl列表
	PS_position_t pos_start;
	pos_start.x_ = stStartPos.x;
	pos_start.y_ = stStartPos.y;
	pos_start.angle_ = stStartPos.angle;
	DataCalculate::Point2UplList( pos_start, m_mapWopType, m_mapEdges, vctStartUpl );
	// 通过给定UPL获取改upl上有效的带wop的结束upl列表
	DataCalculate::Upl2UplList( stEndUpl, m_mapWopType, m_mapEdges, vctEndUpl );

	// 需要判断结束upl是否已经指定了wop
	bool bExistEndWop = false;
	for( auto& wop : m_vctWops ) {
		if( stEndUpl.edge_id == wop.edge_id && stEndUpl.wop_id == wop.wop_id ) {
			bExistEndWop = true;
		}
	}

	if( vctStartUpl.size( ) == 0 || vctEndUpl.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_GetPathBySPosAndEUpl: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}

	//如果已经制定wop,则判断这个wop是否有效的
	if( bExistEndWop == true ) {
		bool bFindEndWop = false;
		for( auto& end_wop : vctEndUpl ) {
			if( end_wop.wop_id == stEndUpl.wop_id && end_wop.edge_id == stEndUpl.edge_id ) {
				vctEndUpl.clear( );
				vctEndUpl.push_back( stEndUpl );
				bFindEndWop = true;
				break;
			}
		}

		if( bFindEndWop == false ) {
			loerror("path_search") << "PathSearch_GetPathBySPosAndEUpl: end Upl's specified wop is not valid.";
			return -1;
		}
	}

	// 存在有效的起始和结束的带wop的upl
	if( vctStartUpl.size( ) == 0 || vctEndUpl.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_GetPathBySPosAndEUpl: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}

	PathSearch_Wop* tmp_path = nullptr;
	int tmp_length = 0;
	float tmp_distance = 0;
	float min_distance = INVALID_DISTANCE;
	PathSearch_UPL	best_startUpl;
	PathSearch_UPL  best_endUpl;
	// 遍历，找到最优的带wop的起始和结束upl
	for( auto& startUpl : vctStartUpl ) {
		for( auto& endUpl : vctEndUpl ) {
			if( GetPath( startUpl, endUpl, &tmp_path, &tmp_length, &tmp_distance ) < 0 ) {
				//Releasepath( tmp_path );
				break;
			}
			if (tmp_path){
				Releasepath(tmp_path);
			}
			tmp_path = nullptr;
			if( tmp_distance < min_distance ) {
				min_distance = tmp_distance;
				best_startUpl = startUpl;
				best_endUpl = endUpl;
			}
		}
	}

	// 最优的最短距离
	if( min_distance >= INVALID_DISTANCE ) {
		loerror("path_search") << "PathSearch_GetPathBySPosAndEUpl: path is not valid.";
		return -1;
	}

	// 将最优的路径反馈
	if( GetPath( best_startUpl, best_endUpl, path, iLength, fDistance ) < 0 ) {
		loerror("path_search") << "PathSearch_GetPathBySPosAndEUpl: path is not valid.";
		return -1;
	}

	return 0;
}

void Floyd::Releasepath( PathSearch_Wop * path )
{
	if( path ) {
		delete[] path;
		path = nullptr;
	}
}





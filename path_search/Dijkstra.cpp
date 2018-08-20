#include "Dijkstra.h"
#include "Calculate.hpp"
#include "log.h"
#include <thread>
#include <memory>
#include <functional>
#include <iostream>
//#include "DHLTest_TimerCounter.h"
//#include <QtWidgets/qmessagebox.h>

Dijkstra::Dijkstra()
{
}

Dijkstra::~Dijkstra()
{
	for (int i = 0; i < m_WopDistancesLength; i++)
	{
		delete[] m_WopDistances[i];
	}
	delete[] m_WopDistances;

}

// private
int Dijkstra::IsSameWop( const PathSearch_Wop stWop1, const PathSearch_Wop stWop2 )
{
	if( stWop1.wop_id != stWop2.wop_id || stWop1.edge_id != stWop2.edge_id ) {
		return -1;
	}
	return 0;
}

int Dijkstra::GetStartAndEndWop( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	int& startWopIndex, int& endWopIndex,
	float& fDistanceStart, float& fDistanceEnd )
{
	if( m_vctWops.size( ) == 0 ) {
		return -1;
	}
	startWopIndex = -1;
	endWopIndex = -1;

	for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) {
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

int Dijkstra::GetStartAndEndWeightWop(const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	int& startWopIndex, int& endWopIndex,
	float& fDistanceStart, float& fDistanceEnd, std::vector<PathSearch_Wop>& m_vctWops_tmp)
{
	if (m_vctWops_tmp.size() == 0) {
		return -1;
	}
	startWopIndex = -1;
	endWopIndex = -1;

	for (size_t wopIndex = 0; wopIndex < m_vctWops_tmp.size(); wopIndex++) {
		// 起始wop的部分长度
		if (m_vctWops_tmp[wopIndex].edge_id == stStartUpl.edge_id && m_vctWops_tmp[wopIndex].wop_id == stStartUpl.wop_id) {
			if (m_vctWops_tmp[wopIndex].direction == PathSearch_FORWARD) {
				fDistanceStart = m_vctWops_tmp[wopIndex].distance * (1 - stStartUpl.fPercent);
			}
			else {
				fDistanceStart = m_vctWops_tmp[wopIndex].distance * stStartUpl.fPercent;
			}
			startWopIndex = wopIndex;
		}
		// 结束wop的部分长度
		if (m_vctWops_tmp[wopIndex].edge_id == stEndUpl.edge_id && m_vctWops_tmp[wopIndex].wop_id == stEndUpl.wop_id) {
			if (m_vctWops_tmp[wopIndex].direction == PathSearch_FORWARD) {
				fDistanceEnd = m_vctWops_tmp[wopIndex].distance * stEndUpl.fPercent;
			}
			else {
				fDistanceEnd = m_vctWops_tmp[wopIndex].distance * (1 - stEndUpl.fPercent);
			}
			endWopIndex = wopIndex;
		}
	}

	if (startWopIndex < 0 || endWopIndex < 0) {
		return -1;
	}
	return 0;
}

int Dijkstra::SameWopPathSearch( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	const unsigned int iSameWopIndex,
	const float fDistanceStart,
	const float fDistanceEnd,
	std::vector<unsigned int>&  vctWopPath, float& fDistance, std::vector<PathSearch_Wop>& m_vctWops_tmp)
{
	if (m_vctWops_tmp.size() == m_vctWops.size()){
		if (SameWopInsideWeightSearch(stStartUpl, stEndUpl, iSameWopIndex, vctWopPath, fDistance, m_vctWops_tmp)<0){
			if (SameWopOutsideWeightSearch(stStartUpl, stEndUpl, iSameWopIndex, fDistanceStart, fDistanceEnd, vctWopPath, fDistance, m_vctWops_tmp) < 0) {
				return -1;
			}
		}
		return 0;
	}
	if (SameWopInsideSearch(stStartUpl, stEndUpl, iSameWopIndex, vctWopPath, fDistance) < 0){
			if (SameWopOutsideSearch(stStartUpl, stEndUpl, iSameWopIndex, fDistanceStart, fDistanceEnd, vctWopPath, fDistance) < 0) {
				return -1;
			}
		}
	
	return 0;
}

int Dijkstra::SameWopInsideSearch( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	const unsigned int iSameWopIndex,
	std::vector<unsigned int>&  vctWopPath, float& fDistance)
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

int Dijkstra::SameWopInsideWeightSearch(const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	const unsigned int iSameWopIndex,
	std::vector<unsigned int>&  vctWopPath, float& fDistance, std::vector<PathSearch_Wop>& m_vctWops_tmp)
{
	//if (m_vctWops_tmp.size() == m_vctWops.size())
	if (m_vctWops_tmp[iSameWopIndex].direction == PathSearch_FORWARD) {
		if (stStartUpl.fPercent <= stEndUpl.fPercent) {
			fDistance = m_vctWops_tmp[iSameWopIndex].distance * (stEndUpl.fPercent - stStartUpl.fPercent);
			vctWopPath.push_back(iSameWopIndex);
			return 0;
		}
	}
	else {
		if (stStartUpl.fPercent >= stEndUpl.fPercent) {
			fDistance = m_vctWops_tmp[iSameWopIndex].distance * (stStartUpl.fPercent - stEndUpl.fPercent);
			vctWopPath.push_back(iSameWopIndex);
			return 0;
		}
	}
	return -1;
}

int Dijkstra::SameWopOutsideSearch( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	const unsigned int iSameWopIndex,
	const float fDistanceStart,
	const float fDistanceEnd,
	std::vector<unsigned int>&  vctWopPath, float& fDistance)
{
	std::vector<unsigned int> vctStartAbleWop;
	std::vector<unsigned int> vctEndAbleWop;
	for( auto iter = m_Links.begin( ); m_Links.end( ) != iter; iter++ ) {
		if( iter->second.link_from.wop_id == m_vctWops[iSameWopIndex].wop_id
			&& iter->second.link_from.edge_id == m_vctWops[iSameWopIndex].edge_id ) {
			for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) {
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

	std::vector<unsigned int> vct_prevPath;
	
	for( size_t start_wop = 0; start_wop < vctStartAbleWop.size( ); start_wop++ ) {
		for( size_t end_wop = 0; end_wop < vctEndAbleWop.size( ); end_wop++ ) {

			if( vct_prevPath.size( ) > 0 ) {
				vct_prevPath.clear( );
			}

			int iStartWopIndex = vctStartAbleWop[start_wop];
			int iEndWopIndex = vctEndAbleWop[end_wop];
			if( iStartWopIndex != iEndWopIndex ) {
				float fDistance = 0;			
				if( StartDijkstra( iStartWopIndex, iEndWopIndex, vct_prevPath, fDistance ) < 0 ) {
					continue;
				}
				dist = fDistance
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

	if( vct_prevPath.size() > 0) {
		vct_prevPath.clear();
	}

	StartDijkstra( iMinStartWopIndex, iMinEndWopIndex, vct_prevPath, fDistance );

	vctWopPath.push_back( iSameWopIndex );
	if( iMinStartWopIndex == iMinEndWopIndex ) {

		vctWopPath.push_back( iMinStartWopIndex );

	} else {
		for( size_t i = 0; i < vct_prevPath.size(); i++ ) {
			unsigned int pathIndex = vct_prevPath[i];
			vctWopPath.push_back( pathIndex );
		}
	}
	vctWopPath.push_back( iSameWopIndex );


	fDistance = min_distance + fDistanceStart + fDistanceEnd;
	if( fDistance >= INVALID_DISTANCE ) {
		return -1;
	}
	return 0;
}
int Dijkstra::SameWopOutsideWeightSearch(const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl,
	const unsigned int iSameWopIndex,
	const float fDistanceStart,
	const float fDistanceEnd,
	std::vector<unsigned int>&  vctWopPath, float& fDistance, std::vector<PathSearch_Wop>& m_vctWops_tmp)
{
	std::vector<unsigned int> vctStartAbleWop;
	std::vector<unsigned int> vctEndAbleWop;
	for (auto iter = m_Links.begin(); m_Links.end() != iter; iter++) {
		if (iter->second.link_from.wop_id == m_vctWops_tmp[iSameWopIndex].wop_id
			&& iter->second.link_from.edge_id == m_vctWops_tmp[iSameWopIndex].edge_id) {
			for (size_t wopIndex = 0; wopIndex < m_vctWops_tmp.size(); wopIndex++) {
				if (m_vctWops_tmp[wopIndex].wop_id == iter->second.link_to.wop_id
					&& m_vctWops_tmp[wopIndex].edge_id == iter->second.link_to.edge_id) {
					vctStartAbleWop.push_back(wopIndex);
				}
			}
		}
		if (iter->second.link_to.wop_id == m_vctWops_tmp[iSameWopIndex].wop_id
			&& iter->second.link_to.edge_id == m_vctWops_tmp[iSameWopIndex].edge_id) {
			for (size_t wopIndex = 0; wopIndex < m_vctWops_tmp.size(); wopIndex++) {
				if (m_vctWops_tmp[wopIndex].wop_id == iter->second.link_from.wop_id
					&& m_vctWops_tmp[wopIndex].edge_id == iter->second.link_from.edge_id) {
					vctEndAbleWop.push_back(wopIndex);
				}
			}
		}
	}

	if (vctStartAbleWop.size() == 0 || vctEndAbleWop.size() == 0) {
		return -1;
	}

	float min_distance = INVALID_DISTANCE;
	float dist = 0;
	unsigned int iMinStartWopIndex;
	unsigned int iMinEndWopIndex;

	std::vector<unsigned int> vct_prevPath;

	for (size_t start_wop = 0; start_wop < vctStartAbleWop.size(); start_wop++) {
		for (size_t end_wop = 0; end_wop < vctEndAbleWop.size(); end_wop++) {

			if (vct_prevPath.size() > 0) {
				vct_prevPath.clear();
			}

			int iStartWopIndex = vctStartAbleWop[start_wop];
			int iEndWopIndex = vctEndAbleWop[end_wop];
			if (iStartWopIndex != iEndWopIndex) {
				float fDistance = 0;
				if (StartWeightDijkstra(iStartWopIndex, iEndWopIndex, vct_prevPath, fDistance,m_vctWops_tmp) < 0) {
					continue;
				}
				dist = fDistance
					+ m_vctWops_tmp[iStartWopIndex].distance
					+ m_vctWops_tmp[iEndWopIndex].distance
					+ m_WopDistances[iSameWopIndex][iStartWopIndex]
					+ m_WopDistances[iEndWopIndex][iSameWopIndex];
			}
			else {
				dist = m_vctWops_tmp[iStartWopIndex].distance
					+ m_WopDistances[iSameWopIndex][iStartWopIndex]
					+ m_WopDistances[iEndWopIndex][iSameWopIndex];
			}

			if (dist < min_distance) {
				min_distance = dist;
				iMinStartWopIndex = iStartWopIndex;
				iMinEndWopIndex = iEndWopIndex;
			}
		}
	}

	if (min_distance >= INVALID_DISTANCE) {
		return -1;
	}

	if (vct_prevPath.size() > 0) {
		vct_prevPath.clear();
	}

	StartDijkstra(iMinStartWopIndex, iMinEndWopIndex, vct_prevPath, fDistance);

	vctWopPath.push_back(iSameWopIndex);
	if (iMinStartWopIndex == iMinEndWopIndex) {

		vctWopPath.push_back(iMinStartWopIndex);

	}
	else {
		for (size_t i = 0; i < vct_prevPath.size(); i++) {
			unsigned int pathIndex = vct_prevPath[i];
			vctWopPath.push_back(pathIndex);
		}
	}
	vctWopPath.push_back(iSameWopIndex);


	fDistance = min_distance + fDistanceStart + fDistanceEnd;
	if (fDistance >= INVALID_DISTANCE) {
		return -1;
	}
	return 0;
}
// public
int Dijkstra::GetDijkstraId()
{
	return m_iDijkstraId;
}

int Dijkstra::SetDijkstraId(unsigned int iDijkstraId)
{
	m_iDijkstraId = iDijkstraId;
	return 0;
}

int Dijkstra::SetSearchType(unsigned int iSearchType)
{
	m_iSearchType = iSearchType;
	return 0;
}

int Dijkstra::LoadData( const PathSearch_LayoutData* pLayoutData )
{
	if( !pLayoutData ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData is NULL.";
		return -1;
	}

	for( auto& woptype : pLayoutData->wopList ) {
		PathSearch_Wop wop_type = woptype;
		m_mapWopType[wop_type.wop_id] = wop_type;
	}

	if( m_mapWopType.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData->wopList is empty.";
		return -1;
	}

	if( m_mapEdges.size( ) > 0 ) {
		m_mapEdges.clear( );
	}
	if( m_vctWops.size( ) > 0 ) {
		m_vctWops.clear( );
	}
	if( m_Links.size( ) > 0 ) {
		m_Links.clear( );
	}

	for( auto& edgeInfo : pLayoutData->edgeList ) {
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

	if( m_mapEdges.size( ) == 0 || m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData->edgeList is empty. or pLayoutData->edgeList.wop_list is empty.";
		return -1;
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

	for( size_t startWopIndex = 0; startWopIndex < m_vctWops.size( ); startWopIndex++ ) {

		for( size_t endWopIndex = 0; endWopIndex < m_vctWops.size( ); endWopIndex++ ) {

			if( startWopIndex == endWopIndex ) {
				//m_mapWopDistances[startWopIndex][endWopIndex] = 0;

				//optimize
				m_WopDistances[startWopIndex][endWopIndex] = 0;

			} else {
				//m_mapWopDistances[startWopIndex][endWopIndex] = INVALID_DISTANCE;

				//optimize
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
		m_Links[Link.link_id] = Link;

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

		//m_mapWopDistances[startWopIndex][endWopIndex] = ( float )Link.convert_cost;

		//optimize
		m_WopDistances[startWopIndex][endWopIndex] = (float)Link.convert_cost;
	}

	if( m_Links.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_LoadData: pLayoutData->linkList is empty.";
		return -1;
	}

	return 0;
}

int Dijkstra::SetWops( const PathSearch_Wop* pWops, const unsigned int iLength )
{
	if( pWops == nullptr || iLength == 0 ) {
		loerror("path_search") << "PathSearch_SetWops: pWops is NULL.or iLength == 0.";
		return -1;
	}

	/*
	if( m_mapWopDistances.size( ) > 0 ) {
		m_mapWopDistances.clear( );
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

	if( m_mapWopType.size( ) > 0 ) {
		m_mapWopType.clear( );
	}


	for( size_t startWopIndex = 0; startWopIndex < iLength; startWopIndex++ ) {
		PathSearch_Wop wop = pWops[startWopIndex];
		m_vctWops.push_back( wop );

		m_mapWopType[wop.wop_id] = wop;

		for( size_t endWopIndex = 0; endWopIndex < iLength; endWopIndex++ ) {

			if( startWopIndex == endWopIndex ) {

				//m_mapWopDistances[startWopIndex][endWopIndex] = 0;

				//optimize
				m_WopDistances[startWopIndex][endWopIndex] = 0;

			} else {

				//m_mapWopDistances[startWopIndex][endWopIndex] = INVALID_DISTANCE;

				//optimize
				m_WopDistances[startWopIndex][endWopIndex] = INVALID_DISTANCE;

			}
		}
	}
	return 0;
}

int Dijkstra::SetLinks( const PathSearch_Link* pLinks, const unsigned int iLength )
{
	if( pLinks == nullptr || iLength == 0 ) {
		loerror("path_search") << "PathSearch_SetLinks: pLinks is NULL.or iLength == 0.";
		return -1;
	}
	if( m_vctWops.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_SetLinks: wops number is 0.";
		return -1;
	}
	if( m_Links.size( ) > 0 ) {
		m_Links.clear( );
	}
	for( size_t i = 0; i < iLength; i++ ) {

		unsigned int link_id = pLinks[i].link_id;
		PathSearch_Link link = pLinks[i];
		m_Links[link_id] = link;
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

		//m_mapWopDistances[startWopIndex][endWopIndex] = ( float )link.convert_cost;

		//optimize
		m_WopDistances[startWopIndex][endWopIndex] = (float)link.convert_cost;
	}
	return 0;
}

int Dijkstra::SetEdges( const PathSearch_EdgeInfo* pEdges, const unsigned int iLength )
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

int Dijkstra::StartDijkstra( unsigned int startWopIndex, unsigned int endWopIndex, std::vector<unsigned int>& vctWopPath, float& fDistance )
{
	std::vector<int> vct_save;    // 判断是否已存入该点到S集合中	 0 表示未， 1表示是的
	std::vector<int> vct_prev;
	std::vector<float>vct_distance;

	for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) {

		//float distance = m_mapWopDistances[startWopIndex][wopIndex];
		
		//optimize
		float distance = m_WopDistances[startWopIndex][wopIndex];

		vct_distance.push_back( distance );
		vct_save.push_back( 0 );           // 初始都未用过该点
		if( distance == INVALID_DISTANCE ) {
			vct_prev.push_back( -1 );
		} else {
			vct_prev.push_back( startWopIndex );
		}

	}
	vct_save[startWopIndex] = 1;

	for( size_t i = 2; i <= m_vctWops.size( ); i++ ) {
		float min_distance = INVALID_DISTANCE;
		int current = startWopIndex;                      // 找出当前未使用的点j的dist[j]最小值
		for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) {
			if( ( !vct_save[wopIndex] ) && vct_distance[wopIndex] < min_distance ) {
				current = wopIndex;                             // u保存当前邻接点中距离最小的点的号码 
				min_distance = vct_distance[wopIndex];
			}
		}
		vct_save[current] = 1;

		for( size_t wopIndex = 0; wopIndex < m_vctWops.size( ); wopIndex++ ) 
		{
			/*
			if( ( !vct_save[wopIndex] ) && m_mapWopDistances[current][wopIndex] < INVALID_DISTANCE ) 
			{
				if( vct_distance[current] + m_vctWops[current].distance + m_mapWopDistances[current][wopIndex] < vct_distance[wopIndex] )     //在通过新加入的u点路径找到离v0点更短的路径  
				{
					vct_distance[wopIndex] = vct_distance[current] + m_vctWops[current].distance + m_mapWopDistances[current][wopIndex];    //更新dist 
					vct_prev[wopIndex] = current;                    //记录前驱顶点 
				}
			}
			*/

			//optimize
			if ((!vct_save[wopIndex]) && m_WopDistances[current][wopIndex] < INVALID_DISTANCE)
			{
				if (vct_distance[current] + m_vctWops[current].distance + m_WopDistances[current][wopIndex] < vct_distance[wopIndex])     //在通过新加入的u点路径找到离v0点更短的路径  
				{
					vct_distance[wopIndex] = vct_distance[current] + m_vctWops[current].distance + m_WopDistances[current][wopIndex];    //更新dist 
					vct_prev[wopIndex] = current;                    //记录前驱顶点 
				}
			}
		}
	}

	fDistance = vct_distance[endWopIndex];

	if( fDistance >= INVALID_DISTANCE ) {
		return -1;
	}

	int currentIndex = endWopIndex;
	vctWopPath.push_back( endWopIndex );
	while( true ) {
		int prevIndex = vct_prev[currentIndex];
		vctWopPath.push_back( prevIndex );
		currentIndex = prevIndex;
		if( prevIndex == startWopIndex ) {
			break;
		}
	}

	return 0;
}
int Dijkstra::StartWeightDijkstra(unsigned int startWopIndex, unsigned int endWopIndex, std::vector<unsigned int>& vctWopPath, float& fDistance, std::vector<PathSearch_Wop>& m_vctWops_tmp)
{
	std::vector<int> vct_save;    // 判断是否已存入该点到S集合中	 0 表示未， 1表示是的
	std::vector<int> vct_prev;
	std::vector<float>vct_distance;

	for (size_t wopIndex = 0; wopIndex < m_vctWops_tmp.size(); wopIndex++) {

		//float distance = m_mapWopDistances[startWopIndex][wopIndex];

		//optimize
		float distance = m_WopDistances[startWopIndex][wopIndex];

		vct_distance.push_back(distance);
		vct_save.push_back(0);           // 初始都未用过该点
		if (distance == INVALID_DISTANCE) {
			vct_prev.push_back(-1);
		}
		else {
			vct_prev.push_back(startWopIndex);
		}

	}
	vct_save[startWopIndex] = 1;

	for (size_t i = 2; i <= m_vctWops_tmp.size(); i++) {
		float min_distance = INVALID_DISTANCE;
		int current = startWopIndex;                      // 找出当前未使用的点j的dist[j]最小值
		for (size_t wopIndex = 0; wopIndex < m_vctWops_tmp.size(); wopIndex++) {
			if ((!vct_save[wopIndex]) && vct_distance[wopIndex] < min_distance) {
				current = wopIndex;                             // u保存当前邻接点中距离最小的点的号码 
				min_distance = vct_distance[wopIndex];
			}
		}
		vct_save[current] = 1;

		for (size_t wopIndex = 0; wopIndex < m_vctWops_tmp.size(); wopIndex++) 
		{
			/*
			if ((!vct_save[wopIndex]) && m_mapWopDistances[current][wopIndex] < INVALID_DISTANCE) 
			{
				if (vct_distance[current] + m_vctWops_tmp[current].distance + m_mapWopDistances[current][wopIndex] < vct_distance[wopIndex])     //在通过新加入的u点路径找到离v0点更短的路径  
				{
					vct_distance[wopIndex] = vct_distance[current] + m_vctWops_tmp[current].distance + m_mapWopDistances[current][wopIndex];    //更新dist 
					vct_prev[wopIndex] = current;                    //记录前驱顶点 
				}
			}
			*/

			//optimize
			if ((!vct_save[wopIndex]) && m_WopDistances[current][wopIndex] < INVALID_DISTANCE)
			{
				if (vct_distance[current] + m_vctWops_tmp[current].distance + m_WopDistances[current][wopIndex] < vct_distance[wopIndex])     //在通过新加入的u点路径找到离v0点更短的路径  
				{
					vct_distance[wopIndex] = vct_distance[current] + m_vctWops_tmp[current].distance + m_WopDistances[current][wopIndex];    //更新dist 
					vct_prev[wopIndex] = current;                    //记录前驱顶点 
				}
			}
		}
	}

	fDistance = vct_distance[endWopIndex];

	if (fDistance >= INVALID_DISTANCE) {
		return -1;
	}

	int currentIndex = endWopIndex;
	vctWopPath.push_back(endWopIndex);
	while (true) {
		int prevIndex = vct_prev[currentIndex];
		vctWopPath.push_back(prevIndex);
		currentIndex = prevIndex;
		if (prevIndex == startWopIndex) {
			break;
		}
	}

	return 0;
}
int Dijkstra::GetPath(const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance)
{
	/*
	if( m_mapWopDistances.size( ) == 0 || m_vctWops.size( ) == 0 ) {
		return -1;
	}
	*/

	//optimize
	if (m_WopDistancesLength == 0 || m_vctWops.size() == 0) {
		return -1;
	}

	int iStartWopIndex = 0;
	int iEndWopIndex = 0;
	float fDistanceStart = 0;
	float fDistanceEnd = 0;

	
	if (GetStartAndEndWop(stStartUpl, stEndUpl, iStartWopIndex, iEndWopIndex, fDistanceStart, fDistanceEnd) < 0) {
		return -1;
	}

	PathSearch_Wop 	stStartWop = m_vctWops[iStartWopIndex];
	PathSearch_Wop  stEndWop = m_vctWops[iEndWopIndex];
	std::vector<unsigned int /*wop_index*/>  vctWopPath;		
	// 不同wop
	if ( IsSameWop( stStartWop, stEndWop ) < 0) {
		if (StartDijkstra( iStartWopIndex, iEndWopIndex, vctWopPath, *fDistance ) < 0) {
			return -1;
		}
		*fDistance = *fDistance + fDistanceStart + fDistanceEnd;
	}
	// 相同的wop	
	else if (IsSameWop(stStartWop, stEndWop) == 0) {
		std::vector<PathSearch_Wop> tmp;
		if (SameWopPathSearch(stStartUpl, stEndUpl, iStartWopIndex, fDistanceStart, fDistanceEnd, vctWopPath, *fDistance,tmp) < 0) {
			return -1;
		}
	}
	if( *fDistance >= INVALID_DISTANCE) {
		return -1;
	}

	if( vctWopPath.size( ) > 0 ) {
		*path = new PathSearch_Wop[vctWopPath.size( )];
	}

	int offset = 0;
	for( int i = vctWopPath.size() -1; i >= 0; i-- ) {
		int WopIndex = vctWopPath[i];
		PathSearch_Wop wop = m_vctWops[WopIndex];
		memcpy( *path + offset, &wop, sizeof( PathSearch_Wop ) );
		offset++;

	}
	*iLength = vctWopPath.size( );	

	return 0;
}
int Dijkstra::GetWeightPath(const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance, std::vector<PathSearch_Wop>& m_vctWops_tmp)
{
	/*
	if (m_mapWopDistances.size() == 0 || m_vctWops.size() == 0) {
		return -1;
	}
	*/

	//optimize
	if (m_WopDistancesLength == 0 || m_vctWops.size() == 0) {
		return -1;
	}

	int iStartWopIndex = 0;
	int iEndWopIndex = 0;
	float fDistanceStart = 0;
	float fDistanceEnd = 0;

	
	if (GetStartAndEndWeightWop(stStartUpl, stEndUpl, iStartWopIndex, iEndWopIndex, fDistanceStart, fDistanceEnd, m_vctWops_tmp) < 0){
		return -1;
	}
	PathSearch_Wop 	stStartWop = m_vctWops_tmp[iStartWopIndex];
	PathSearch_Wop  stEndWop = m_vctWops_tmp[iEndWopIndex];
	// 不同wop
	std::vector<unsigned int /*wop_index*/>  vctWopPath;
	if (IsSameWop(stStartWop, stEndWop) < 0) {
		if (StartWeightDijkstra(iStartWopIndex, iEndWopIndex, vctWopPath, *fDistance, m_vctWops_tmp) < 0) {
			return -1;
		}
		*fDistance = *fDistance + fDistanceStart + fDistanceEnd;
	}
	// 相同的wop
	else if (IsSameWop(stStartWop, stEndWop) == 0) {
		if (SameWopPathSearch(stStartUpl, stEndUpl, iStartWopIndex, fDistanceStart, fDistanceEnd, vctWopPath, *fDistance,m_vctWops_tmp) < 0) {
			return -1;
		}
	}
	if (*fDistance >= INVALID_DISTANCE) {
		return -1;
	}

	if (vctWopPath.size() > 0) {
		*path = new PathSearch_Wop[vctWopPath.size()];
	}

	int offset = 0;
	for (int i = vctWopPath.size() - 1; i >= 0; i--) {
		int WopIndex = vctWopPath[i];
		PathSearch_Wop wop = m_vctWops_tmp[WopIndex];
		memcpy(*path + offset, &wop, sizeof(PathSearch_Wop));
		offset++;

	}

	*iLength = vctWopPath.size();
	return 0;
}
int Dijkstra::GetPathByUpl( const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance )
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

	if( vctStartUpl.size( ) == 0 || vctEndUpl.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_GetPath: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}
	path_info.clear();
	// 如果已经指定wop,则需要判断该upl使用指定的wop是否有效，如果没有指定wop,则需要找出该upl能使用的有效的wop，组成upl列表
	if( bExistStartWop == true ) {
		bool bFindStartWop = false;
		for( auto& start_wop : vctStartUpl ) {
			if( start_wop.wop_id == stStartUpl.wop_id && start_wop.edge_id == stStartUpl.edge_id ) {
				vctStartUpl.clear( );
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
	if( vctStartUpl.size( ) == 0 || vctEndUpl.size( ) == 0 ) {
		loerror("path_search") << "PathSearch_GetPath: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}

	// 遍历，找到最优的带wop的起始和结束upl
	float min_dis = INVALID_DISTANCE;
	{	
		int Wopsum = vctStartUpl.size()*vctEndUpl.size();//需要getpath的次数
		std::vector<std::shared_ptr<std::thread>>vct_th;
		//std::vector<Path_Info> path_info;
		Path_Info tmp_pathinfo;
		int i = 0;
			for (auto &startUpl : vctStartUpl) {
				for (auto& endUpl : vctEndUpl) {
					tmp_pathinfo.stStartUpl = startUpl;
					tmp_pathinfo.stEndUpl = endUpl;
#if _WIN32
					std::thread th(std::bind(&Dijkstra::Dijktra_thread, this, std::placeholders::_1), tmp_pathinfo);
#else
					std::thread th( std::bind( &Dijkstra::Dijktra_thread, this, std::placeholders::_1), (void*)&tmp_pathinfo);
#endif					
					vct_th.push_back(std::make_shared<std::thread>(std::move(th)));
					i++;
				}
		}
			for (auto iter : vct_th){			
			iter->join();			
		}

		for (auto iter : path_info)
			if (iter.ireturn == 0 && iter.fDistance+iter.iLength*1 < min_dis){
				min_dis = iter.fDistance+ iter.iLength * 1;
			}
		if (min_dis >= INVALID_DISTANCE) {
			loerror("path_search") << "PathSearch_GetPath: path is not valid.";
			return -1;
		}
		else{
			for (auto iter : path_info)
				if (min_dis == iter.fDistance+iter.iLength * 1){
					*path = iter.path;
					*iLength = iter.iLength;
					*fDistance = iter.fDistance;
					break;
				}
		}
		
	}
	//auto cpu_cnt = std::thread::hardware_concurrency();//获取cpu个数  

	/*PathSearch_Wop* tmp_path = nullptr;
	int tmp_length = 0;
	float tmp_distance = 0;
	float min_distance = INVALID_DISTANCE;
	PathSearch_UPL	best_startUpl;
	PathSearch_UPL  best_endUpl;
	int Wopsum = vctStartUpl.size()*vctEndUpl.size();
	// 遍历，找到最优的带wop的起始和结束upl
	for( auto startUpl : vctStartUpl ) {
		for( auto endUpl : vctEndUpl ) {
			if( GetPath( startUpl, endUpl, &tmp_path, &tmp_length, &tmp_distance ) < 0 ) {
				Releasepath( tmp_path );
				break;
			}
			Releasepath( tmp_path );
			if( tmp_distance < min_distance ) {
				min_distance = tmp_distance;
				best_startUpl = startUpl;
				best_endUpl = endUpl;
			}
		}
	}*/
	// 最优的最短距离
	// 将最优的路径反馈
	//if( GetPath( best_startUpl, best_endUpl, path, iLength, fDistance ) < 0 ) {
	//	loerror("path_search") << "PathSearch_GetPath: path is not valid.";
	//	return -1;
	//}
	return 0;
}


#if _WIN32
void Dijkstra::Dijktra_thread(Path_Info &path_iata){
	path_iata.ireturn = GetPath(path_iata.stStartUpl, path_iata.stEndUpl, &path_iata.path, &path_iata.iLength, &path_iata.fDistance);
	std::lock_guard<std::recursive_mutex> guard_lock(_lock_data);
	path_info.push_back(path_iata);
}
#else
void Dijkstra::Dijktra_thread(void* param) {
	Path_Info path_iata = *(Path_Info*)param;
	path_iata.ireturn = GetPath(path_iata.stStartUpl, path_iata.stEndUpl, &path_iata.path, &path_iata.iLength, &path_iata.fDistance);
	std::lock_guard<std::recursive_mutex> guard_lock(_lock_data);
	path_info.push_back(path_iata);
}
#endif

int Dijkstra::GetPathByPos( const PathSearch_Position stStartPos, const PathSearch_Position stEndPos, PathSearch_Wop** path, int* iLength, float* fDistance )
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
	if( vctStartUpl.size( ) == 0 || vctEndUpl.size( ) == 0 ) {
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
			if (GetPath(startUpl, endUpl, &tmp_path, &tmp_length, &tmp_distance) < 0) {
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
	if( GetPath( best_startUpl, best_endUpl, path, iLength, fDistance) < 0 ) {
		loerror("path_search") << "PathSearch_GetPathByPos: path is not valid.";
		return -1;
	}

	return 0;
}

int Dijkstra::GetPathBySPosAndEUpl( const PathSearch_Position stStartPos, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance )
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
		for( auto end_wop : vctEndUpl ) {
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
			if( GetPath( startUpl, endUpl, &tmp_path, &tmp_length, &tmp_distance) < 0 ) {
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
	if( GetPath( best_startUpl, best_endUpl, path, iLength, fDistance) < 0 ) {
		loerror("path_search") << "PathSearch_GetPathBySPosAndEUpl: path is not valid.";
		return -1;
	}

	return 0;
}

void Dijkstra::Releasepath( PathSearch_Wop * path )
{
	if( path ) {
		delete[] path;
		path = nullptr;
	}
}

int Dijkstra::GetPathAtWeight(const PathSearch_Upl stStartUpl, const PathSearch_Upl stEndUpl, PathSearch_Wop** path, int* iLength, float* fDistance, const int* pWeightEdgesId, const unsigned int iWeightEdgesNum)
{
	if (pWeightEdgesId == nullptr || iWeightEdgesNum == 0) {
		loerror("path_search") << "PathSearch_SetEdges: pWeightEdgesId is NULL. or iWeightEdgesNum == 0.";
	}
	// 首先通过给定UPL获取改upl上有效的带wop的upl列表
	std::vector<PathSearch_UPL> vctStartUpl;
	std::vector<PathSearch_UPL>	vctEndUpl;
	DataCalculate::Upl2UplList(stStartUpl, m_mapWopType, m_mapEdges, vctStartUpl);
	DataCalculate::Upl2UplList(stEndUpl, m_mapWopType, m_mapEdges, vctEndUpl);

	// 需要判断起始和结束upl是否已经指定了wop
	bool bExistStartWop = false;
	bool bExistEndWop = false;
	for (auto wop : m_vctWops) {
		if (stStartUpl.edge_id == wop.edge_id && stStartUpl.wop_id == wop.wop_id) {
			bExistStartWop = true;
		}
		if (stEndUpl.edge_id == wop.edge_id && stEndUpl.wop_id == wop.wop_id) {
			bExistEndWop = true;
		}
	}

	if (vctStartUpl.size() == 0 || vctEndUpl.size() == 0) {
		loerror("path_search") << "PathSearch_GetPath: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}

	// 如果已经指定wop,则需要判断该upl使用指定的wop是否有效，如果没有指定wop,则需要找出该upl能使用的有效的wop，组成upl列表
	if (bExistStartWop == true) {
		bool bFindStartWop = false;
		for (auto start_wop : vctStartUpl) {
			if (start_wop.wop_id == stStartUpl.wop_id && start_wop.edge_id == stStartUpl.edge_id) {
				vctStartUpl.clear();
				vctStartUpl.push_back(stStartUpl);
				bFindStartWop = true;
				break;
			}
		}

		if (bFindStartWop == false) {
			loerror("path_search") << "PathSearch_GetPath: start Upl's specified wop is not valid.";
			return -1;
		}
	}

	if (bExistEndWop == true) {
		bool bFindEndWop = false;
		for (auto end_wop : vctEndUpl) {
			if (end_wop.wop_id == stEndUpl.wop_id && end_wop.edge_id == stEndUpl.edge_id) {
				vctEndUpl.clear();
				vctEndUpl.push_back(stEndUpl);
				bFindEndWop = true;
				break;
			}
		}

		if (bFindEndWop == false) {
			loerror("path_search") << "PathSearch_GetPath: end Upl's specified wop is not valid.";
			return -1;
		}
	}

	// 能够使用的带wop的起始UPL都存在
	if (vctStartUpl.size() == 0 || vctEndUpl.size() == 0) {
		loerror("path_search") << "PathSearch_GetPath: valid start Upl number is 0. or valid end Upl number is 0.";
		return -1;
	}
	std::vector<PathSearch_Wop> m_vctWops_tmp;
	ChangeWopValue(pWeightEdgesId, iWeightEdgesNum, m_vctWops_tmp);

	PathSearch_Wop* tmp_path = nullptr;
	int tmp_length = 0;
	float tmp_distance = 0;
	float min_distance = INVALID_DISTANCE;
	PathSearch_UPL	best_startUpl;
	PathSearch_UPL  best_endUpl;
	// 遍历，找到最优的带wop的起始和结束upl
	for (auto startUpl : vctStartUpl) {
		for (auto endUpl : vctEndUpl) {
			if (GetWeightPath(startUpl, endUpl, &tmp_path, &tmp_length, &tmp_distance, m_vctWops_tmp) < 0) {
				//Releasepath(tmp_path);
				break;
			}
			if (tmp_path){
				Releasepath(tmp_path);
			}
			tmp_path = nullptr;
			if (tmp_distance < min_distance) {
				min_distance = tmp_distance;
				best_startUpl = startUpl;
				best_endUpl = endUpl;
			}
		}
	}

	// 最优的最短距离
	if (min_distance >= INVALID_DISTANCE) {
		loerror("path_search") << "PathSearch_GetPath: path is not valid.";
		//RestoreWopValue(pWeightEdgesId, iWeightEdgesNum);
		return -1;
	}

	// 将最优的路径反馈
	if (GetWeightPath(best_startUpl, best_endUpl, path, iLength, fDistance, m_vctWops_tmp) < 0) {
		loerror("path_search") << "PathSearch_GetPath: path is not valid.";
		//RestoreWopValue(pWeightEdgesId, iWeightEdgesNum);
		return -1;
	}

	//RestoreWopValue(pWeightEdgesId, iWeightEdgesNum);
	return 0;
	
}

int Dijkstra::ChangeWopValue(const int* pWeightEdgesId, const unsigned int iWeightEdgesNum, std::vector<PathSearch_Wop>& m_vctWops_tmp)
{
	if (pWeightEdgesId == nullptr || iWeightEdgesNum == 0) {
		loerror("path_search") << "PathSearch_SetEdges: pWeightEdgesId is NULL. or iWeightEdgesNum == 0.";
	}
	m_vctWops_tmp.assign(m_vctWops.begin(), m_vctWops.end());
	for (size_t i = 0; i < iWeightEdgesNum; i++) {
		for (size_t wopIndex = 0; wopIndex < m_vctWops_tmp.size(); wopIndex++) {
			if (pWeightEdgesId[i] == m_vctWops_tmp[wopIndex].edge_id) {
				m_vctWops_tmp[wopIndex].distance += INVALID_DISTANCE;
				//m_vctWops[wopIndex].distance += INVALID_DISTANCE;
			}
		}
	}
	return 0;
}

int Dijkstra::RestoreWopValue(const int* pWeightEdgesId, const unsigned int iWeightEdgesNum)
{
	if (pWeightEdgesId == nullptr || iWeightEdgesNum == 0) {
		loerror("path_search") << "PathSearch_SetEdges: pWeightEdgesId is NULL. or iWeightEdgesNum == 0.";
	}

	for (size_t i = 0; i < iWeightEdgesNum; i++) {
		for (size_t wopIndex = 0; wopIndex < m_vctWops.size(); wopIndex++) {
			if (pWeightEdgesId[i] == m_vctWops[wopIndex].edge_id) {
				m_vctWops[wopIndex].distance -= INVALID_DISTANCE;
			}
		}
	}
	return 0;
}
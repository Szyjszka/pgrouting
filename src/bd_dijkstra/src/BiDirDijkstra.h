/******************************************************************************
* $Id$
*
* Project:  pgRouting bdsp and bdastar algorithms
* Purpose:
* Author:   Razequl Islam <ziboncsedu@gmail.com>
*

******************************************************************************
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies of this Software or works derived from this Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.

*****************************************************************************/

#ifndef BIDIRDIJKSTRA_H
#define BIDIRDIJKSTRA_H

#include <vector>
#include <map>
#include <queue>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "bdsp.h"

#define INF 1e15



typedef std::vector<long> LongVector;
typedef std::vector<LongVector> VectorOfLongVector;
//typedef std::pair<int, bool> PIB;
typedef std::pair<double, int> PDI;
//typedef std::pair<double, std::vector<int> > PDVI;
/*
typedef struct edge 
{
	int id;
	int source;
	int target;
	double cost;
	double reverse_cost;
} edge_t;

typedef struct path_element 
{
	int vertex_id;
	int edge_id;
	double cost;
}path_element_t;
*/

typedef struct{
	int par_Node;
	int par_Edge;
	int par_EdgeIndex;
}PARENT_PATH;

typedef struct{
	int NodeID;
	std::vector<int> Connected_Nodes;
	std::vector<int> Connected_Edges_Index;
}GraphNodeInfo;

struct GraphEdgeInfo
{
public:
	int EdgeID;
	int EdgeIndex;
	int Direction;
	int64_t osm_id;
	double Cost;
	double ReverseCost;
	int StartNode;
	int EndNode;
	bool incOrder;
    int Shortcut;
};

struct ShortcutInfo
{
    int ShortcutID;
    int shA;
    int shB;
};

typedef std::vector<GraphEdgeInfo> GraphEdgeVector;
typedef std::map<long, GraphEdgeVector> ShortcutsTable;
typedef std::vector<ShortcutInfo> ShortcutInfos;
typedef std::map<long,LongVector> Long2LongVectorMap;
typedef std::map<long,long> Long2LongMap;
typedef std::vector<GraphNodeInfo*> GraphNodeVector;


class BiDirDijkstra
{
public:
	BiDirDijkstra(void);
	~BiDirDijkstra(void);
	
	int bidir_dijkstra(edge_t *edges, unsigned int edge_count, int maxNode, int start_vertex, int end_vertex,
		path_element_t **path, int *path_count, char **err_msg);
	

private:
	bool construct_graph(edge_t *edges, int edge_count, int maxNode);
	void fconstruct_path(int node_id);
	void rconstruct_path(int node_id);
        void unwrapShortcut(int edgeId, int edgeIndex, int start_node);
    	bool addEdge(const edge& edgeIn);
        void addShortcutsIndexes();
	bool connectEdge(GraphEdgeInfo& firstEdge, GraphEdgeInfo& secondEdge, bool bIsStartNodeSame);
	void init();
	void initall(int maxNode);
	void deleteall();
    void explore(int cur_node, double cur_cost, std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > &que);
    void exploreReverse(int cur_node, double cur_cost, std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > &que);

    inline double getcost(const int node_id)
    {
        return(m_pFCost[node_id]);
    }

    inline double getcostReverse(const int node_id)
    {
        return(m_pRCost[node_id]);
    }
    /*
        Set the forward or reverse cost list depending on dir (1 for forward search and -1 for reverse search.
    */
    inline void setcost(int node_id, double c)
    {
        m_pFCost[node_id] = c;
    }
    inline void setcostReverse(int node_id, double c)
    {
        m_pRCost[node_id] = c;
    }

    inline void setparent(const int node_id, const int parnode, const int paredge, const int paredgeindex)
    {
            m_pFParent[node_id].par_Node = parnode;
            m_pFParent[node_id].par_Edge = paredge;
            m_pFParent[node_id].par_EdgeIndex = paredgeindex;
    }
    inline void setparentReverse(const int node_id, const int parnode, const int paredge, const int paredgeindex)
    {
            m_pRParent[node_id].par_Node = parnode;
            m_pRParent[node_id].par_Edge = paredge;
            m_pRParent[node_id].par_EdgeIndex = paredgeindex;
    }
    void unwrapShortcutR(int edgeID, int edgeIndex, int start_node);

private:
	GraphEdgeVector m_vecEdgeVector;
	ShortcutsTable m_shortcutsTable;
	Long2LongMap m_mapEdgeId2Index;
	Long2LongVectorMap m_mapNodeId2Edge;
    	Long2LongMap m_mapShortcut2Id;
	GraphNodeVector m_vecNodeVector;
    	ShortcutInfos m_shortcutsInfos;

    //For every index stores it's shortcuts
    struct ShortcutIndexes{
        uint32_t shAIndex, shBIndex;
    };

    typedef std::vector<ShortcutIndexes> ShortcutVec;

    ShortcutVec m_ShortcutVec;

	int max_node_id;
	int max_edge_id;
	int m_lStartNodeId;
	int m_lEndNodeId;

	double m_MinCost;
	int m_MidNode;
	std::vector <path_element_t> m_vecPath;
	PARENT_PATH *m_pFParent;
	PARENT_PATH *m_pRParent;
	double *m_pFCost;
	double *m_pRCost;
};

#endif

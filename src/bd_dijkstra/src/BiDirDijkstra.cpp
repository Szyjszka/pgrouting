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

#ifdef __MINGW32__
#include <winsock2.h>
#include <windows.h>
#endif

#include "BiDirDijkstra.h"
#include "algorithm_time_measure.hpp"

#undef DEBUG
#define DEBUG

#ifdef DEBUG
#include <stdio.h>
static FILE *dbg;
#define DBG(format, arg...) \
    dbg = fopen("/tmp/sew-debug", "a"); \
    if (dbg) { \
        fprintf(dbg, format,  ## arg); \
        fclose(dbg); \
    }
#else
#define DBG(format, arg...) do { ; } while (0)
#endif


BiDirDijkstra::BiDirDijkstra(void)
{
}

BiDirDijkstra::~BiDirDijkstra(void)
{
}

void BiDirDijkstra::init()
{
	//max_edge_id = 0;
	//max_node_id = 0;
	
}

/*
	Initialization and allocation of memories.
*/
void BiDirDijkstra::initall(int maxNode)
{
	int i;
	m_vecPath.clear();
//    DBG("BiDirDijkstra::initall: allocating m_pFParent, m_pRParent maxNode: %d\n", maxNode+1);
	m_pFParent = new PARENT_PATH[maxNode + 1];
	m_pRParent = new PARENT_PATH[maxNode + 1];
//    DBG("BiDirDijkstra::initall: allocated m_pFParent, m_pRParent\n");

//    DBG("BiDirDijkstra::initall: allocating m_pFCost, m_pRCost maxNode: %d\n", maxNode+1);
	m_pFCost = new double[maxNode + 1];
	m_pRCost = new double[maxNode + 1];
//    DBG("BiDirDijkstra::initall: allocated m_pFCost, m_pRCost\n");

	for(i = 0; i <= maxNode; i++)
	{
		m_pFParent[i].par_Node = -2;
		m_pRParent[i].par_Node = -2;
		m_pFCost[i] = INF;
		m_pRCost[i] = INF;
		
	}
	m_MinCost = INF;
	m_MidNode = -1;

//    DBG("BiDirDijkstra::initall: m_vecNodeVector.reserve(%d)\n", maxNode + 1);
    // reserve space for nodes
    m_vecNodeVector.reserve(maxNode + 1);
//    DBG("           space reserved!\n");
}

/*
	Delete the allocated memories to avoid memory leak.
*/
void BiDirDijkstra::deleteall()
{
	std::vector<GraphNodeInfo*>::iterator it;
	for(it  = m_vecNodeVector.begin(); it != m_vecNodeVector.end(); it++){
		delete *it;
	}
	m_vecNodeVector.clear();
	delete [] m_pFParent;
	delete [] m_pRParent;
	delete [] m_pFCost;
    delete [] m_pRCost;
}

void BiDirDijkstra::exploreReverse(int cur_node, double cur_cost, std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > &que)
{
//    if(m_ReverseStall[cur_node])
//        return;
        int i;
        // Number of connected edges
        int con_edge = m_vecNodeVector[cur_node]->Connected_Edges_Index.size();
        double edge_cost;

        for(i = 0; i < con_edge; ++i)
        {
            int edge_index = m_vecNodeVector[cur_node]->Connected_Edges_Index[i];
            // Get the edge from the edge list.
            GraphEdgeInfo edge = m_vecEdgeVector[edge_index];
            // Get the connected node
            int new_node = m_vecNodeVector[cur_node]->Connected_Nodes[i];
            edge_cost = edge.Cost + cur_cost;
            const bool goodOrder= cur_node == edge.StartNode ? edge.incOrder : !edge.incOrder;
            // Check if the direction is valid for exploration
            if(goodOrder)
            {
                // Check if the current edge gives better result
                if(edge_cost < getcostReverse(new_node))
                {
                    // explore the node, and push it in the queue
                    setcostReverse(new_node, edge_cost);
                    setparentReverse(new_node, cur_node, edge.EdgeID, edge.EdgeIndex);
                    que.push(std::make_pair(edge_cost, new_node));
//                    m_ReverseStall[cur_node] = false;

                    // Update the minimum cost found so far.
                    if(getcostReverse(new_node) + getcost(new_node) < m_MinCost)
                    {
                        m_MinCost = getcostReverse(new_node) + getcost(new_node);
                        m_MidNode = new_node;
                    }
                }
            }
//            else
//            {
//                if((getcost(new_node) + EPSILON_PLUS_1*edge.Cost) < getcost(cur_node))
//                {
//                    m_ReverseStall[cur_node] = true;
//                    setcost(cur_node, (getcost(new_node) + EPSILON_PLUS_1*edge.Cost));
//                    return;
//                }
//            }
        }
}

/*
    This is the main exploration module. The parameter dir indicates whether the exploration will be in forward or reverser direction. The reference to the corresponding
    que is also passed as parameter que. The current node and the current costs are also available as parameter.
*/

void BiDirDijkstra::explore(int cur_node, double cur_cost, std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > &que)
{
//    if(m_ForwardStall[cur_node])
//        return;
    int i;
    // Number of connected edges
    int con_edge = m_vecNodeVector[cur_node]->Connected_Edges_Index.size();
    double edge_cost;

    for(i = 0; i < con_edge; ++i)
    {
        int edge_index = m_vecNodeVector[cur_node]->Connected_Edges_Index[i];
        // Get the edge from the edge list.
        GraphEdgeInfo edge = m_vecEdgeVector[edge_index];
        // Get the connected node
        int new_node = m_vecNodeVector[cur_node]->Connected_Nodes[i];
        edge_cost = cur_cost + edge.Cost;
        const bool goodOrder= cur_node == edge.StartNode ? edge.incOrder : !edge.incOrder;

        // Check if the direction is valid for exploration
        if(goodOrder)
        {
            // Check if the current edge gives better result
            if(edge_cost < getcost(new_node))
            {
                // explore the node, and push it in the queue
                setcost(new_node, edge_cost);
                setparent(new_node, cur_node, edge.EdgeID, edge.EdgeIndex);
                que.push(std::make_pair(edge_cost, new_node));
//                m_ForwardStall[new_node] = false;
                // Update the minimum cost found so far.
                if(getcost(new_node) + getcostReverse(new_node) < m_MinCost)
                {
                    m_MinCost = getcost(new_node) + getcostReverse(new_node);
                    m_MidNode = new_node;
                }
            }
        }
//        else
//        {
//            if((getcost(new_node) + EPSILON_PLUS_1*edge.Cost) < getcost(cur_node))
//            {
//                m_ForwardStall[cur_node] = true;
//                setcost(cur_node, (getcost(new_node) + EPSILON_PLUS_1*edge.Cost));
//                return;
//            }
//        }
    }
}



/*
	Reconstruct path for forward search. It is like normal dijkstra. The parent array contains the parent of the current node and there is a -1 in the source.
	So one need to recurse upto the source and then add the current node and edge to the list.
*/
void BiDirDijkstra::fconstruct_path(int node_id)
{
	if(m_pFParent[node_id].par_Node == -1)
		return;
    fconstruct_path(m_pFParent[node_id].par_Node);

//    DBG("FORWARD\n");

    unwrapShortcut(m_pFParent[node_id].par_Edge, m_pFParent[node_id].par_EdgeIndex, node_id);
//    DBG("Czy jest skrot z %d : %d %d\n",osm_id, m_shortcutsTable.find(osm_id) != m_shortcutsTable.end(), m_vecEdgeVector[m_mapEdgeId2Index[edge_ID]].Shortcut);

}

/*
	Reconstruct path for the reverse search. In this case the subsequent node is stored in the parent and the target contains a -1. So one need to add the node
	and edge to the list and then recurse through the parent upto hitting a -1.
*/

void BiDirDijkstra::rconstruct_path(int node_id)
{
	if(m_pRParent[node_id].par_Node == -1)
    {
		return;
    }

    unwrapShortcutR(m_pRParent[node_id].par_Edge,  m_pRParent[node_id].par_EdgeIndex, node_id);
    rconstruct_path(m_pRParent[node_id].par_Node);
}

void BiDirDijkstra::unwrapShortcut(int edgeID, int edgeIndex,int start_node)
{
    ShortcutInfo shortcutInfo = m_shortcutsInfos[edgeIndex];
    GraphEdgeInfo edgeInfo = m_vecEdgeVector[edgeIndex];


    int Aindex = m_ShortcutVec[edgeIndex].shAIndex;
    int Bindex = m_ShortcutVec[edgeIndex].shBIndex;
    GraphEdgeInfo AedgeInfo = m_vecEdgeVector[Aindex];
    GraphEdgeInfo BedgeInfo = m_vecEdgeVector[Bindex];

    GraphEdgeInfo& firstEdge = start_node == AedgeInfo.StartNode || start_node == AedgeInfo.EndNode ?
                AedgeInfo : BedgeInfo;
    GraphEdgeInfo& secondEdge = start_node != BedgeInfo.StartNode && start_node != BedgeInfo.EndNode ?
                 BedgeInfo : AedgeInfo;

    if(shortcutInfo.shA == -1 && shortcutInfo.shB == -1)
    {
        path_element_t pt;
        pt.vertex_id = edgeInfo.EndNode == start_node ? edgeInfo.StartNode : edgeInfo.EndNode;
        pt.edge_id = edgeID;
        pt.cost = edgeInfo.Cost;
        m_vecPath.push_back(pt);
    }
    else
    {
        unwrapShortcut(secondEdge.EdgeID, secondEdge.EdgeIndex, secondEdge.EndNode == firstEdge.StartNode ||  secondEdge.EndNode == firstEdge.EndNode ?
                                               secondEdge.EndNode : secondEdge.StartNode);
        unwrapShortcut(firstEdge.EdgeID, firstEdge.EdgeIndex, firstEdge.StartNode == start_node ? firstEdge.StartNode : firstEdge.EndNode);
    }
}

void BiDirDijkstra::unwrapShortcutR(int edgeID, int edgeIndex, int start_node)
{
    ShortcutInfo shortcutInfo = m_shortcutsInfos[edgeIndex];
    GraphEdgeInfo edgeInfo = m_vecEdgeVector[edgeIndex];

    int Aindex = m_ShortcutVec[edgeIndex].shAIndex;
    int Bindex = m_ShortcutVec[edgeIndex].shBIndex;
    GraphEdgeInfo AedgeInfo = m_vecEdgeVector[Aindex];
    GraphEdgeInfo BedgeInfo = m_vecEdgeVector[Bindex];

    GraphEdgeInfo& firstEdge = start_node == AedgeInfo.StartNode || start_node == AedgeInfo.EndNode ?
                AedgeInfo : BedgeInfo;
    GraphEdgeInfo& secondEdge = start_node != BedgeInfo.StartNode && start_node != BedgeInfo.EndNode ?
                 BedgeInfo : AedgeInfo;

    if(shortcutInfo.shA == -1 && shortcutInfo.shB == -1)
    {
        path_element_t pt;
        pt.vertex_id = edgeInfo.StartNode == start_node ? edgeInfo.StartNode : edgeInfo.EndNode;
        pt.edge_id = edgeID;
        pt.cost = edgeInfo.Cost;
        m_vecPath.push_back(pt);
    }
    else
    {
        unwrapShortcut(firstEdge.EdgeID, firstEdge.EdgeIndex, firstEdge.StartNode == start_node ? firstEdge.EndNode : firstEdge.StartNode);
        unwrapShortcut(secondEdge.EdgeID, secondEdge.EdgeIndex, secondEdge.EndNode == firstEdge.StartNode ||  secondEdge.EndNode == firstEdge.EndNode ?
                                               secondEdge.StartNode : secondEdge.EndNode);
    }
}


/* 
	This is the entry function that the wrappers should call. Most of the parameters are trivial. maxNode refers to Maximum
	node id. As we run node based exploration cost, parent etc will be based on maximam node id.
*/


int BiDirDijkstra::bidir_dijkstra(edge_t *edges, unsigned int edge_count, int maxNode, int start_vertex, int end_vertex,
				path_element_t **path, int *path_count, char **err_msg)
{
	max_node_id = maxNode;
	max_edge_id = -1;
	
	// Allocate memory for local storage like cost and parent holder
//    DBG("calling initall(maxNode=%d)\n", maxNode);
	initall(maxNode);

	// construct the graph from the edge list, i.e. populate node and edge data structures
//    DBG("Calling construct_graph\n");
	construct_graph(edges, edge_count, maxNode);
	

	//int nodeCount = m_vecNodeVector.size();
//	DBG("Setting up std::priority_queue\n");
	std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > fque;
	std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI> > rque;
	
//    DBG("calling m_vecPath.clear()\n");
	m_vecPath.clear();

	// Initialize the forward search
	m_pFParent[start_vertex].par_Node = -1;
	m_pFParent[start_vertex].par_Edge = -1;
	m_pFCost[start_vertex] = 0.0;
	fque.push(std::make_pair(0.0, start_vertex));

	// Initialize the reverse search
	m_pRParent[end_vertex].par_Node = -1;
	m_pRParent[end_vertex].par_Edge = -1;
	m_pRCost[end_vertex] = 0.0;
	rque.push(std::make_pair(0.0, end_vertex));
//    DBG("Z %d do %d \n", start_vertex+1, end_vertex+1);
    m_ForwardStall.resize(m_vecNodeVector.size(), false);
    m_ReverseStall.resize(m_vecNodeVector.size(), false);
    *path = (path_element_t *) malloc(sizeof(path_element_t) * (30000 + 1));
    RouterCH::AlgorithmTimeMeasure atm;
    atm.startMeasurement();
	int i;
	// int new_node;
	int cur_node;
	// int dir;

/*
	The main loop. The algorithm is as follows:
	1. IF the sum of the current minimum of both heap is greater than so far found path, we cannot get any better, so break the loop.
	2. IF the reverse heap minimum is lower than the forward heap minimum, explore from reverse direction.
	3. ELSE explore from the forward directtion.
*/

    PDI fTop = fque.top();
    PDI rTop = rque.top();
    while(!fque.empty() || !rque.empty())
    {
        if(!fque.empty())
            fTop = fque.top();
        if(!rque.empty())
            rTop = rque.top();
        if(rTop.first > m_MinCost && fTop.first > m_MinCost) //We are done, there is no path with lower cost
        {
			break;
        }
        if(((rTop.first < fTop.first) || fque.empty()) && !rque.empty()) // Explore from reverse queue
		{
            cur_node = rTop.second;
            rque.pop();
            exploreReverse(cur_node, rTop.first, rque);
		}
		else                        // Explore from forward queue
		{
            cur_node = fTop.second;
            fque.pop();
            explore(cur_node, fTop.first, fque);
		}
	}

/*
	Path reconstruction part. m_MidNode is the joining point where two searches meet to make a shortest path. It is updated in explore.
	If it contains -1, then no path is found. Other wise we have a shortest path and that is reconstructed in the m_vecPath.
*/ 
	if(m_MidNode == -1)
	{
		*err_msg = (char *)"Path Not Found";
		deleteall();
		return -1;
	}
	else
	{
		// reconstruct path from forward search
		fconstruct_path(m_MidNode);
		// reconstruct path from backward search
		rconstruct_path(m_MidNode);

		// insert the last row in the path trace (having edge_id = -1 and cost = 0.0)
		path_element_t pelement;
		pelement.vertex_id = end_vertex;
		pelement.edge_id = -1;
		pelement.cost = 0.0;
		m_vecPath.push_back(pelement);

		// Transfer data path to path_element_t format and allocate memory and populate the pointer

//        DBG("BiDirDijkstra::bidir_dijkstra: allocating path m_vecPath.size=%d\n", m_vecPath.size() + 1);
		*path_count = m_vecPath.size();
//        DBG("BiDirDijkstra::bidir_dijkstra: allocated path\n");

		for(i = 0; i < *path_count; i++)
		{
			(*path)[i].vertex_id = m_vecPath[i].vertex_id;
			(*path)[i].edge_id = m_vecPath[i].edge_id;
			(*path)[i].cost = m_vecPath[i].cost;
        }

        atm.stopMeasurement();
        DBG("%f\n", atm.getMeanTime())
	}
//    DBG("calling deleteall\n");
	deleteall();
//    DBG("back from deleteall\n");
	return 0;
}

/*
	Populate the member variables of the class using the edge list. Basically there is a node list and an edge list. Each node contains the list of adjacent nodes and 
	corresponding edge indices from edge list that connect this node with the adjacent nodes.
*/

bool BiDirDijkstra::construct_graph(edge_t* edges, int edge_count, int maxNode)
{
	int i;

	/*
	// Create a dummy node
//    DBG("Create a dummy node\n");
	GraphNodeInfo nodeInfo;
//    DBG("calling nodeInfo.Connected_Edges_Index.clear\n");
	nodeInfo.Connected_Edges_Index.clear();
//    DBG("calling nodeInfo.Connected_Nodes.clear\n");
	nodeInfo.Connected_Nodes.clear();
	*/

	// Insert the dummy node into the node list. This acts as place holder. Also change the nodeId so that nodeId and node index in the vector are same.
	// There may be some nodes here that does not appear in the edge list. The size of the list is upto maxNode which is equal to maximum node id.
//    DBG("m_vecNodeVector.push_back for 0 - %d\n", maxNode);
	for(i = 0; i <= maxNode; i++)
	{
		// Create a dummy node
		GraphNodeInfo* nodeInfo = new GraphNodeInfo();
		nodeInfo->Connected_Edges_Index.clear();
		nodeInfo->Connected_Nodes.clear();
		
		nodeInfo->NodeID = i;
		m_vecNodeVector.push_back(nodeInfo);
	}

	// Process each edge from the edge list and update the member data structures accordingly.
//    DBG("reserving space for m_vecEdgeVector.reserve(%d)\n", edge_count);
    m_vecEdgeVector.reserve(edge_count);
//    DBG("calling addEdge in a loop\n");
	for(i = 0; i < edge_count; i++)
	{
//        DBG("KOSZT: %f \n", edges[i].cost);
		addEdge(edges[i]);
	}

    addShortcutsIndexes();

	return true;
}

/*
	Process the edge and populate the member nodelist and edgelist. The nodelist already contains upto maxNode dummy entries with nodeId same as index. Now the
	connectivity information needs to be updated.
*/


void BiDirDijkstra::addShortcutsIndexes()
{
    m_ShortcutVec.resize(m_vecEdgeVector.size());

    for(unsigned int i = 0; i < m_vecEdgeVector.size(); ++i)
    {
        uint32_t edgeIndex = m_vecEdgeVector[i].EdgeIndex;
        ShortcutInfo shortcutInfo = m_shortcutsInfos[edgeIndex];
        int Aid = m_mapShortcut2Id[shortcutInfo.shA];
        int Bid = m_mapShortcut2Id[shortcutInfo.shB];
        int Aindex = m_mapEdgeId2Index[Aid];
        int Bindex = m_mapEdgeId2Index[Bid];

        m_ShortcutVec[edgeIndex].shAIndex = Aindex;
        m_ShortcutVec[edgeIndex].shBIndex = Bindex;
    }

}

bool BiDirDijkstra::addEdge(const edge_t& edgeIn)
{
	// long lTest;

	// Check if the edge is already processed.
//TODO Michal check
//    Long2LongMap::iterator itMap = m_mapEdgeId2Index.find(edgeIn.id);
//    if(itMap != m_mapEdgeId2Index.end() && edgeIn.shortcut < 1)
//        return false;


	// Create a GraphEdgeInfo using the information of the current edge
	GraphEdgeInfo newEdge;
	newEdge.EdgeID = edgeIn.id;
    newEdge.osm_id = edgeIn.osm_id;
	newEdge.EdgeIndex = m_vecEdgeVector.size();	
	newEdge.StartNode = edgeIn.source;
	newEdge.EndNode = edgeIn.target;
	newEdge.Cost = edgeIn.cost;
    newEdge.ReverseCost = edgeIn.cost;/*edgeIn.reverse_cost;*/
    newEdge.incOrder = edgeIn.incOrder;
    newEdge.Shortcut = edgeIn.shortcut;
//    DBG("INC %d DEC %d", Inc, Dec);
	// Set the direction. If both cost and reverse cost has positive value the edge is bidirectional and direction field is 0. If cost is positive and reverse cost
	// negative then the edge is unidirectional with direction = 1 (goes from source to target) otherwise it is unidirectional with direction = -1 (goes from target
	// to source). Another way of creating unidirectional edge is assigning big values in cost or reverse_cost. In that case the direction is still zero and this case
	// is handled in the algorithm automatically.
	if(newEdge.Cost >= 0.0 && newEdge.ReverseCost >= 0)
	{
		newEdge.Direction = 0;
	}
	else if(newEdge.Cost >= 0.0)
	{
		newEdge.Direction = 1;
	}
	else
	{
		newEdge.Direction = -1;
	}

	// Update max_edge_id
	if(edgeIn.id > max_edge_id)
	{
		max_edge_id = edgeIn.id;
	}

	//Update max_node_id
	if(newEdge.StartNode > max_node_id)
	{
//TODO Michal check
        	max_node_id = newEdge.StartNode;
	}
	if(newEdge.EndNode > max_node_id)
	{
//TODO Michal check
        	max_node_id = newEdge.EdgeIndex;
	}

    ShortcutInfo info;
    info.shA = edgeIn.shA;
    info.shB = edgeIn.shB;
    info.ShortcutID = edgeIn.shortcutID;
    m_shortcutsInfos.push_back(info);

    m_mapShortcut2Id[info.ShortcutID] = newEdge.EdgeID;
//    DBG("mapa %d %d \n", info.ShortcutID, newEdge.EdgeID);

    if(newEdge.Shortcut < 1)
    {
        // update connectivity information for the start node.
        m_vecNodeVector[newEdge.StartNode]->Connected_Nodes.push_back(newEdge.EndNode);
        m_vecNodeVector[newEdge.StartNode]->Connected_Edges_Index.push_back(newEdge.EdgeIndex);

        // update connectivity information for the start node.
        m_vecNodeVector[newEdge.EndNode]->Connected_Nodes.push_back(newEdge.StartNode);
        m_vecNodeVector[newEdge.EndNode]->Connected_Edges_Index.push_back(newEdge.EdgeIndex);


        //Adding edge to the list
        m_mapEdgeId2Index.insert(std::make_pair(newEdge.EdgeID, m_vecEdgeVector.size()));
        m_vecEdgeVector.push_back(newEdge);
    }
	return true;
}

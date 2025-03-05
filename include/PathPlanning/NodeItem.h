#ifndef NODE_ITEM_H
#define	NODE_ITEM_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "Mapping.h"
#include "NodeList.h"

constexpr uint8_t LIST_LENGTH_2 = 24; // TODO !!! Mapping::Max_Vertex;
constexpr uint8_t INVALID_VERTEX_ID = 255;

//using namespace Mapping;

class NodeItem
{
public:
    uint32 currentCost = 0;
    uint32 parentCost = 0;
    t_vertexID parentID = INVALID_VERTEX_ID;
    t_vertexID currentID = INVALID_VERTEX_ID;
    
    void Set(t_vertexID parentID, t_vertexID currentID, uint32 parentCost);
    //void NodeNew(NodeItem * newNode);
    void SetParent(t_vertexID parentID, uint32 parentCost);
    uint32 GetCost();
    uint32 GetF();
    int8 FCmp(NodeItem node);
    uint32 CostWillBe();
    void ListGetPossibleNode(std::vector<NodeItem> &list);

    //NodeItem() : currentCost(0), parentCost(0), parentID(INVALID_VERTEX_ID), currentID(INVALID_VERTEX_ID) {}
    
//ListNode *NodeListSortedAddDichotomic(ListNode * listNode, NodeItem *node);
};

//void NodeNew(NodeItem * newNode);

#endif

#ifndef NODE_ITEM_H
#define	NODE_ITEM_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "Mapping.h"

constexpr uint8_t INVALID_VERTEX_ID = 255;

//using namespace Mapping;

class NodeItem
{
public:
    uint32_t currentCost = 0;
    uint32_t parentCost = 0;
    t_vertexID parentID = INVALID_VERTEX_ID;
    t_vertexID currentID = INVALID_VERTEX_ID;

    void Set(t_vertexID parentID, t_vertexID currentID, uint32_t parentCost);
    //void NodeNew(NodeItem * newNode);
    void SetParent(t_vertexID parentID, uint32_t parentCost);
    uint32_t GetCost();
    uint32_t GetF();
    int8_t FCmp(NodeItem node);
    uint32_t CostWillBe();
    void ListGetPossibleNode(std::vector<NodeItem> &list);

    //NodeItem() : currentCost(0), parentCost(0), parentID(INVALID_VERTEX_ID), currentID(INVALID_VERTEX_ID) {}
    
//ListNode *NodeListSortedAddDichotomic(ListNode * listNode, NodeItem *node);
};

//void NodeNew(NodeItem * newNode);

#endif

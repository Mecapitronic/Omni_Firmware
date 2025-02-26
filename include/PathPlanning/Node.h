#ifndef NODE_H
#define	NODE_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "NodeList.h"
#include "Mapping.h"

//using namespace Mapping;

namespace Node
{
    void NodeSet(t_node * node, t_vertexID parentID, t_vertexID currentID, uint32 parentCost);
    void NodeNew(t_node * newNode);
    void NodeSetParent(t_node * node, t_vertexID parentID, uint32 parentCost);
    uint32 NodeGetCost(t_node node);
    uint32 NodeF(t_node node);
    int8 NodeFCmp(t_node p1, t_node p2);
    uint32 NodeCostWillBe(t_node node);
    void NodeListGetPossibleNode(t_node list[], t_node node);

//ListNode *NodeListSortedAddDichotomic(ListNode * listNode, t_node *node);
}
#endif

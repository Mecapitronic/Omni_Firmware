#ifndef NODE_LIST_H
#define NODE_LIST_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "Mapping.h"
#include "Node.h"

constexpr size_t LIST_LENGTH = 16; // TODO !!! Mapping::Max_Vertex;
constexpr size_t INVALID_VERTEX_ID = 255;

namespace NodeList
{
    /****************************************************************************************
     * Prototypes
     ****************************************************************************************/
    void ListAddFirst(std::array<t_node, LIST_LENGTH> &list, t_node data);
    void ListAddEnd(std::array<t_node, LIST_LENGTH> &list, t_node data);
    uint32 ListLength(std::array<t_node, LIST_LENGTH> &list);
    void ListInsertSorted(std::array<t_node, LIST_LENGTH> &list, t_node data);
    void ListGetFirstItem(std::array<t_node, LIST_LENGTH> &list, t_node *data);
    int ListIsDataExist(std::array<t_node, LIST_LENGTH> &list, t_node data);
    void ListFreeALL(std::array<t_node, LIST_LENGTH> &list);
    void ListVertexIDInit(std::array<t_vertexID, LIST_LENGTH> &list);

#ifdef SERIAL_PRINT
    void ListPrint(std::array<t_node, LIST_LENGTH> &list, String str = "");
    void ListVertexPrint(std::array<t_vertexID, LIST_LENGTH> &list);
#endif
}
#endif

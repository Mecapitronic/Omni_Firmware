#ifndef NODE_LIST_H
#define NODE_LIST_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "Mapping.h"
#include "Node.h"

#define LIST_LENGTH 24 // TODO !!! Max_Vertex
#define INVALID_VERTEX_ID  255

namespace NodeList
{
    /****************************************************************************************
     * Protoypes
     ****************************************************************************************/
    void ListAddFirst(t_node list[], t_node data);
    void ListAddEnd(t_node list[], t_node data);
    uint32 ListLength(t_node list[]);
    void ListInsertSorted(t_node list[], t_node data);
    void ListGetFirstItem(t_node list[], t_node *data);
    int ListIsDataExist(t_node list[], t_node data);
    void ListFreeALL(t_node list[]);
    void ListVertexIDInit(t_vertexID list[]);

#ifdef SERIAL_PRINT
    void ListPrint(t_node list[], String str = "");
    void ListVertexPrint(t_vertexID list[]);
#endif
}
#endif

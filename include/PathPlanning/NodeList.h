#ifndef NODE_LIST_H
#define NODE_LIST_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "NodeItem.h"

namespace NodeList
{
    constexpr size_t LIST_LENGTH = 24; // TODO !!! Mapping::Max_Vertex;
    constexpr size_t INVALID_VERTEX_ID = 255;

    /****************************************************************************************
     * Prototypes
     ****************************************************************************************/
    template <typename T>
    void ListAddFirst(std::vector<T> &list, T data);

    template <typename T>
    void ListAddEnd(std::vector<T> &list, T data);
    
    template <typename T>
    uint32_t ListLength(std::vector<T> &list);
    
    template <typename T>
    void ListInsertSorted(std::vector<T> &list, T data);
    
    template <typename T>
    void ListGetFirstItem(std::vector<T> &list, T *data);
    
    template <typename T>
    int ListIsDataExist(std::vector<T> &list, T data);
    
    //void ListFreeALL(std::vector<T> &list);
    
    void ListVertexIDInit(std::vector<t_vertexID> &list);

#ifdef SERIAL_PRINT
    template <typename T>
    void ListPrint(std::vector<T> &list, String str = "");

    void ListVertexPrint(std::vector<t_vertexID> &list);
#endif
}
#endif

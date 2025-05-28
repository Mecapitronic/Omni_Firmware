#ifndef PATH_FINDING_H
#define PATH_FINDING_H

#include "Mapping.h"
#include "NodeItem.h"
#include "Structure.h"

using namespace Mapping;

// Timeout in millisecond
// #define Set_Timeout_PF(time)    do{timeout_pf = current_time + time;}while(0)
// #define TIMEOUT_PF              (current_time > timeout_pf)

// Condition d'arrét anticipé du calcul de PathFinding
// #define PATH_FINDING_STOP           (TIMEOUT_PF)
// Durée limite du calcul de PathFinding en millisecond
// #define PATH_FINDING_TIME_LIMIT     (10000)

namespace PathFinding
{
    // t_vertexID *solution;
    constexpr uint8_t LIST_LENGTH = Mapping::Max_Vertex;
    constexpr bool PRINT_PF = false;

    extern std::vector<t_vertexID> solution;

    boolean PathFinding(int16_t start_x, int16_t start_y, t_vertexID end_vertex_ID);
    void PathFindingAddToOpen(std::vector<NodeItem> &list);

    void ListPrint(std::vector<NodeItem> &list, String str);
    void NodePrint(NodeItem node, String str);
    void ListVertexPrint(std::vector<t_vertexID> &list, String str);

    template <typename T, typename Pred>
    typename std::vector<T>::iterator insert_sorted(std::vector<T> &vec,
                                                    T const &item,
                                                    Pred pred);

} // namespace PathFinding
#endif

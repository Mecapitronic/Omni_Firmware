#ifndef PATH_FINDING_H
#define	PATH_FINDING_H

#include "Structure.h"
#include "Mapping.h"
#include "NodeList.h"
#include "NodeItem.h"

using namespace Mapping;

// Timeout in millisecond
//#define Set_Timeout_PF(time)    do{timeout_pf = current_time + time;}while(0)
//#define TIMEOUT_PF              (current_time > timeout_pf)

// Condition d'arrét anticipé du calcul de PathFinding
//#define PATH_FINDING_STOP           (TIMEOUT_PF)
// Durée limite du calcul de PathFinding en millisecond
//#define PATH_FINDING_TIME_LIMIT     (10000)

namespace PathFinding
{
    //t_vertexID *solution;
    constexpr uint8_t LIST_LENGTH = Mapping::Max_Vertex;

    boolean Path_Planning();
    boolean Path_Finding();
    void PathFindingAddToOpen(std::vector<NodeItem> &list);
}
#endif


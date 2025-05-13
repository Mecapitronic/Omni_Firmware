/****************************************************************************************
* Includes
****************************************************************************************/
#include "PathPlanning/PathFinding.h"

namespace PathFinding
{
/****************************************************************************************
* Variables
****************************************************************************************/
std::vector <NodeItem> open;
std::vector <NodeItem> close;
std::vector <t_vertexID> solution;

uint32_t timeout_pf = 0;

/****************************************************************************************
 * Path planning
 ****************************************************************************************/
boolean Path_Planning(void)
{
  boolean result;
  //Update_Obstacles(BRAKE_DISTANCE);

  Update_Passability_Graph();
  result = Path_Finding();
  return result;
}

/****************************************************************************************
 Fonction : Calculate the shortest path between the start point and the end point
 ****************************************************************************************/
boolean Path_Finding()
{
	std::vector <NodeItem> listPossible;
	NodeItem startNode;
	NodeItem best;
	//Set_Timeout_PF(PATH_FINDING_TIME_LIMIT);

	// Initialisation des list
	open.clear();
	open.reserve(32);
	close.clear();
	close.reserve(32);
	listPossible.clear();
	listPossible.reserve(32);

	// Création du premier noeud de départ
	startNode.currentID = 0;// start to vertex 0
	// On ajoute le noeud à la liste open
	open.push_back(startNode);
	//ListAddFirst(open, startNode);
	int iteration = 0;

	while (open.size() > 0)
	{		
    	Debugger::WaitForAvailableSteps();
		//on récupére le premier noeud de la liste cad le meilleur
		best = open.front();
		pop_front(open);
		
		if(PRINT_PF)
		{
			Printer::println("Iteration N° ", iteration++);
			NodePrint(best, "best");
		}
		//ListGetFirstItem(open, &best);

		// on regarde si le noeud est le noeud final OU si il y a condition d'arrét anticipé
		if ((Is_Equal_Vertex(best.currentID, Get_End_Vertex()) ))//|| (PATH_FINDING_STOP))
		{
			int i = 0;
			int pos = 0;
			NodeItem dummy = NodeItem();
			//ListVertexIDInit(solution);
			solution.clear();
			solution.reserve(32);
			//NodeNew(&dummy);
			solution.push_back(best.currentID);
			//solution[i++] = best.currentID;
			dummy.currentID = best.parentID;
			
			//A partir du noeud de fin,
			while (dummy.currentID != INVALID_VERTEX_ID)
			{
				auto it = std::find_if(close.begin(), close.end(), [&dummy](const NodeItem &node) { return Mapping::Is_Equal_Vertex(node.currentID, dummy.currentID); });
				//if(it != close.end()) // we assume it's in !
					pos = it - close.begin();
				//pos = ListIsDataExist(close, dummy);
				dummy = close.at(pos);
				solution.insert(solution.begin(), dummy.currentID);
				//solution[i++] = dummy.currentID;
				dummy.currentID = dummy.parentID;
			}
			if(PRINT_PF)
			{
			ListVertexPrint(solution, "solution");
			}
			return true;
		}
		// on ajoute le noeud évalué à la liste close
		//ListAddEnd(close, best);
		close.push_back(best);

		// on récupére dans une liste tous les noeuds voisins autour du best
		listPossible.clear();
		best.ListGetPossibleNode(listPossible);
		
		if(PRINT_PF)
		{
		printf("\n");
		ListPrint(open, "open");
		ListPrint(close, "close");
		ListPrint(listPossible, "listPossible");
		}

		// on ajoute cette liste de nodes à open pour que les noeuds soient évalués
		PathFindingAddToOpen(listPossible);
		
		if(PRINT_PF)
		{
		ListPrint(open , "open");
		}

	}
	// No path found
	if(PRINT_PF)
	{
	printf("No solution founded !\n");
	}
	return false;
}

/****************************************************************************************
* Fonction : Add a list of nodes to the open list if needed
****************************************************************************************/
void PathFindingAddToOpen(std::vector<NodeItem> &list)
{
	int i = 0;
	int pos = 0;
	NodeItem dummy = NodeItem();
	// on parcourt la liste des noeuds
	//while (list[i].currentID != INVALID_VERTEX_ID)
	for (size_t i = 0; i < list.size(); i++)
	{
		dummy = list.at(i);
		// on vérifie que le noeud n'existe pas déjà dans open
		//pos = ListIsDataExist(open, list[i]);
		auto itOpen = std::find_if(open.begin(), open.end(), [&dummy](const NodeItem &node) { return Mapping::Is_Equal_Vertex(node.currentID, dummy.currentID); });
		pos = itOpen - open.begin();
		if (itOpen == open.end())
		{
			// on vérifie que le noeud n'existe pas déjà dans close
			//pos = ListIsDataExist(close, list[i]);
			auto itClose = std::find_if(close.begin(), close.end(), [&dummy](const NodeItem &node) { return Mapping::Is_Equal_Vertex(node.currentID, dummy.currentID); });
			pos = itClose - close.begin();			
			if (itClose == close.end())
			{
				// On ajoute le noeud de façon trié dans la liste open
				//ListInsertSorted(open, list[i]);
				//dummyOpen = list[i];
				insert_sorted(open, dummy, [](const NodeItem &node1, const NodeItem &node2) { return node2.FCmp(node1) > 0; });
			}
			else
			{
				// le noeud existe déjà dans close
				// on regarde si son cout sera meilleur
				// sinon on affecte le nouveau parent
				NodeItem node = NodeItem();
				//NodeNew(&node);
				node = close.at(pos);
				if (list[i].CostWillBe() < close[pos].GetCost())
				{
					// we just need to modify the parent, close is not sorted
					close[pos].SetParent(list[i].parentID, list[i].parentCost);
				}
			}
		}
		else
		{
			// le noeud existe déjà dans open
			// on regarde si son cout sera meilleur
			// sinon on affecte le nouveau parent
			if (list[i].CostWillBe() < open[pos].GetCost())
			{
				// we need to remove it from open then put it back sorted because it may not be at the right place
				open.erase(itOpen);
				dummy.SetParent(list[i].parentID, list[i].parentCost);
				insert_sorted(open, dummy, [](const NodeItem &node1, const NodeItem &node2) { return node2.FCmp(node1) > 0; });
			}
		}
	}
}

void ListPrint(std::vector<NodeItem> &list, String str)
{
    int i = 0;
    printf("-------\n");
    printf("ListNode : %s \n", str);
    for (i = 0; i < list.size(); i++)
    {
        //if (list[i].currentID == INVALID_VERTEX_ID)
        //    break;
        printf("%d] ", i);
        // printf ("Liste 0x%X - NodeItem 0x%X ->",current,current->data);
        printf("Vtx ID:%d ", list[i].currentID);
        printf("Cost %ld ", list[i].currentCost);
        printf("P %ld ", list[i].parentID);
        printf("PCost %ld ", list[i].parentCost);
        // printf("Heuristic %f , Cost %f " , NodeF(current->data) , NodeGetCost(current->data));
        printf("\n");
    }
}

void NodePrint(NodeItem node, String str)
{
    printf("-------\n");
    printf("Node : %s \n", str);
        // printf ("Liste 0x%X - NodeItem 0x%X ->",current,current->data);
        printf("Vtx ID:%d ", node.currentID);
        printf("Cost %ld ", node.currentCost);
        printf("P %ld ", node.parentID);
        printf("PCost %ld ", node.parentCost);
        // printf("Heuristic %f , Cost %f " , NodeF(current->data) , NodeGetCost(current->data));
        printf("\n");
}

void ListVertexPrint(std::vector<t_vertexID> &list, String str)
{
    int i = 0;
    printf("-------\n");
    printf("ListVertex : %s \n", str);		
    for (i = 0; i < list.size(); i++)
    //while (list[i] != INVALID_VERTEX_ID)
    {
        printf("%d] ", i);
        printf("Vertex id:%d", list[i]);
		printf("\n");
	}
}

template <typename T, typename Pred>
typename std::vector<T>::iterator insert_sorted(std::vector<T> &vec, T const &item, Pred pred)
{
    return vec.insert(
        std::upper_bound(vec.begin(), vec.end(), item, pred),
        item);
}

}
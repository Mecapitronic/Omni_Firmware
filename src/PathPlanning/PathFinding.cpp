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
std::vector <t_vertexID> solution = {INVALID_VERTEX_ID}; // /!\ init of first index only !
std::vector <t_vertexID> solutionInverse;

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

	while (open.size() > 0)
	{
		//on récupére le premier noeud de la liste cad le meilleur
		best = open.front();
		pop_front(open);
		//ListGetFirstItem(open, &best);

		// on regarde si le noeud est le noeud final OU si il y a condition d'arrét anticipé
		if ((Is_Equal_Vertex(best.currentID, Get_End_Vertex()) ))//|| (PATH_FINDING_STOP))
		{
			int i = 0;
			int pos = 0;
			NodeItem dummy = NodeItem();
			//ListVertexIDInit(solutionInverse);
			solutionInverse.clear();
			solutionInverse.reserve(32);
			//NodeNew(&dummy);
			solutionInverse[i++] = best.currentID;
			dummy.currentID = best.parentID;
			
			//A partir du noeud de fin,
			while (dummy.currentID != INVALID_VERTEX_ID)
			{

				pos = NodeList::ListIsDataExist(close, dummy);
				dummy = close[pos];
				solutionInverse[i++] = dummy.currentID;
				dummy.currentID = dummy.parentID;
			}
			int j=0;
			solution.clear();
			solution.reserve(32);
			//ListVertexIDInit(solution);
			for (i = 0; i < LIST_LENGTH; i++)
			{
				if(solutionInverse[LIST_LENGTH-1-i] != INVALID_VERTEX_ID)
					if(solutionInverse[LIST_LENGTH-1-i] != 0)
						solution[j++] = solutionInverse[LIST_LENGTH-1-i];
			}
#ifdef SERIAL_PRINT
			NodeList::ListPrint(open , "open");
			NodeList::ListPrint(close , "close");
			NodeList::ListVertexPrint(solution);
#endif
			return true;
		}
		// on ajoute le noeud évalué à la liste close
		//ListAddEnd(close, best);
		close.push_back(best);

		// on récupére dans une liste tous les noeuds voisins autour du best
		listPossible.clear();
		listPossible.reserve(32);
		best.ListGetPossibleNode(listPossible);

#ifdef SERIAL_PRINT
		printf("\n");
		NodeList::ListPrint(open, "open");
		NodeList::ListPrint(close, "close");
		NodeList::ListPrint(listPossible, "best");
#endif

		// on ajoute cette liste de nodes à open pour que les noeuds soient évalués
		PathFindingAddToOpen(listPossible);
	}
	// No path found
#ifdef SERIAL_PRINT
	printf("No solution founded !\n");
#endif
	return false;


}

/****************************************************************************************
* Fonction : Add a list of nodes to the open list if needed
****************************************************************************************/
void PathFindingAddToOpen(std::vector<NodeItem> &list)
{
	int i = 0;
	int pos = 0;
	// on parcourt la liste des noeuds
	while (list[i].currentID != INVALID_VERTEX_ID)
	{
		// on vérifie que le noeud n'existe pas déjé dans open
		pos = NodeList::ListIsDataExist(open, list[i]);
		if (pos == -1)
		{
			// on vérifie que le noeud n'existe pas déjé dans close
			pos = NodeList::ListIsDataExist(close, list[i]);
			if (pos == -1)
			{
				// On ajoute le noeud de faéon trié dans la liste open
				NodeList::ListInsertSorted(open, list[i]);
			}
			else
			{
				// le noeud existe déjé dans close
				// on regarde si son cout sera meilleur
				// sinon on affecte le nouveau parent
				NodeItem node = NodeItem();
				//NodeNew(&node);
				node = close[pos];
				if (list[i].CostWillBe() < close[pos].GetCost())
				{
					close[pos].SetParent(list[i].parentID, list[i].parentCost);
				}
			}
		}
		else
		{
			// le noeud existe déjé dans open
			// on regarde si son cout sera meilleur
			// sinon on affecte le nouveau parent
			if (list[i].CostWillBe() < open[pos].GetCost())
			{
				open[pos].SetParent(list[i].parentID, list[i].parentCost);
			}
		}

		i++;
	}
}

}
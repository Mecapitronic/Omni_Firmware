/****************************************************************************************
* Includes
****************************************************************************************/
#include "PathPlanning/Path_Finding.h"

namespace PathFinding
{
/****************************************************************************************
* Variables
****************************************************************************************/
t_node open[LIST_LENGTH];
t_node close[LIST_LENGTH];
t_vertexID solution[LIST_LENGTH] = {INVALID_VERTEX_ID}; // /!\ init of first index only !
t_vertexID solutionInverse[LIST_LENGTH];

uint32 timeout_pf = 0;

/****************************************************************************************
 * Path planning
 ****************************************************************************************/
boolean Path_Planning(void)
{
  boolean result;
  //LED = !LED;
  //Update_Obstacles(BRAKE_DISTANCE);
  Update_Passability_Graph();
  result = Path_Finding();
  //LED = OFF;
  return result;
}

/****************************************************************************************
 Fonction : Calculate the shortest path between the start point and the end point
 ****************************************************************************************/
boolean Path_Finding()
{
	t_node listPossible[LIST_LENGTH];
	t_node startNode;
	t_node best;
	//Set_Timeout_PF(PATH_FINDING_TIME_LIMIT);

	// Initialisation des list
	ListFreeALL(open);
	ListFreeALL(close);
	ListFreeALL(listPossible);

	NodeNew(&startNode);
	NodeNew(&best);

	// Création du premier noeud de départ
	NodeSet(&startNode, INVALID_VERTEX_ID, 0, 0);  // start to vertex 0
	// On ajoute le noeud à la liste open
	ListAddFirst(open, startNode);

	while (ListLength(open) > 0)
	{
		//on récupére le premier noeud de la liste cad le meilleur
		ListGetFirstItem(open, &best);

		// on regarde si le noeud est le noeud final OU si il y a condition d'arrét anticipé
		if ((Is_Equal_Vertex(best.currentID, Get_End_Vertex()) ))//|| (PATH_FINDING_STOP))
		{
			int i = 0;
			int pos = 0;
			t_node dummy;
			ListVertexIDInit(solutionInverse);
			NodeNew(&dummy);
			solutionInverse[i++] = best.currentID;
			dummy.currentID = best.parentID;
			
			//A partir du noeud de fin,
			while (dummy.currentID != INVALID_VERTEX_ID)
			{
				pos = ListIsDataExist(close, dummy);
				dummy = close[pos];
				solutionInverse[i++] = dummy.currentID;
				dummy.currentID = dummy.parentID;
			}
			int j=0;
			ListVertexIDInit(solution);
			for (i = 0; i < LIST_LENGTH; i++)
			{
				if(solutionInverse[LIST_LENGTH-1-i] != INVALID_VERTEX_ID)
					if(solutionInverse[LIST_LENGTH-1-i] != 0)
						solution[j++] = solutionInverse[LIST_LENGTH-1-i];
			}
#ifdef SERIAL_PRINT
			ListPrint(open , "open");
			ListPrint(close , "close");
			ListVertexPrint(solution);
#endif
			return TRUE;
		}
		// on ajoute le noeud évalué à la liste close
		ListAddEnd(close, best);

		// on récupére dans une liste tous les noeuds voisins autour du best
		ListFreeALL(listPossible);
		NodeListGetPossibleNode(listPossible, best);

#ifdef SERIAL_PRINT
		printf("\n");
		ListPrint(open, "open");
		ListPrint(close, "close");
		ListPrint(listPossible, "best");
#endif

		// on ajoute cette liste de nodes à open pour que les noeuds soient évalués
		PathFindingAddToOpen(listPossible);
	}
	// No path found
#ifdef SERIAL_PRINT
	printf("No solution founded !\n");
#endif
	return FALSE;


}

/****************************************************************************************
* Fonction : Add a list of nodes to the open list if needed
****************************************************************************************/
void PathFindingAddToOpen(t_node list[])
{
	int i = 0;
	int pos = 0;
	// on parcourt la liste des noeuds
	while (list[i].currentID != INVALID_VERTEX_ID)
	{
		// on vérifie que le noeud n'existe pas déjé dans open
		pos = ListIsDataExist(open, list[i]);
		if (pos == -1)
		{
			// on vérifie que le noeud n'existe pas déjé dans close
			pos = ListIsDataExist(close, list[i]);
			if (pos == -1)
			{
				// On ajoute le noeud de faéon trié dans la liste open
				ListInsertSorted(open, list[i]);
			}
			else
			{
				// le noeud existe déjé dans close
				// on regarde si son cout sera meilleur
				// sinon on affecte le nouveau parent
				t_node node;
				NodeNew(&node);
				node = close[pos];
				if (NodeCostWillBe(list[i]) < NodeGetCost(close[pos]))
				{
					NodeSetParent(&close[pos], list[i].parentID, list[i].parentCost);
				}
			}
		}
		else
		{
			// le noeud existe déjé dans open
			// on regarde si son cout sera meilleur
			// sinon on affecte le nouveau parent
			if (Node::NodeCostWillBe(list[i]) < Node::NodeGetCost(open[pos]))
			{
				Node::NodeSetParent(&open[pos], list[i].parentID, list[i].parentCost);
			}
		}

		i++;
	}
}

}
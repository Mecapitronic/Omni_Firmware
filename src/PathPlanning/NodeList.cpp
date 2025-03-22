/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "PathPlanning/NodeList.h"

namespace NodeList
{
	/****************************************************************************************
	 * Fonction : Rajoute en tete de la liste pList l'element data.
	 ****************************************************************************************/
	template <typename T>
	void ListAddFirst(std::vector<T> &list, T data)
	{
		int i;
		if (list[0].currentID == INVALID_VERTEX_ID)
		{ // la liste est vide
			list[0] = data;
		}
		else
		{
			for (i = LIST_LENGTH - 1; i > 0; i--)
			{
				list[i] = list[i - 1];
			}
			list[0] = data;
		}
	}

	/****************************************************************************************
	 * Fonction : Rajoute en queue de la liste pList l'element data.
	 ****************************************************************************************/
	template <typename T>
	void ListAddEnd(std::vector<T> &list, T data)
	{
		int i = 0;
		for (i = 0; i < LIST_LENGTH; i++)
		{
			if (list[i].currentID == INVALID_VERTEX_ID)
				break;
		}
		if (list[i].currentID == INVALID_VERTEX_ID)
		{
			list[i] = data;
		}
		else
		{
			// the list is full
		}
	}

	/****************************************************************************************
	 * Fonction : Renvoie le nombre d'element contenu dans la liste.
	 ****************************************************************************************/
	template <typename T>
	uint32_t ListLength(std::vector<T> &list)
	{
		int i = LIST_LENGTH - 1;
		for (i = LIST_LENGTH - 1; i >= 0; i--)
		{
			if (list[i].currentID != INVALID_VERTEX_ID)
				break;
		}
		return (i + 1);
	}

	/****************************************************************************************
	 * Fonction : Insere l'element data dans la liste pList, trie d'apres la fonction NodeFCmp.
	 * Elle doit renvoyer 0 si p1 pointe sur un element equivalent a p2.
	 * < 0 si p1 est inferieur a p2, >0 si p2 est superieur a p1.
	 ****************************************************************************************/
	template <typename T>
	void ListInsertSorted(std::vector<T> &list, T data)
	{
		int i = 0;
		int j = 0;
		boolean find = false;

		// si la liste est vide
		if (list[0].currentID == INVALID_VERTEX_ID)
		{
			list[0] = data;
		}
		else
		{
			// on recherche le premier élément dans la liste qui sera supérieur au data
			for (i = 0; i < LIST_LENGTH; i++)
			{
				if (list[i].currentID != INVALID_VERTEX_ID)
				{
					if (list[i].FCmp(data) > 0)
					{
						find = true;
						break;
					}
				}
				else
					break;
			}
			if (find == true)
			{
				// on décale toute la liste jusqu'é l'élément trouvé inclus
				for (j = LIST_LENGTH - 1; j >= i; j--)
				{
					if (list[j].currentID != INVALID_VERTEX_ID)
						list[j + 1] = list[j];
				}
				// test de verification
				if (j + 1 == i)
					// on insere l'element dans la liste
					list[i] = data;
			}
			else
			{
				if (i < LIST_LENGTH)
					list[i] = data;
				else
					i++;
			}
		}
	}

	/****************************************************************************************
	 * Fonction : Libere le premier element.
	 * Renvoi dans data le premier élément enlevé.
	 ****************************************************************************************/
	template <typename T>
	void ListGetFirstItem(std::vector<T> &list, T *data)
	{
		int i = 0;
		/* on fait pointer data vers cet element*/
		(*data) = list[0];
		/* on libere l'element */
		for (i = 0; i < LIST_LENGTH - 1; i++)
		{
			if (list[i].currentID != INVALID_VERTEX_ID)
				list[i] = list[i + 1];
		}
		list[LIST_LENGTH - 1] = T();
		//NodeNew(&list[LIST_LENGTH - 1]);
	}

	/****************************************************************************************
	 * Fonction : Recherche l'element de liste qui contient la premiere donnee data.
	 * Renvoie la position si trouvé, -1 sinon.
	 ****************************************************************************************/
	template <typename T>
	int ListIsDataExist(std::vector<T> &list, T data)
	{
		int i = 0;
		for (i = 0; i < LIST_LENGTH - 1; i++)
		{
			if (list[i].currentID != INVALID_VERTEX_ID)
				if (Mapping::Is_Equal_Vertex(list[i].currentID, data.currentID))
					return i;
		}
		return -1;
	}

	/****************************************************************************************
	 * Fonction : Initialise chaques éléments de la list
	 ****************************************************************************************/
	////template<size_t LIST_LENGTH>
	// void ListFreeALL(std::vector<T> &list)
	// {
	// 	for (int i = 0; i < LIST_LENGTH; i++)
	// 	{
	// 		list[i].currentCost = 0;
	// 		list[i].currentID = INVALID_VERTEX_ID;
	// 		list[i].parentCost = 0;
	// 		list[i].parentID = INVALID_VERTEX_ID;
	// 	}
	// }

	/****************************************************************************************
	 * Fonction : Initialise chaques éléments de la list
	 ****************************************************************************************/
	void ListVertexIDInit(std::vector<t_vertexID> &list)
	{
		int i;
		for (i = 0; i < LIST_LENGTH; i++)
		{
			list[i] = INVALID_VERTEX_ID;
		}
	}

#ifdef SERIAL_PRINT

	/****************************************************************************************
	 * Fonction : Imprime la liste dans UART 1 OUTPUT avec le simulateur.
	 ****************************************************************************************/
	template <typename T>
	void ListPrint(std::vector<T> &list,  String str)
	{
		int i = 0;
		printf("-------\n");
		printf("ListNode : %s \n", str);
		for (i = 0; i < LIST_LENGTH; i++)
		{
			if (list[i].currentID == INVALID_VERTEX_ID)
				break;
			printf("%d] ", i);
			// printf ("Liste 0x%X - NodeItem 0x%X ->",current,current->data);
			printf("Vertex ID:%d -> ", list[i].currentID);
			printf("Cost %ld ", list[i].currentCost);
			// printf("Heuristic %f , Cost %f " , NodeF(current->data) , NodeGetCost(current->data));
			printf("\n");
		}
	}

	/****************************************************************************************
	 * Fonction : Imprime la liste dans UART 1 OUTPUT avec le simulateur
	 ****************************************************************************************/
	void ListVertexPrint(std::vector<t_vertexID> &list)
	{
		int i = 0;
		printf("-------\n");
		printf("Solution founded : \n");
		while (list[i] != INVALID_VERTEX_ID)
		{
			printf("%d] ", i);
			printf("Vertex id:%d", list[i]);
			printf("\n");
			i++;
		}
	}

#endif

}
/**

il existe une liste de point d'intéret qui sont devant les actions a effectuer
on génère le graph de visisbilité ou la liste des autres points accessibles depuis un point donnée
celà se génère à l'init en regardant s'il existe un obstacle entre les deux points ou non
Puis lorsque l'on veut atteindre une destination on compose le chemin comme une suite de poins visble de pair en pair
On peut ajouter des segements qui interdise des zones 

Il ya une liste des points: 
vertex est composé d'un point et de sa visibilitée vers les autres points. (liste des autres points visibles) 
actif : avec les obstacles
passifs : sans les obstacles

Un noued est composé du coup pour aller vers ce noeud + le coups du parents 
le parents c'est le point avant qui est plus proche du robot. Si y'en a pas le parent c'est le robot


TODO: transformer le node en classe
*/

/****************************************************************************************
* Includes
****************************************************************************************/
#include "PathPlanning/NodeItem.h"

/****************************************************************************************
* Fonction : Set a NodeItem with a parent and a vertex ID
****************************************************************************************/
void NodeItem::Set(t_vertexID _parent , t_vertexID _currentPoint, uint32 _parentCost)
{
  currentID = _currentPoint;
  SetParent(_parent, _parentCost);
}

/****************************************************************************************
* Fonction : Create a new node
****************************************************************************************/
/*void NodeNew(NodeItem * newNode)
{
  newNode->currentCost = 0;
  newNode->parentCost = 0;
  newNode->currentID = INVALID_VERTEX_ID;
  newNode->parentID = INVALID_VERTEX_ID;
}*/

/****************************************************************************************
* Fonction : Set the parent
****************************************************************************************/
void NodeItem::SetParent(t_vertexID _parentID, uint32 _parentCost)
{
  parentID = _parentID;
  parentCost = _parentCost;
  // Refresh the cost : the cost of the parent + the cost of the current point
  if (_parentID != INVALID_VERTEX_ID)
  {

    uint32 cost = Mapping::Get_Distance_Vertex(currentID, parentID);
    currentCost = _parentCost + cost;
  }
  else
    currentCost = 0;
}

/****************************************************************************************
* Fonction : Get the cost
****************************************************************************************/
uint32 NodeItem::GetCost()
{
  return currentCost;
}


/****************************************************************************************
* Fonction : Get the F distance (Cost + Heuristic)
****************************************************************************************/
uint32 NodeItem::GetF() const
{
    return currentCost;
}

/****************************************************************************************
* Fonction : Comparaison between 2 node by their Heuristic
****************************************************************************************/
int8 NodeItem::FCmp(const NodeItem node) const
{
  uint32 nodeFp1 = GetF();
  uint32 nodeFp2 = node.GetF();

  if (nodeFp1 > nodeFp2)
    return 1;
  else if (nodeFp1 < nodeFp2)
    return -1;
  else
    return 0;
}

/****************************************************************************************
* Fonction : Return the cost if you move to this
****************************************************************************************/
uint32 NodeItem::CostWillBe()
{
  // The cost of the parent + the cost of the current point
  if (parentID != INVALID_VERTEX_ID)
  {
    uint32 cost = Mapping::Get_Distance_Vertex(currentID, parentID);
    return ( parentCost + cost );
  }
  else
  {
    return 0;
  }
}

/****************************************************************************************
* Fonction : Get a list of all neigbors from the node
* if they're not wall and are allowed by the Path
****************************************************************************************/
void NodeItem::ListGetPossibleNode(std::vector<NodeItem> &list)
{
  int i = 0;
  //int j = 0;
  for (i = 0; i < Mapping::Max_Vertex; i++)
  {
    if (Mapping::Is_Adjacent(currentID, i))
    {
      NodeItem node = NodeItem();
      node.Set(currentID, i, currentCost);
      list.push_back(node);
      //list[j] = NodeItem();
      //list[j].Set(currentID , i, currentCost);
	    //j++;
    }
  }
}

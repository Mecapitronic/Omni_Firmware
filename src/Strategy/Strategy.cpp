#include "Strategy/Strategy.h"

using namespace Printer;

namespace Strategy
{
    Actions actions;

    void Initialisation()
    {
        File file = SPIFFS.open("/actions.json", "r");
        if (file)
        {
            DynamicJsonDocument doc(4096);
            DeserializationError error = deserializeJson(doc, file);
            if (!error)
            {
                actions.FromJson(doc);
            }
            file.close();
        }
    }

    boolean Execute_Action(uint8_t ID)
    {
        Action action = actions.actions[ID];
        bool navigation = false;

        // Set_Timeout_Action(25000);

        // if (TIMEOUT_ACTION)
        //{
        //     action[ID].possible = ACTION_TO_RETRY;
        //     return OK;
        // }

        // navigation = Navigate_To_Vertex(action[ID].vertexID, 10);

        if (navigation)
        {
            // Execution de la strategie choisie
            // validation de l'action effectuée pour éviter de la refaire => valider quand
            // l'action est fini...
            // actions[ID].done = actions[ID].function();
            return true;
        }
        else
        {
            return false;
        }
    }

    void Print()
    {
      println("Strategy Action's at %s", actions.date);
      for (auto &&i : actions.actions)
      {
        i.Print();
      }
    }

} // namespace Strategy
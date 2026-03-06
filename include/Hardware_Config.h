#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

namespace Hardware_Config
{
    enum class ServoPosition
    {
        UpCanMin = 0,
        UpCanMax = 295,
        UpCanHaut = 5,
        UpCanBas = 290,
        UpCanMid = 40,

        LeftMax = 197,
        LeftDepose = 185,
        LeftPrise = 150,
        LeftTourne = 80,
        LeftMin = 70,

        RightMin = 100,
        RightDepose = 112,
        RightPrise = 149,
        RightTourne = 220,
        RightMax = 290,

        Min = 0,
        Max = 290
    };
    
    enum class ServoID
    {
        UpPlank = 6,                 // Servo pour soulever la planche
        UpCan = 15,                  // Servo pour lever les conserves
        Left = 5,                    // Servo pour la pince gauche
        Right = 3,                   // Servo pour la pince droite
        BroadCast = 0xFE             // Broadcast ID pour communiquer avec tous les servos
    };
}
#endif
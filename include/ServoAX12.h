#ifndef SERVO_AX12_H
#define SERVO_AX12_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <unordered_map>

#include "ESP32_Helper.h"
#include "pins.h"

namespace ServoAX12
{
    // https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/tree/master

    constexpr size_t MAX_BAUD = 5;
    const BaudRate dxlBaud[MAX_BAUD] = {BaudRate::BAUD_RATE_57600,
                                        BaudRate::BAUD_RATE_115200,
                                        BaudRate::BAUD_RATE_1000000,
                                        BaudRate::BAUD_RATE_2000000,
                                        BaudRate::BAUD_RATE_3000000};

    enum class DxlProtocolVersion
    {
        PROTOCOL_1 = 1,
        PROTOCOL_2 = 2
    };

    constexpr size_t MAX_PROTOCOL = 2;
    const DxlProtocolVersion dxlProtocol[MAX_PROTOCOL] = {DxlProtocolVersion::PROTOCOL_1,
                                                          DxlProtocolVersion::PROTOCOL_2};

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
        BroadCast = DXL_BROADCAST_ID // Broadcast ID pour communiquer avec tous les servos
    };

    // id vitesse acceleration position command_position ledState
    /**
     * @brief Structure représentant l'état d'un servo moteur.
     * @param id Identifiant du servo moteur.
     * @param vitesse Vitesse maximale du servo moteur en degrés par seconde.
     * @param positionMin Position minimale du servo moteur.
     * @param positionMax Position maximale du servo moteur.
     * @param command_position Position cible du servo moteur.
     * @param ledState État de la LED du servo moteur (allumée en mouvement, éteinte
     * sinon).
     */
    struct ServoMotion
    {
        uint8_t id;
        String name;
        float position;
        ServoPosition positionMin;
        ServoPosition positionMax;
        float command_position;
        bool IsMoving;
        bool ledState;

        ServoMotion()
        {
            id = (uint8_t)ServoID::BroadCast;
            name = "";
            position = 0;
            positionMin = ServoPosition::Min;
            positionMax = ServoPosition::Max;
            command_position = 0;
            IsMoving = false;
            ledState = false;
        }

        /**
         * @brief Construct a new Servo Motion object
         *
         * @param _id Identifiant du servo moteur
         * @param _vitesse Vitesse maximale du servo moteur en degrés par seconde
         * @param _acceleration Accélération maximale du servo moteur en degrés par
         * seconde carrée
         */
        ServoMotion(ServoID _id,
                    String _name,
                    ServoPosition _positionMin,
                    ServoPosition _positionMax)
        {
            // Initialisation des valeurs
            id = (uint8_t)_id;
            name = _name;
            position = 0;
            positionMin = _positionMin;
            positionMax = _positionMax;
            command_position = 0;
            IsMoving = false;
            ledState = false;
        }

        bool operator==(const ServoMotion &other) const
        {
            return id == other.id;
        }
    };

    void Initialisation(bool simulation);
    void InitServo(ServoMotion &servo);

    void StopAllServo();
    void StopServo(ServoMotion &servo);

    void StartAllServo();
    void StartServo(ServoMotion &servo);

    void Update();
    void UpdateServo(ServoMotion &servo);

    bool AreAllServoMoving();
    bool IsServoMoving(ServoMotion &servo);

    void SetServoPosition(ServoMotion &servo, float position);
    void Prise();
    void Depose();
    void Tourne();
    void Haut();
    void Bas();
    void Mid();

    void HandleCommand(Command cmd);
    const void PrintCommandHelp();
    int16_t Scan();
    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud);
    void PrintDxlInfo(uint8_t id = DXL_BROADCAST_ID);

    void TeleplotPosition();
    void PrintPosition();
} // namespace ServoAX12

namespace std
{
    template <> struct hash<ServoAX12::ServoMotion>
    {
        std::size_t operator()(const ServoAX12::ServoMotion &k) const
        {
            // Hash only the id, since operator== only compares id
            return std::hash<uint8_t>()(k.id);
        }
    };
} // namespace std
#endif
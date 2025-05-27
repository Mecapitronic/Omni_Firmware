#ifndef SERVO_AX12_H
#define SERVO_AX12_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <unordered_map>

#include "pins.h"
#include "ESP32_Helper.h"

namespace ServoAX12
{
    // https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/tree/master

    constexpr size_t MAX_BAUD = 5;
    const BaudRate dxlBaud[MAX_BAUD] = {
        BaudRate::BAUD_RATE_57600,
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
    const DxlProtocolVersion dxlProtocol[MAX_PROTOCOL] = {
        DxlProtocolVersion::PROTOCOL_1,
        DxlProtocolVersion::PROTOCOL_2};

    enum class ServoPosition
    {
        Haut = 270,
        Bas = 0,

        GaucheMax = 197,
        GaucheDepose = 185,
        GauchePrise = 150,
        GaucheTourne = 80,
        GaucheMin = 70,

        DroiteMin = 100,
        DroiteDepose = 112,
        DroitePrise = 149,
        DroiteTourne = 220,
        DroiteMax = 290,

        Min = 0,
        Max = 290
    };

    enum class ServoID
    {
        Up = 6,                      // Servo pour lever le bras
        Left = 5,                    // Servo pour la pince gauche
        Right = 3,                   // Servo pour la pince droite
        BroadCast = DXL_BROADCAST_ID // Broadcast ID pour communiquer avec tous les servos
    };

    // id vitesse acceleration position command_position ledState
    /**
     * @brief Structure représentant l'état d'un servo moteur.
     * @param id Identifiant du servo moteur.
     * @param vitesse Vitesse maximale du servo moteur en degrés par seconde.
     * @param acceleration Accélération maximale du servo moteur en degrés par seconde carrée.
     * @param position Position actuelle du servo moteur.
     * @param command_position Position cible du servo moteur.
     * @param ledState État de la LED du servo moteur (allumée en mouvement, éteinte sinon).
     */
    struct ServoMotion
    {
        uint8_t id;
        int32_t vitesse;
        int32_t acceleration;
        float position;
        float command_position;
        bool IsMoving;
        bool ledState;
        ServoMotion()
        {
            id = (uint8_t)ServoID::BroadCast;
            vitesse = 0;
            acceleration = 0;
            position = 0;
            command_position = 0;
            IsMoving = false;
            ledState = false;
        }
        /**
         * @brief Construct a new Servo Motion object
         *
         * @param _id Identifiant du servo moteur
         * @param _vitesse Vitesse maximale du servo moteur en degrés par seconde
         * @param _acceleration Accélération maximale du servo moteur en degrés par seconde carrée
         */
        ServoMotion(ServoID _id, int _vitesse, int _acceleration)
        {
            // Initialisation des valeurs
            id = (uint8_t)_id;
            vitesse = _vitesse;
            acceleration = _acceleration;
            position = 0;
            command_position = 0;
            IsMoving = false;
            ledState = false;
        }
        bool operator==(const ServoMotion &other) const
        {
            return id == other.id;
        }
    };

    void Initialisation();
    void InitServo(ServoMotion &servo);

    void StopAllServo();
    void StopServo(ServoMotion &servo);

    void Update();
    void UpdateServo(ServoMotion &servo);

    void SetServoPosition(ServoMotion &servo, float position);
    void Prise();
    void Depose();
    void Tourne();
    void Haut();
    void Bas();

    void HandleCommand(Command cmd);
    const void PrintCommandHelp();
    int16_t Scan();
    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud);
    void PrintDxlInfo(uint8_t id = DXL_BROADCAST_ID);

    void TeleplotPosition();
}

namespace std
{
    template <>
    struct hash<ServoAX12::ServoMotion>
    {
        std::size_t operator()(const ServoAX12::ServoMotion &k) const
        {
            // Hash only the id, since operator== only compares id
            return std::hash<uint8_t>()(k.id);
        }
    };
}
#endif
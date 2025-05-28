#include "ServoAX12.h"

using namespace Printer;

namespace ServoAX12
{
    Dynamixel2Arduino dxl(SERIAL_SERVO, PIN_SERVO_DIR);

    std::unordered_map<ServoID, ServoMotion, std::hash<ServoID>> Servos;

    void Initialisation()
    {
        SERIAL_SERVO.setPins(RX_SERVO, TX_SERVO);
        // Set Port baudrate. This has to match with DYNAMIXEL baudrate.
        dxl.begin((unsigned long)BaudRate::BAUD_RATE_1000000);
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)DxlProtocolVersion::PROTOCOL_1);

        Servos.clear();
        Servos[ServoID::UpPlank] = ServoMotion(
            ServoID::UpPlank, "UpPlank", ServoPosition::Min, ServoPosition::Max);
        Servos[ServoID::UpCan] = ServoMotion(
            ServoID::UpCan, "UpCan", ServoPosition::UpCanMin, ServoPosition::UpCanMax);
        Servos[ServoID::Left] = ServoMotion(
            ServoID::Left, "Left", ServoPosition::LeftMin, ServoPosition::LeftMax);
        Servos[ServoID::Right] = ServoMotion(
            ServoID::Right, "Right", ServoPosition::RightMin, ServoPosition::RightMax);

        for (auto &[id, servo] : Servos)
        {
            InitServo(servo);
        }
    }

    void InitServo(ServoMotion &servo)
    {
        if (dxl.ping((uint8_t)servo.id))
        {
            print("Init Servo ID : ", servo.id);
            println(" name : ", servo.name);
            PrintDxlInfo(servo.id);

            servo.ledState = true;
            dxl.ledOn(servo.id);

            // Turn off torque when configuring items in EEPROM area
            dxl.torqueOff(servo.id);
            dxl.setOperatingMode(servo.id, OP_POSITION);
            dxl.torqueOn(servo.id);

            servo.position = servo.command_position =
                dxl.getPresentPosition(servo.id, UNIT_DEGREE);
        }
        else
        {
            println("Servo ID : ", servo.id, " is NOT connected");
            return;
        }
    }

    void StopAllServo()
    {
        println("Stop All Servo");
        for (auto &[id, servo] : Servos)
        {
            StopServo(servo);
        }
    }

    void StopServo(ServoMotion &servo)
    {
        dxl.torqueOff(servo.id);
        dxl.ledOff(servo.id);
    }

    void StartAllServo()
    {
        println("Start All Servo");
        for (auto &[id, servo] : Servos)
        {
            StartServo(servo);
        }
    }

    void StartServo(ServoMotion &servo)
    {
        dxl.torqueOn(servo.id);
    }

    void Update()
    {
        // 1 ms / servo
        for (auto &[id, servo] : Servos)
        {
            UpdateServo(servo);
        }
    }

    void UpdateServo(ServoMotion &servo)
    {
        if (servo.position >= servo.command_position + 0.5
            || servo.position <= servo.command_position - 0.5)
        {
            servo.position = dxl.getPresentPosition(servo.id, UNIT_DEGREE);
            if (!servo.ledState)
            {
                servo.ledState = true;
                dxl.ledOn(servo.id);
            }
            servo.IsMoving = true;
        }
        else
        {
            if (servo.ledState)
            {
                servo.ledState = false;
                dxl.ledOff(servo.id);
            }
            servo.IsMoving = false;
        }
    }

    bool AreAllServoMoving()
    {
        for (const auto &[id, servo] : Servos)
        {
            if (servo.IsMoving)
            {
                return true;
            }
        }
        return false;
    }

    bool IsServoMoving(ServoMotion &servo)
    {
        return servo.IsMoving;
    }

    void SetServoPosition(ServoMotion &servo, float position)
    {
        if (position < (float)servo.positionMin || position > (float)servo.positionMax)
        {
            println("Position out of range for Servo ID : ", servo.id);
            println("Position : ", position);
            println("Min : ", (float)servo.positionMin);
            println("Max : ", (float)servo.positionMax);
            return;
        }
        servo.command_position = position;
        servo.IsMoving = true;
        dxl.setGoalPosition(servo.id, servo.command_position, UNIT_DEGREE);
    }

    void Prise()
    {
        SetServoPosition(Servos[ServoID::Left], (float)ServoPosition::LeftPrise);
        SetServoPosition(Servos[ServoID::Right], (float)ServoPosition::RightPrise);
    }

    void Depose()
    {
        SetServoPosition(Servos[ServoID::Left], (float)ServoPosition::LeftDepose);
        SetServoPosition(Servos[ServoID::Right], (float)ServoPosition::RightDepose);
    }

    void Tourne()
    {
        SetServoPosition(Servos[ServoID::Left], (float)ServoPosition::LeftTourne);
        SetServoPosition(Servos[ServoID::Right], (float)ServoPosition::RightTourne);
    }

    void Haut()
    {
        SetServoPosition(Servos[ServoID::UpCan], (float)ServoPosition::UpCanHaut);
    }

    void Bas()
    {
        SetServoPosition(Servos[ServoID::UpCan], (float)ServoPosition::UpCanBas);
    }

    void Mid()
    {
        SetServoPosition(Servos[ServoID::UpCan], (float)ServoPosition::UpCanMid);
    }

    void HandleCommand(Command cmd)
    {
        if (cmd.cmd == "AX12Scan")
        {
            Scan();
            // Put back to inital state
            Initialisation();
        }
        else if (cmd.cmd == "AX12PrintInfo")
        {
            if (cmd.size == 1)
            {
                PrintDxlInfo(cmd.data[0]);
            }
            else
                PrintDxlInfo();
        }
        else if (cmd.cmd == "AX12Pos")
        {
            if (cmd.size == 2)
            {
                // AX12Pos:3:100
                ServoID id = (ServoID)cmd.data[0];
                print("AX12 Servo id : ", cmd.data[0]);
                if (Servos.count(id) == 0)
                {
                    println(" is not initialized");
                }
                else
                {
                    println(" Position : ", cmd.data[1]);
                    SetServoPosition(Servos[id], (float)cmd.data[1]);
                }
            }
            else
            {
                // TeleplotPosition();
                PrintPosition();
            }
        }
        else if (cmd.cmd == "AX12Stop")
        {
            println("AX12Stop");
            StopAllServo();
        }
        else if (cmd.cmd == "AX12Start")
        {
            println("AX12Start");
            StartAllServo();
        }
        else if (cmd.cmd == "AX12Prise")
        {
            Prise();
        }
        else if (cmd.cmd == "AX12Depose")
        {
            Depose();
        }
        else if (cmd.cmd == "AX12Tourne")
        {
            Tourne();
        }
        else if (cmd.cmd == "AX12Haut")
        {
            Haut();
        }
        else if (cmd.cmd == "AX12Bas")
        {
            Bas();
        }
        else if (cmd.cmd == "AX12Mid")
        {
            Mid();
        }
        else if (cmd.cmd == "AX12Help")
        {
            PrintCommandHelp();
        }
        else
        {
            println("Unknown command : ", cmd.cmd);
        }
    }

    const void PrintCommandHelp()
    {
        Printer::println("AX12 Command Help :");
        Printer::println(" > AX12Scan");
        Printer::println(
            "      Scan all Dynamixel servos on all protocols and baudrates");
        Printer::println(" > AX12PrintInfo[:id]");
        Printer::println("      Print info for all servos or for the given id");
        Printer::println(" > AX12Pos[:id]:[position]");
        Printer::println("      Set servo [id] to [position] (in degrees)");
        Printer::println("      If no argument, print current positions");
        Printer::println(" > AX12Vit:[id]:[vitesse]");
        Printer::println("      Set velocity for servo [id] to [vitesse]");
        Printer::println(" > AX12Acc:[id]:[acceleration]");
        Printer::println("      Set acceleration for servo [id] to [acceleration]");
        Printer::println(" > AX12Stop");
        Printer::println("      Stop all servos (torque off)");
        Printer::println(" > AX12Prise");
        Printer::println("      Move left and right servos to 'prise' position");
        Printer::println(" > AX12Depose");
        Printer::println("      Move left and right servos to 'depose' position");
        Printer::println(" > AX12Tourne");
        Printer::println("      Move left and right servos to 'tourne' position");
        Printer::println(" > AX12Haut");
        Printer::println("      Move up servo to 'haut' position");
        Printer::println(" > AX12Bas");
        Printer::println("      Move up servo to 'bas' position");
        Printer::println(" > AX12Help");
        Printer::println("      Print this help");
        Printer::println();
    }

    int16_t Scan()
    {
        int16_t found_dynamixel = 0;
        for (auto &&proto : dxlProtocol)
        {
            for (auto &&baud : dxlBaud)
            {
                found_dynamixel += Scan(proto, baud);
            }
        }
        println("Total : ", found_dynamixel, " Dynamixel(s) found");
        return found_dynamixel;
    }

    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud)
    {
        int16_t found_dynamixel = 0;
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)_protocol);
        print("Scan Protocol ", (float)_protocol, " - ");

        // Set Port baudrate.
        dxl.begin((int)_dxlBaud);
        println("Scan Baudrate ", (int)_dxlBaud);
        for (int id = 0; id < DXL_BROADCAST_ID; id++)
        {
            // iterate until all ID in each baudrate is scanned.
            if (dxl.ping(id))
            {
                print("ID : ", id);
                println(", Model Number: ", dxl.getModelNumber(id));
                found_dynamixel++;
            }
        }
        println("Found ", found_dynamixel, " Dynamixel(s)");
        return found_dynamixel;
    }

    void PrintDxlInfo(uint8_t id)
    {
        if (dxl.ping(id))
        {
            print("ID : ", id);
            println(", Model Number: ", dxl.getModelNumber(id));
        }
    }

    void TeleplotPosition()
    {
        for (auto &[id, servo] : Servos)
        {
            teleplot("Servo " + servo.name + " " + String(servo.id), servo.position);
        }
    }

    void PrintPosition()
    {
        for (auto &[id, servo] : Servos)
        {
            servo.position = servo.command_position =
                dxl.getPresentPosition(servo.id, UNIT_DEGREE);
            println("Servo " + servo.name + " " + String(servo.id) + " : ",
                    servo.position);
        }
    }
} // namespace ServoAX12
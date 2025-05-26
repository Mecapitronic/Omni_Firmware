#include "Servo.h"

using namespace Printer;

namespace ServoAX12
{
    Dynamixel2Arduino dxl(SERIAL_SERVO, PIN_SERVO_DIR);

    ServoMotion Servo_Up = {6, 30, 50, 0, 0, false};
    // ServoMotion Servo_Down = {7, 30, 50, 0, 0, false};
    ServoMotion Servo_Left = {5, 30, 50, 0, 0, false};
    ServoMotion Servo_Right = {3, 30, 50, 0, 0, false};

    void Initialisation()
    {
        SERIAL_SERVO.setPins(RX_SERVO, TX_SERVO);
        // Set Port baudrate. This has to match with DYNAMIXEL baudrate.
        dxl.begin((unsigned long)BaudRate::BAUD_RATE_1000000);
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)DxlProtocolVersion::PROTOCOL_1);

        InitServo(Servo_Up);
        // InitServo(Servo_Down);
        InitServo(Servo_Left);
        InitServo(Servo_Right);
    }

    void InitServo(ServoMotion &servo)
    {
        if (dxl.ping(servo.id))
        {
            println("Init Servo ID : ", servo.id);
            PrintDxlInfo(servo.id);

            servo.ledState = true;
            dxl.ledOn(servo.id);

            // Turn off torque when configuring items in EEPROM area
            dxl.torqueOff(servo.id);
            dxl.setOperatingMode(servo.id, OP_POSITION);
            dxl.torqueOn(servo.id);

            // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
            dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, servo.id, servo.vitesse);
            dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, servo.id, servo.acceleration);

            servo.position = servo.command_position = dxl.getPresentPosition(servo.id, UNIT_DEGREE);

            // Limit the angle range of the motor, 0 in both for speed control mode
            // dxl.writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0);   // 0
            // dxl.writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 1023);  // 1023

            // servo.ledTO.Start(200);
        }
        else
        {
            println("Servo ID : ", servo.id, " is NOT connected");
            return;
        }
    }

    void Update()
    {
        // 1 ms / servo
        UpdateServo(Servo_Up);
        UpdateServo(Servo_Left);
        UpdateServo(Servo_Right);
    }

    void UpdateServo(ServoMotion &servo)
    {
        if (servo.position >= servo.command_position + 1 || servo.position <= servo.command_position - 1)
        {
            servo.position = dxl.getPresentPosition(servo.id, UNIT_DEGREE);
            if (!servo.ledState)
            {
                servo.ledState = true;
                dxl.ledOn(servo.id);
            }
        }
        else
        {
            if (servo.ledState)
            {
                servo.ledState = false;
                dxl.ledOff(servo.id);
            }
        }
    }

    void SetServoPosition(ServoMotion &servo, float position)
    {
        servo.command_position = position;
        dxl.setGoalPosition(servo.id, servo.command_position, UNIT_DEGREE);
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
                PrintDxlInfo(cmd.data[0]);
            else
                PrintDxlInfo();
        }
        else if (cmd.cmd == "AX12Pos" && cmd.size == 1)
        {
            // 0° to 290°
            // AX12Pos:100:Up
            if (cmd.dataStr == "Up")
                SetServoPosition(Servo_Up, cmd.data[0]);
            else if (cmd.dataStr == "Left")
                SetServoPosition(Servo_Left, cmd.data[0]);
            else if (cmd.dataStr == "Right")
                SetServoPosition(Servo_Right, cmd.data[0]);
        }
        /*
        else if (cmd.cmd == "AX12Vit" && cmd.size == 2)
        {
            // AX12Vit:5:30
            dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, cmd.data[0], cmd.data[1]);
        }
        else if (cmd.cmd == "AX12Acc" && cmd.size == 2)
        {
            // AX12Acc:5:50
            dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, cmd.data[0], cmd.data[1]);
        }*/
    }

    const void PrintCommandHelp()
    {
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
}
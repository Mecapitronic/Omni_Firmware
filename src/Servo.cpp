#include "Servo.h"

using namespace Printer;

namespace ServoAX12
{

    Dynamixel2Arduino dxl(SERIAL_SERVO, PIN_SERVO_DIR);

    void Initialisation()
    {
        SERIAL_SERVO.setPins(RX_SERVO, TX_SERVO);
        // Set Port baudrate. This has to match with DYNAMIXEL baudrate.
        dxl.begin((unsigned long)BaudRate::BAUD_RATE_1000000);
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)DxlProtocolVersion::PROTOCOL_1);

        /*/
        uint8_t id = 5; // Dynamixel ID
        // Get DYNAMIXEL information
        dxl.ping(id);

        // Turn off torque when configuring items in EEPROM area
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, OP_POSITION);
        dxl.torqueOn(id);

        // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
        dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 30);
        dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, 50);

        // Limit the angle range of the motor, 0 in both for speed control mode
        // dxl.writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0);   // 0
        // dxl.writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 1023);  // 1023
        */
    }

    void Update() {}

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
    }

    const void PrintCommandHelp() {}

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
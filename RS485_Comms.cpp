#include <SerialStream.h>
#include <iostream>
#include <cstring>

using namespace std;
using namespace LibSerial;

int main() {
    SerialStream serial;
    serial.Open("/dev/ttyUSB0");
    serial.SetBaudRate(SerialStreamBuf::BAUD_9600);
    serial.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    serial.SetNumOfStopBits(1);
    serial.SetParity(SerialStreamBuf::PARITY_NONE);
    serial.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);

    string command = "SetMotorSpeed:100";
    serial.write(command.c_str(), command.length());

    return 0;
}
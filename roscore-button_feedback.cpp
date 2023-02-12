#include <wiringPi.h>
#include <iostream>
#include <string>

const int buttonPin = 0;
const std::string launchCommand = "roslaunch my_package my_launch_file.launch";
const std::string coreCommand = "roscore";

int main(int argc, char **argv)
{
  wiringPiSetup();
  pinMode(buttonPin, INPUT);

  while (true)
  {
    if (digitalRead(buttonPin) == LOW)
    {
      std::cout << "Button pressed!" << std::endl;
      system(coreCommand.c_str());
      sleep(5);
      system(launchCommand.c_str());
      break;
    }
  }

  return 0;
}
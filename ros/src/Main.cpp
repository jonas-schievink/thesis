#include "EvdevEncoder.hpp"
#include "Motor.hpp"

#include <iostream>

using std::cout;
using std::endl;

int main(int argc, char** argv)
{
    Encoder* left = new EvdevEncoder("rot_left");
    return 0;
}

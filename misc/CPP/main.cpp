//build with: g++ -lONA -lm main.cpp
#include <iostream>
//namespace ona { //if used with other namespaces
extern "C" {
    #include "./../../src/NAR.h"
    //#include <ona/NAR.h>
    #undef assert //for ROS and Mac
}
//}
//using namespace ona;

bool executed = false;
Feedback NAR_Op()
{
    std::cout << "Hello world" << std::endl;
    executed=true;
    return (Feedback) {0};
}
int main()
{
    NAR_INIT();
    MOTOR_BABBLING_CHANCE = 0.0;
    NAR_AddOperation((char*) "^op", (Action) NAR_Op);
    NAR_AddInputNarsese((char*) "<(a &/ ^op) =/> g>.");
    NAR_AddInputNarsese((char*) "a. :|:");
    NAR_AddInputNarsese((char*) "g! :|:");
    Globals_assert(executed, (char*) "Eecution should have happened!");
    return 0;
}

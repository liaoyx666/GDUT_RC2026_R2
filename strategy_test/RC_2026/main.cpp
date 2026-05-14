#include <iostream>
#include "./include/Nine_Square/Game_Strategy.h"
Strategy::Best_Weigt_Finding  a(5.0f,2.0f,5.0f,2.0f,
                                 5.0f,2.0f,5.0f,2.0f,30.0f,30,{0.0f,1.0f},{0.0f,1.0f},{0.0f,15.0f},{0.0f,100.0f});

int main() {
    a.Set_Black_Num(1,2,1,1,2,1);
    a.Main_Runing();    
    return 0;
}
#include "display.hpp"
#include "others.hpp"
#include "../config.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

std::string SerialWatch;

void PinSetup(){
    
}

//正面0度時計回りの度数法の角度を座標平面の弧度法(正面π/2反時計回り)に変換する。
//定義域は 0 <= θ < 2π
float radian(float angle){
    if(angle <= 90){
        return (angle * -1.0 + 90) * 3.1415 / 180;
    }else{
        return (angle * -1.0 + 450) * 3.1415 / 180;
    }
}
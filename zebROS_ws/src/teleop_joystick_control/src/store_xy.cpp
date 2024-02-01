#include "teleop_joystick_control/store_xy.h"

store_xy operator*(const double scalar, const store_xy& obj){
    store_xy multiplied(0.0, 0.0);
    multiplied.x = scalar * obj.x;
    multiplied.y = scalar * obj.y;
    return multiplied;
}

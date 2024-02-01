#ifndef STORE_XY_INC_
#define STORE_XY_INC_

#include <cmath>

class store_xy {
    
    public: 
        double x;
        double y;

        store_xy(double input_x, double input_y) {
            x = input_x;
            y = input_y;
        }

        store_xy operator+(const store_xy& lower) const {
            store_xy added(0.0, 0.0);
            added.x = x + lower.x;
            added.y = y + lower.y;
            return added; 
        }

        double hypot() const{
            return std::hypot(x, y);
        }
};

store_xy operator*(const double scalar, const store_xy& obj);

#endif
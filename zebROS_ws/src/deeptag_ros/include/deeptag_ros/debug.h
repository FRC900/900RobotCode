#ifndef DEBUG_INC__
#define DEBUG_INC__
#include <array>
#include <string>
#include <vector>
#ifdef DEBUG
#include <iostream>
#endif
template <typename T>
void printPoints(const std::string &name, const T &vec)
{
#ifdef DEBUG
    std::cout << name << " : " << std::endl;
    for (size_t i = 0; i < vec.size(); i++)
    {
        std::cout << "\t" << vec[i].x << " " << vec[i].y << std::endl;
    }
#endif
}
#endif
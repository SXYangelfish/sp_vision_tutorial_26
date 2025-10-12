#include <fmt/core.h>  
#include <iostream>

int main() {
    //fmt::print
    fmt::print("Hello, {}!\n", "World");

    //fmt::format
    std::string msg = fmt::format("number is {}.", 1);
    std::cout << msg << std::endl;

    return 0;
}
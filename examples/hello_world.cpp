#include<iostream>

#include "hello.h"

class test{
    public:
    test(uint8_t* datas){
        std::cout << datas[0] << datas[1] << datas[2] << datas[3] << std::endl;
        std::cout << sizeof(&datas) << std::endl;
    }
};

int main(){
    hello::say_hello();

    uint8_t data[] = {'1','2','3'};
    std::cout << data[0] << data[1] << data[2] << data[3] << std::endl;
    std::cout << sizeof(data) << std::endl;
    test test(data);
    return 0;
}
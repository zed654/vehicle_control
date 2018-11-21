#include <iostream>
#include "c_test.h"



int test_val;

//__test_ _test___;

void test_()
{
    
    std::cout << test_val << std::endl;
    std::cout << _test___.a << std::endl;
    
    _test___.b = 3333;
}


void *test_thread(void *data)
{
    while(1)
    {
        printf("test_thread\n");
    }
}

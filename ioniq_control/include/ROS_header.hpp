#include <stdio.h>
#include <stdlib.h>
#include <iostream>

typedef struct ARG
{
    int argc;
    char** argv;
} ARG;

extern ARG *arg;

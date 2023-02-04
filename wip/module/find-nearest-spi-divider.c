


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

uint32_t CONST = 1200000;

uint32_t nums[9] = {99999999,49000000,24000000,11500000,6150000,3025000,1462500,771250,380625};
//uint32_t nums[9] = {390625,781250,1562500,3125000,6250000,12500000,25000000,50000000,100000000};

uint32_t is_first_closest(int n) {
    int results;
    for (int i = 0; i < n; ++i) {
        printf("loop results = %d\r\n", results);
        if (CONST > nums[i]) {
            printf("divider number = %d\r\n", i);
            return nums[i];
        }
    }
    return 1;
}

int main(uint32_t argc, const char *argv[])
{
int result;
int divider;
long arg = strtol(argv[1], NULL, 10);
CONST = arg;
result = is_first_closest(9);
printf("closest = %d\r\n", result);
divider = (200000000 / result);
printf("divider = %d\r\n", divider);
}


#include <stdio.h>
//#include <process.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {

if( argv[1] == 0 ) {
   printf("\n Please supply base clock in mhz; heres a common list \n");

u_int16_t test;
test |= 0x03;
printf("test with 0x03 = %d\r\n", test);
test |= 0x08;
printf("test with 0x03 then added 0x08 = %d\r\n", test);
int divis = 2;
float result = 0.000;
printf("\r\n");
printf("Clock at 200\r\n");
printf("\r\n");
for (divis = 2; divis < 513; divis = divis * 2) {
result = (200 / ((double)divis));
printf("200  /  %d =  %f\r\n", divis, result);
    }
printf("\r\n");
printf("Clock at 100\r\n");
printf("\r\n");
for (divis = 2; divis < 513; divis = divis * 2) {
result = (100 / ((double)divis));
printf("100 divided by  %d =  %f\r\n", divis, result);
    }
printf("\r\n");
printf("Clock at 50\r\n");
printf("\r\n");
for (divis = 2; divis < 513; divis = divis * 2) {
result = (50 / ((double)divis));
printf("50 divided by  %d =  %f\r\n", divis, result);
    }
printf("\r\n");
printf("Clock at 25\r\n");
printf("\r\n");
for (divis = 2; divis < 513; divis = divis * 2) {
result = (25 / ((double)divis));
printf("25 divided by  %d =  %f\r\n", divis, result);
    }
printf("\r\n");
printf("Clock at 12.5\r\n");
printf("\r\n");
for (divis = 2; divis < 513; divis = divis * 2) {
result = (12.5 / ((double)divis));
printf("12.5 divided by  %d =  %f\r\n", divis, result);
    }
printf("\r\n");
printf("Clock at 6.25\r\n");
printf("\r\n");
for (divis = 2; divis < 513; divis = divis * 2) {
result = (6.25 / ((double)divis));
printf("6.25 divided by  %d =  %f\r\n", divis, result);
    }
printf("\r\n");
printf("Clock at 3.125\r\n");
printf("\r\n");
for (divis = 2; divis < 513; divis = divis * 2) {
result = (((double)3.125) / ((double)divis));
printf("3.125 divided by  %d =  %f\r\n", divis, result);
    }
  return 0;
    } else {
int divis = 2;
float result = 0.000;
long arg = strtol(argv[1], NULL, 10);
printf("\r\n");
printf("Clock at %ld\r\n", arg);
for (divis = 2; divis < 513; divis = divis * 2) {
result = (arg / ((double)divis));
printf("%ld divided by  %d =  %f\r\n", arg, divis, result);
    }
printf("\r\n");
return 0;
    }
}  

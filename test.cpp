//
//  Test.c
//

#define FPPROCESSORDOESNTSUPPORTMULSU

#include <stdio.h>
#include "minunit.h"
#include "lib_fp.h"

int tests_run = 0;

static char * test_fp_math() {
    fixedpointnum num1 = (10L<<FIXEDPOINTSHIFT);
    fixedpointnum num2 = (50L<<FIXEDPOINTSHIFT);
    fixedpointnum product = lib_fp_multiply(num1,num2);
    printf("%ld * %ld = %ld\n", num1>>FIXEDPOINTSHIFT, num2>>FIXEDPOINTSHIFT, product>>FIXEDPOINTSHIFT);
    mu_assert("error, product != 500", product == (500L<<FIXEDPOINTSHIFT));
    
    fixedpointnum quotient = lib_fp_multiply((100L<<FIXEDPOINTSHIFT),FIXEDPOINTONEOVERFOUR);
    printf("%ld / 4 = %ld\n", 100L, quotient>>FIXEDPOINTSHIFT);
    mu_assert("error, quotient != 25", quotient == (25L<<FIXEDPOINTSHIFT));
    
    fixedpointnum frac1 = FIXEDPOINTCONSTANT(4.5);
    fixedpointnum frac2 = FIXEDPOINTCONSTANT(0.5);
    fixedpointnum product2 = lib_fp_multiply(frac1,frac2);
    printf("%ld * %ld = %ld\n", frac1, frac2, product2);
    mu_assert("error, product != 2", product2 == FIXEDPOINTCONSTANT(2.25));
    
    num1 = (10L<<FIXEDPOINTSHIFT);
    num2 = -FIXEDPOINTONE;
    fixedpointnum product3 = lib_fp_multiply(num1,num2);
    printf("%ld * %ld = %ld\n", num1>>FIXEDPOINTSHIFT, num2>>FIXEDPOINTSHIFT, product3>>FIXEDPOINTSHIFT);
    mu_assert("error, product != -10", product3 == (-10L<<FIXEDPOINTSHIFT));
    return 0;
}

static char * test_fp_constrain() {
    fixedpointnum min = (500L<<FIXEDPOINTSHIFT);
    fixedpointnum max = (1000L<<FIXEDPOINTSHIFT);
    
    fixedpointnum large = (2000L<<FIXEDPOINTSHIFT);
    lib_fp_constrain(&large,min,max);
    printf("min:%l > %l < max:%ld\n",min>>FIXEDPOINTSHIFT, large>>FIXEDPOINTSHIFT, max>>FIXEDPOINTSHIFT);
    mu_assert("error, large != 1000", large == max);
    
    fixedpointnum small = (FIXEDPOINTONE);
    lib_fp_constrain(&small,min,max);
    printf("min:%ld > %ld < max:%ld\n",min>>FIXEDPOINTSHIFT, small>>FIXEDPOINTSHIFT, max>>FIXEDPOINTSHIFT);
    mu_assert("error, small != 500", small == min);

    return 0;
}

static char * test_fp_constrain180() {
    fixedpointnum inside = (90L<<FIXEDPOINTSHIFT);
    printf("inside before: %ld\n",inside);
    lib_fp_constrain180(&inside);
    printf("inside %l < max:180\n",inside>>FIXEDPOINTSHIFT);
    mu_assert("error, inside != 90", inside == FIXEDPOINT90);
    
    fixedpointnum small = FIXEDPOINTCONSTANT(-290);
    printf("small before: %ld\n",small);
    lib_fp_constrain180(&small);
    printf("small %ld < max:180\n",small>>FIXEDPOINTSHIFT);
    mu_assert("error, small != 70", small == 70L<<FIXEDPOINTSHIFT);
    
    fixedpointnum large = FIXEDPOINTCONSTANT(320);
    lib_fp_constrain180(&large);
    printf("large %ld < max:180\n",large>>FIXEDPOINTSHIFT);
    mu_assert("error, large != 180", large == -40<<FIXEDPOINTSHIFT);
    
    return 0;
}

static char * all_tests() {
    mu_run_test(test_fp_math);
    mu_run_test(test_fp_constrain);
    mu_run_test(test_fp_constrain180);
    return 0;
}

int main(int argc, char **argv) {
    char *result = all_tests();
    if (result != 0) {
        printf("%s\n", result);
    }
    else {
        printf("ALL TESTS PASSED\n");
    }
    printf("Tests run: %d\n", tests_run);
    
    return result != 0;
}

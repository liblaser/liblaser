/* What the test programs share: checks that say what failed and go on, so
 * that one run shows every failure and not only the first. */
#ifndef LASER_TESTS_CHECK_H
#define LASER_TESTS_CHECK_H

#include <stdio.h>

static int check_failures;

#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            check_failures++; \
            printf("    FAILED, %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

/* Names the case the checks that follow belong to. */
static inline void check_case(const char *name)
{
    printf("%s\n", name);
}

/* The program's exit status. */
static inline int check_result(void)
{
    if (check_failures != 0) {
        printf("%d check(s) failed\n", check_failures);
        return 1;
    }
    puts("all passed");
    return 0;
}

#endif

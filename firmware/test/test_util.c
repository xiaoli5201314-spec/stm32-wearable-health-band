/*
 * test_util.c -- 单元测试框架实现
 */
#include "test_util.h"
#include <string.h>

test_ctx_t g_tc;

void test_suite_begin(const char *suite)
{
    g_tc.suite = suite;
    g_tc.case_failed = 0;
    printf("\n== SUITE: %s ==\n", suite);
}

int test_suite_end(void)
{
    printf("== SUITE DONE: %s (cases=%d, checks=%d, failed=%d) ==\n",
           g_tc.suite, g_tc.cases, g_tc.checks, g_tc.failed);
    return g_tc.failed;
}

void test_case_begin(const char *name)
{
    g_tc.cases++;
    printf("  -- %s\n", name);
}

void test_pass_count(void)
{
    g_tc.checks++;
}

/*
 * test_util.h -- 极简单元测试框架（无第三方依赖）
 *
 * 用法：
 *   TEST_CASE("frame encode/decode");
 *   TEST_ASSERT(cond, "描述");
 *   TEST_ASSERT_EQ_INT(a, b, "描述");
 *   TEST_END();
 * run_tests.c 汇总所有套件的通过/失败数，任何失败都让进程返回非 0。
 */
#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdio.h>

typedef struct {
    int   cases;
    int   checks;
    int   failed;
    int   case_failed;
    const char *suite;
} test_ctx_t;

extern test_ctx_t g_tc;

void test_suite_begin(const char *suite);
int  test_suite_end(void);
void test_case_begin(const char *name);
void test_pass_count(void);

#define TEST_CASE(name) do { test_case_begin(name); } while (0)

#define TEST_ASSERT(cond, msg)                                                 \
    do {                                                                       \
        g_tc.checks++;                                                         \
        if (!(cond)) {                                                         \
            g_tc.failed++;                                                     \
            g_tc.case_failed = 1;                                              \
            printf("  [FAIL] %s:%d  %s\n", __FILE__, __LINE__, (msg));          \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_EQ_INT(actual, expect, msg)                                \
    do {                                                                       \
        long _a = (long)(actual);                                              \
        long _e = (long)(expect);                                              \
        g_tc.checks++;                                                         \
        if (_a != _e) {                                                        \
            g_tc.failed++;                                                     \
            g_tc.case_failed = 1;                                              \
            printf("  [FAIL] %s:%d  %s  (actual=%ld expect=%ld)\n",            \
                   __FILE__, __LINE__, (msg), _a, _e);                         \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_EQ_UINT(actual, expect, msg)                               \
    do {                                                                       \
        unsigned long _a = (unsigned long)(actual);                            \
        unsigned long _e = (unsigned long)(expect);                            \
        g_tc.checks++;                                                         \
        if (_a != _e) {                                                        \
            g_tc.failed++;                                                     \
            g_tc.case_failed = 1;                                              \
            printf("  [FAIL] %s:%d  %s  (actual=%lu expect=%lu)\n",            \
                   __FILE__, __LINE__, (msg), _a, _e);                         \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_EQ_FLOAT(actual, expect, tol, msg)                         \
    do {                                                                       \
        double _a = (double)(actual);                                          \
        double _e = (double)(expect);                                          \
        double _t = (double)(tol);                                             \
        double _d = (_a > _e) ? (_a - _e) : (_e - _a);                          \
        g_tc.checks++;                                                         \
        if (_d > _t) {                                                         \
            g_tc.failed++;                                                     \
            g_tc.case_failed = 1;                                              \
            printf("  [FAIL] %s:%d  %s  (actual=%f expect=%f tol=%f)\n",       \
                   __FILE__, __LINE__, (msg), _a, _e, _t);                     \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_EQ_STR(actual, expect, msg)                                \
    do {                                                                       \
        const char *_a = (actual);                                             \
        const char *_e = (expect);                                             \
        g_tc.checks++;                                                         \
        if ((_a == NULL) || (_e == NULL) || (strcmp(_a, _e) != 0)) {            \
            g_tc.failed++;                                                     \
            g_tc.case_failed = 1;                                              \
            printf("  [FAIL] %s:%d  %s  (actual=\"%s\" expect=\"%s\")\n",      \
                   __FILE__, __LINE__, (msg), (_a ? _a : "(null)"),           \
                   (_e ? _e : "(null)"));                                      \
        }                                                                      \
    } while (0)

#define TEST_END() do { (void)test_suite_end(); } while (0)

/* 各测试套件入口（run_tests.c 逐个调用） */
int test_frame_codec_run(void);
int test_step_counter_run(void);
int test_ws_client_run(void);
int test_power_mgr_run(void);
int test_drivers_run(void);
int test_uplink_run(void);

#endif /* TEST_UTIL_H */

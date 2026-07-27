#include "unity.h"
#include "web/web.h"
#include "web/http_parse.h"
#include "web/rest.h"
#include "web/wss.h"
#include "web/auth_web.h"
void test_placeholder(void) { TEST_ASSERT_TRUE(1); }
int main(void) { UNITY_BEGIN(); RUN_TEST(test_placeholder); return UNITY_END(); }

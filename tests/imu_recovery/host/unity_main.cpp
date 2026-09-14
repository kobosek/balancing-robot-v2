#include "unity.h"

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UnityBegin("host-audit");
    for (const auto& test : auditTests()) {
        Unity.TestFile = test.file;
        UnityDefaultTestRun(test.run, test.name, test.line);
    }
    return UnityEnd();
}

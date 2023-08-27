#include "unit_test/unit_test.hpp"

int main () {

    UnitTest ut;
    ut.test_mat_object();

    ut.test_IoU();

    return 0;
}
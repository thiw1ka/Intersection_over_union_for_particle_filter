#include <iostream>
#include <intersection_over_union/intersection_over_union.hpp>

struct UnitTest {

    IntersectionOverUnion iou;

    void test_mat_object ();

    void test_IoU ();

    void show_canvas (cv::Mat* array_of_canvas[], std::string name [],int l) const;
};
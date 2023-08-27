#include "unit_test/unit_test.hpp"

void UnitTest::show_canvas (cv::Mat* array_of_canvas[],std::string name[], int l) const {
    // auto arr_canSz = sizeof(array_of_canvas) / sizeof(array_of_canvas[0]);
    // auto arr_nameSz = sizeof(name)/sizeof(name[0]);
    // std::cout << "array quantity " << arr_nameSz; 
    // if (arr_canSz != arr_nameSz ) {
    //     printf("[show_canvas] Error array dont match");
    //     exit(0);
    // }
    for (int i = 0; i < l; i++) {
            cv::imshow(name[i], *array_of_canvas[i]);
    }    
    cv::waitKey(0);
    cv::destroyAllWindows();
}


void UnitTest::test_mat_object () {
    std::printf("[test_mat_object] test starting =========\n");

    auto canvas = iou.getOpenCVMatObject(400, 400);
    std::vector<cv::Point> pointList = {cv::Point(200,200)};
    iou.drawCirclesInCanvas(canvas,pointList,10);
    cv::Mat* pointerlist [1] = {&canvas};
    std::string stringlist [1] = {std::string("Unit test")};
    show_canvas(pointerlist, stringlist, 1);
    // cv::imshow("Unit test",canvas);
    // cv::waitKey(0);
    // cv::destroyAllWindows();
    std::printf("[test_mat_object] test ending =========\n");

}

void UnitTest::test_IoU (){
    std::printf("[test_IoU] test starting =========\n");

    /*ground truth*/
    auto ground_truth = iou.getOpenCVMatObject(400, 400);
    std::vector<cv::Point> pointList = {cv::Point(200,200)};
    iou.drawCirclesInCanvas(ground_truth,pointList,10);

    /*filter result*/
    auto filter_result = iou.getOpenCVMatObject(400, 400);
    iou.drawCirclesInCanvas(filter_result,pointList,10);

    auto result = iou.getOpenCVMatObject(400, 400);

    //result should be one
    auto iou_value = iou.calculateIoU(ground_truth, filter_result);
    std::printf("[test_IoU] 1.test 2 circles overlaps 1 == (%f) \n", iou_value);
    cv::add(ground_truth, filter_result, result);
    int canvas_count = 3;
    cv::Mat* pointerlist [canvas_count] = {&ground_truth, &filter_result , &result};
    std::string stringlist [canvas_count] = {std::string("ground_truth"), std::string("filter_result"), std::string("union")};
    
    show_canvas(pointerlist, stringlist, canvas_count);

    //result should be 0.6
    //three same circles and two overlaps
    pointList.push_back(cv::Point(100,100));
    iou.drawCirclesInCanvas(filter_result,pointList,10);
    iou_value = iou.calculateIoU(ground_truth, filter_result);
    std::printf("[test_IoU] 2.test- 2 out of 3 circles overlaps. answer 0.6 == (%f) \n", iou_value);
    cv::add(ground_truth, filter_result, result);
    show_canvas(pointerlist, stringlist, canvas_count);

    //partially overlaps
    pointList.pop_back();
    pointList.push_back(cv::Point(90,100));
    iou.drawCirclesInCanvas(ground_truth,pointList,10);
    iou_value = iou.calculateIoU(ground_truth, filter_result);
    std::printf("[test_IoU] 3.test- 2 and almost half of a circle out of 4 circles overlaps. answer 0.7 == (%f) \n", iou_value);
    cv::add(ground_truth, filter_result, result);
    show_canvas(pointerlist, stringlist, canvas_count);

    // cv::imshow("Unit test",canvas);
    // cv::waitKey(0);
    std::printf("[test_IoU] test end =========\n");


}
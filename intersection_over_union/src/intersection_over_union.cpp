#include "intersection_over_union/intersection_over_union.hpp"

/*static values*/
int IntersectionOverUnion::pixel_val_for_circle = 100;
int IntersectionOverUnion::CIRCLE_RAD_ = 5;
cv::Point IntersectionOverUnion::canvas_size(1000, 1000); 

IntersectionOverUnion::IntersectionOverUnion() {
    std::printf("[IntersectionOverUnion] Initiating intersection over union \n");
    ///TODO set radius for each point
    ///set pixel value. Maybe from a launch file?
}

cv::Mat IntersectionOverUnion::getOpenCVMatObject (int l, int w) {
    return cv::Mat::zeros(w, l, CV_8UC1);
    // return std::move(canvas);
}

void IntersectionOverUnion::drawCirclesInCanvas (cv::Mat& canvas,std::vector<cv::Point> center_locations, int radius) {
    for (const auto& center : center_locations){
        cv::circle(canvas,center, radius, pixel_val_for_circle,-1); //-1 to fill the circle
    }
}

double IntersectionOverUnion::calculateIoU (const cv::Mat& ground_truth, const cv::Mat& filter_output) {
    std::printf("[calculateIoU] calcualting intersection over union \n");
    double iou_value = 0.0;
    auto canvas_size = ground_truth.size();
    // cv::Mat canvas_with_sum_val(canvas_size, CV_8UC1);
    // cv::add(ground_truth, filter_output, canvas_with_sum_val);


    auto no_of_pixel_union = (double) cv::countNonZero (ground_truth);
    no_of_pixel_union += (double) cv::countNonZero (filter_output);
    cv::Mat canvas_with_intersection (canvas_size, CV_8UC1);
    cv::absdiff(ground_truth, filter_output, canvas_with_intersection);
    auto non_overlapping_area = (double) cv::countNonZero(canvas_with_intersection);
    auto no_of_pixels_intersect = no_of_pixel_union - non_overlapping_area;
    // cv::imshow("canvas_with_sum_val",canvas_with_sum_val);
    // cv::imshow("canvas_with_intersection",canvas_with_intersection);
    // cv::waitKey(0);

    iou_value = no_of_pixel_union > 0 ? no_of_pixels_intersect / no_of_pixel_union : 0.0; //make sure it is not dvided by 0; 
    std::printf("[calculateIoU] iou_value(%f) = no_of_pixels_intersect(%f)/no_of_pixel_union(%f)  \n", 
                                    iou_value,no_of_pixels_intersect, no_of_pixel_union);
    return iou_value;
}

double IntersectionOverUnion::getIoUForTwoPointsList (const std::vector<cv::Point>& a,const std::vector<cv::Point>& b, cv::Point area_dimensions, int pt_radius) {
    std::printf("[getIoUForTwoPointsList] a_list_size(%i), b_list_size(%i), area_dimensions-w(%i)xl(%i), circle radius(%i) \n",
                                                a,b, int(area_dimensions.x), int(area_dimensions.y), pt_radius);
    /*get two canvas with same dimensions*/
    auto a_canvas = getOpenCVMatObject(area_dimensions.x, area_dimensions.y);
    auto b_canvas = getOpenCVMatObject(area_dimensions.x, area_dimensions.y);
    /*draw each point list in seperate canvas*/
    drawCirclesInCanvas(a_canvas, a, pt_radius);
    drawCirclesInCanvas(b_canvas, b, pt_radius);
    /*calculate the IoU*/
    auto result = calculateIoU (a_canvas, b_canvas);
    std::printf("[getIoUForTwoPointsList] Finished calcualting IoU. IoU = %f \n", result);
    return result;
}

/*using point clouds*/
double IntersectionOverUnion::getIoUForTwoPointclouds ( const sensor_msgs::PointCloud& filter_output, 
                                                        const sensor_msgs::PointCloud& groundtruth, 
                                                        cv::Point area_dimensions, int pt_radius, bool showPoints) {
    std::printf("[getIoUForTwoPointsList] calcualting IoU started. \n");
    auto filter_output_canvas = getOpenCVMatObject(area_dimensions.x, area_dimensions.y);
    auto groundtruth_canvas = getOpenCVMatObject(area_dimensions.x, area_dimensions.y);
    /*converting to canvas coordinate which is (0,0) stating from left upper corner*/
    /*because points from sim is lake centered but canvas coordinates start from left upper corner*/
    auto translated_pt_via_left_upper_to_center = [&] (decltype(filter_output.points)::value_type pt) {
        static int center_px_x = area_dimensions.x /2;
        static int center_px_y = area_dimensions.y /2;
        int x = int(pt.x);
        int y = int(pt.y);
        cv::Point return_point (center_px_x + x, center_px_y -y);
        // std::printf("[getIoUForTwoPointsList::lambda_translator] convertED_pt(%i,%i) =  Orig_ptx(center_px_x(%i) + %i), org_pt_y(center_px_y(%i) - %i), center_px_x (%i /2), center_px_y (%i /2) \n",
        //                                                         return_point.x, return_point.y, center_px_x, x, center_px_y, y, area_dimensions.x,  area_dimensions.y);
        /*making sure points are inside the canvas*/
        if(return_point.x > area_dimensions.x || return_point.y > area_dimensions.y || return_point.x*return_point.y < 0) {
            std::printf("[getIoUForTwoPointsList::lambda_translator] ERROR: return_point.x(%i)> area_dimensions.x(%i) || return_point.y(%i) > area_dimensions.y(%i) || return_point.x*return_point.y < 0 \n",
                                                                                return_point.x, area_dimensions.x, return_point.y, area_dimensions.y);
            throw std::invalid_argument("[IntersectionOverUnion::getIoUForTwoPointclouds] Converted pixel points outside canvas. Maybe increase Canvas size.");
        }
        return return_point;
    };
    std::vector<cv::Point> vec_a, vec_b;
    /*filter output drawn to canvas*/
    for (auto const& pt : filter_output.points) {
        /*used weight to multiply the radius. so if weight is 2 then rad = original * 2 */
        /*CONFIRM WITH PAOLO{Confirmed}, radius of point is multiplied by weight value*/
        auto pt_radius_multiplied_by_weight = pt.z > 1.0 ?  pt_radius + (pt_radius *  (pt.z - 1.0) / 10) : pt_radius; 
        // = pt_radius +  pt_radius * int(pt.z);
        // std::printf("[getIoUForTwoPointsList] filter_output: pt_radius_multiplied_by_weight(%i) = pt_radius(%i) * pt.z(%i)  \n",
        //                                            pt_radius_multiplied_by_weight, pt_radius, pt.z);
        cv::Point converted_to_px_pt = translated_pt_via_left_upper_to_center(pt);
        cv::circle(filter_output_canvas, converted_to_px_pt, pt_radius_multiplied_by_weight, pixel_val_for_circle,-1); //-1 to fill the circle
    }
    /*no weight is consider here*/
    for (auto const& pt : groundtruth.points) {
        cv::Point converted_to_px_pt = translated_pt_via_left_upper_to_center(pt);
        // std::printf("[getIoUForTwoPointsList] groundtruth: pt.z(%i)  \n",
        //                                            int(pt.z));
        cv::circle(groundtruth_canvas, converted_to_px_pt, pt_radius, pixel_val_for_circle,-1); //-1 to fill the circle
    }
    /*calculate the IoU*/
    auto result = calculateIoU (groundtruth_canvas, filter_output_canvas);
    std::printf("[getIoUForTwoPointsList] Finished calcualting IoU. IoU = %f \n", result);
    /*showing two windows if required*/
    if (showPoints) {
        cv::imshow("filter_output_canvas", filter_output_canvas);
        cv::imshow("groundtruth_canvas", groundtruth_canvas);
        cv::waitKey(800);
        ///TODO SAVE images// cv::imwrite()
    }
    if (showPoints) {
        cv::destroyAllWindows();
    }
    return result;
}
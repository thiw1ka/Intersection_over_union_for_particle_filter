#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/PointCloud.h"

class IntersectionOverUnion {

    public:
        IntersectionOverUnion(); ///TODO: Implement what to take in construction

        /**
         * @brief canvas size
         * 
         * @param a W
         * @param b L
         */
        void setAreaOfImage(int a, int b) {
            canvas_size = cv::Point(a, b);
        };

        void setSinglePointRadius (int a) {
             CIRCLE_RAD_ = a;
        };

        /**
         * @brief calculate IoU when two lists are provided
         * 
         * @param a point list 1
         * @param b point list 2
         * @param area_dimensions area cv::Point(W, L)
         * @param pt_radius radius
         * @return double IoU
         */
        virtual double getIoUForTwoPointsList (const std::vector<cv::Point>& a, const std::vector<cv::Point>& b, cv::Point area_dimensions, int pt_radius);

        double getIoUForTwoPointclouds (const sensor_msgs::PointCloud& filter_output, 
                                        const sensor_msgs::PointCloud& groundtruth, 
                                        cv::Point area_dimensions = canvas_size, 
                                        int pt_radius = CIRCLE_RAD_,
                                        bool showPoints = false
                                        );

    private:
        
        friend class UnitTest;

    protected:

        /**
         * @brief area cv::Point(W, L)
         * 
         */
        static cv::Point canvas_size;

        /**
         * @brief value for each pixel in the circle
         * 
         */
        static int pixel_val_for_circle;

        static int CIRCLE_RAD_;

        /**
         * @brief create a canvas to draw particles.
         * 
         * @param l length of the canvas
         * @param w width of the canvas
         * @return cv::Mat 
         */
        cv::Mat getOpenCVMatObject (int l, int w);

        /**
         * @brief draw circles using the point provided
         * 
         * @param canvas 
         * @param center_locations list of points to draw 
         * @param radius 
         */
        void drawCirclesInCanvas (cv::Mat& canvas, std::vector<cv::Point> center_locations, int radius);

        /**
         * @brief calculating Intersection over union value
         * 
         * @param ground_truth ground truth image
         * @param filter_output filter result image
         * @return double IOU value 0.0 - 1.0
         */
        double calculateIoU (const cv::Mat& ground_truth, const cv::Mat& filter_output);


};
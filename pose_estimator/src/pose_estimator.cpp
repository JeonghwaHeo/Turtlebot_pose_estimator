#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <array>
#include <vector>

constexpr float max_range = 0.8;
constexpr float threshold = 0.001;

class LaserScanListener : public rclcpp::Node
{
public:
    LaserScanListener() : Node("laser_scan_listener")
    {
        rclcpp::QoS qos_profile = rclcpp::QoS(10).reliable().best_effort();
        
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile, std::bind(&LaserScanListener::scan_callback, this, std::placeholders::_1));
    }

private:
    struct Line
    {
        float a;
        float b;
        float c;
    };

    using Point = std::pair<float, float>;

    std::pair<Line, Line> calculate_lines_from_three_points(const Point& p1, const Point& p2, const Point& p3)
    {
        // Line 1: a1x + b1y + c1 = 0, through p1 and p2
        float a1 = p2.second - p1.second;
        float b1 = p1.first - p2.first;
        float c1 = -(a1 * p1.first + b1 * p1.second);

        // Line 2: a2x + b2y + c2 = 0, perpendicular from Line 1 and through p3
        float a2 = -b1;
        float b2 = a1;
        float c2 = -(a2 * p3.first + b2 * p3.second);

        return {{a1, b1, c1}, {a2, b2, c2}};
    }

    float calculate_distance_to_line(const Point& p, const Line& line)
    {
        return std::abs(line.a * p.first + line.b * p.second + line.c) /
               std::sqrt(line.a * line.a + line.b * line.b);
    }

    size_t find_best_combination(const std::array<Point, 360>& positions, size_t count, float threshold)
    {
        size_t max_count = 0;
        Line best_line1, best_line2;

        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                for (size_t k = 0; k < count; ++k) {
                    if (k == i || k == j)
                        continue;

                    auto [line1, line2] = calculate_lines_from_three_points(positions[i], positions[j], positions[k]);

                    size_t inliers = 0;
                    for (size_t l = 0; l < count; ++l) {
                        if (calculate_distance_to_line(positions[l], line1) <= threshold ||
                            calculate_distance_to_line(positions[l], line2) <= threshold) {
                            ++inliers;
                        }
                    }

                    if (inliers > max_count) {
                        max_count = inliers;
                        best_line1 = line1;
                        best_line2 = line2;
                    }
                }
            }
        }

        std::cout << "Number of points: " << count << std::endl;
        std::cout << "Maximum  Inliers: " << max_count << std::endl;
        std::cout << "Best combination of lines:" << std::endl;
        std::cout << "Line 1: y = " << - best_line1.a / best_line1.b << " x + " << - best_line1.c / best_line1.b << std::endl;
        std::cout << "Line 2: y = " << - best_line2.a / best_line2.b << " x + " << - best_line2.c / best_line2.b << std::endl;
        std::cout << "\n" << std::endl;

        return max_count;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        constexpr size_t max_points = 360;
        std::array<Point, max_points> positions;
        size_t count = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float distance = msg->ranges[i];

            if (distance > 0 && distance <= max_range) {
                float x = distance * cos(angle);
                float y = distance * sin(angle);
                positions[count++] = {x, y};
            }
        }

        find_best_combination(positions, count, threshold);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanListener>());
    rclcpp::shutdown();
    return 0;
}

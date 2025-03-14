#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <omp.h>
#include <cmath>
#include <array>
#include <algorithm>

double pose_estimation = M_PI / 2;
float robot_center_x, robot_center_y;

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

    float max_range = 0.7;
    float threshold_line = 0.0012;
    float threshold_proj = 0.003;
    float length_short = 0.145;
    float length_long = 0.225;

    //for DBSCAN
    float eps = 0.15;
    int minPts = 3;

    Line best_line1, best_line2;

    double rad_to_deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    double angle_regulation(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle <= -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    double angle_diff(double angle1, double angle2)
    {
        return std::abs(angle_regulation(angle1 - angle2));
    }

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

    Point project_point_onto_line(const Point& p, const Line& line)
    {
        float x0 = p.first;
        float y0 = p.second;
        float a = line.a;
        float b = line.b;
        float c = line.c;

        float denominator = a * a + b * b;
        if (denominator == 0) {
            throw std::runtime_error("Invalid line parameters (a and b cannot both be zero).");
        }

        float x_proj = x0 - (a * (a * x0 + b * y0 + c)) / denominator;
        float y_proj = y0 - (b * (a * x0 + b * y0 + c)) / denominator;

        return {x_proj, y_proj};
    }

    Point intersection_line(const Line &line1, const Line &line2)
    {
        float det = line1.a * line2.b - line2.a * line1.b;
        
        float x = (line2.b * (-line1.c) - line1.b * (-line2.c)) / det;
        float y = (line1.a * (-line2.c) - line2.a * (-line1.c)) / det;
        
        return std::make_pair(x, y);
    }

    Point find_center_point_1(const std::vector<Point>& positions, Line line, float threshold, float distance)
    {
        Point result_point{0.0f, 0.0f};
        std::vector<Point> projections;
        
        // positions.size()를 이용하여 반복
        for (size_t i = 0; i < positions.size(); i++) {
            if (calculate_distance_to_line(positions[i], line) <= threshold) {
                Point projected_point = project_point_onto_line(positions[i], line);
                projections.push_back(projected_point);
            }
        }

        if (projections.size() >= 2) {
            float max_dist = 0.0f;
            Point farthest_p1, farthest_p2;

            for (size_t i = 0; i < projections.size(); i++) {
                for (size_t j = i + 1; j < projections.size(); j++) {
                    float dist = std::hypot(projections[i].first - projections[j].first,
                                            projections[i].second - projections[j].second);
                    if (dist > max_dist) {
                        max_dist = dist;
                        farthest_p1 = projections[i];
                        farthest_p2 = projections[j];
                    }
                }
            }

            float center_x = (farthest_p1.first + farthest_p2.first) / 2.0f;
            float center_y = (farthest_p1.second + farthest_p2.second) / 2.0f;

            float direction_x = line.a;
            float direction_y = line.b;
            float length = std::hypot(direction_x, direction_y);
            direction_x /= length;
            direction_y /= length;

            if (center_x * direction_x + center_y * direction_y < 0) {
                direction_x = -direction_x;
                direction_y = -direction_y;
            }

            result_point = {center_x + distance * direction_x, center_y + distance * direction_y};
        }

        return result_point;
    }

    Point find_center_point_2(Line line1, Line line2, float distance1, float distance2)
    {
        Point result_point;

        float intersect_x = intersection_line(line1, line2).first;
        float intersect_y = intersection_line(line1, line2).second;

        float dir1_x = line1.b, dir1_y = -line1.a;
        float dir2_x = line2.b, dir2_y = -line2.a;

        float length1 = std::hypot(dir1_x, dir1_y);
        dir1_x /= length1;
        dir1_y /= length1;

        float length2 = std::hypot(dir2_x, dir2_y);
        dir2_x /= length2;
        dir2_y /= length2;

        if (intersect_x * dir1_x + intersect_y * dir1_y < 0) {
            dir1_x = -dir1_x;
            dir1_y = -dir1_y;
        }
        if (intersect_x * dir2_x + intersect_y * dir2_y < 0) {
            dir2_x = -dir2_x;
            dir2_y = -dir2_y;
        }

        float result_x = intersect_x + distance1 * dir1_x + distance2 * dir2_x;
        float result_y = intersect_y + distance1 * dir1_y + distance2 * dir2_y;

        result_point = {result_x, result_y};

        return result_point;
    }

    void find_best_combination(const std::array<Point, 360>& positions, size_t count, float threshold)
    {
        size_t max_count = 0;

        //OpenMP multi-thread for loop
        #pragma omp parallel
        {
            size_t local_max_count = 0;
            Line local_best_line1, local_best_line2;

            #pragma omp for
            for (size_t i = 0; i < count; i++) {
                for (size_t j = i + 1; j < count; j++) {
                    for (size_t k = 0; k < count; k++) {
                        if (k == i || k == j)
                            continue;

                        auto [line1, line2] = calculate_lines_from_three_points(positions[i], positions[j], positions[k]);

                        size_t inliers = 0;
                        for (size_t l = 0; l < count; l++) {
                            float d1 = calculate_distance_to_line(positions[l], line1);
                            float d2 = calculate_distance_to_line(positions[l], line2);
                            if (d1 <= threshold || d2 <= threshold) {
                                inliers++;
                            }
                        }

                        if (inliers > local_max_count) {
                            local_max_count = inliers;
                            local_best_line1 = line1;
                            local_best_line2 = line2;
                        }
                    }
                }
            }

            #pragma omp critical
            {
                if (local_max_count > max_count) {
                    max_count = local_max_count;
                    best_line1 = local_best_line1;
                    best_line2 = local_best_line2;
                }
            }
        }


        //Estimation of relative pose starting from the beginning
        double pose1 = atan2(-best_line1.a, best_line1.b);
        double pose2 = atan2(-best_line2.a, best_line2.b);

        std::array<double, 4> possible_poses = 
        {angle_regulation(pose1), angle_regulation(pose1 + 0.5 * M_PI),
         angle_regulation(pose1 + M_PI), angle_regulation(pose1 + 1.5 * M_PI)};

        double closest_pose = possible_poses[0];
        double min_pose_diff = angle_diff(possible_poses[0], pose_estimation);

        for (size_t i = 0; i < possible_poses.size(); i++) {
            double diff = angle_diff(possible_poses[i], pose_estimation);

            if (diff < min_pose_diff) {
                closest_pose = possible_poses[i];
                min_pose_diff = diff;
            }
        }

        pose_estimation = closest_pose;


        //Determination of long, short line
        if (std::min(angle_diff(pose1, pose_estimation), angle_diff(pose1 + M_PI, pose_estimation)) >
        std::min(angle_diff(pose2, pose_estimation), angle_diff(pose2 + M_PI, pose_estimation))) {
            Line line_temp = best_line1;
            best_line1 = best_line2;
            best_line2 = line_temp;

            double pose_temp = pose1;
            pose1 = pose2;
            pose2 = pose_temp;
        }

        std::cout << "_____________________________________\n\n" << std::endl;
        std::cout << "Number of points: " << count << std::endl;
        std::cout << "Maximum  Inliers: " << max_count << "\n" << std::endl;
        std::cout << "Best combination of lines:" << std::endl;
        std::cout << "Line 1: y = " << - best_line1.a / best_line1.b << " x " << - best_line1.c / best_line1.b << std::endl;
        std::cout << "Line 2: y = " << - best_line2.a / best_line2.b << " x " << - best_line2.c / best_line2.b << "\n" << std::endl;
        std::cout << "Pose estimation: " << rad_to_deg(pose_estimation) << "deg\n" <<std::endl;
        //std::cout << "Pose 1: " << rad_to_deg(pose1) << "deg" <<std::endl;
        //std::cout << "Pose 2: " << rad_to_deg(pose2) << "deg\n" <<std::endl;
    }

    std::vector<Point> performDBSCAN(const std::array<Point, 360>& points, size_t count, float eps, int minPts)
    {
        std::vector<std::vector<Point>> clusters;
        std::vector<bool> visited(count, false);
        // cluster_ids[i] == -1 -> noise, more than 0 -> cluster_id
        std::vector<int> cluster_ids(count, -1);
        int cluster_id = 0;

        // distance between points
        auto distance = [&](int i, int j) -> float {
            float dx = points[i].first - points[j].first;
            float dy = points[i].second - points[j].second;
            return std::hypot(dx, dy);
        };

        // regionQuery: return neighbor index lists
        auto regionQuery = [&](int idx) -> std::vector<int> {
            std::vector<int> neighbors;
            for (size_t j = 0; j < count; j++) {
                if (distance(idx, j) <= eps) {
                    neighbors.push_back(j);
                }
            }
            return neighbors;
        };

        // DBSCAN clustering
        for (size_t i = 0; i < count; i++) {
            if (visited[i])
                continue;
            visited[i] = true;
            std::vector<int> neighborPts = regionQuery(i);

            if (neighborPts.size() < static_cast<size_t>(minPts)) {
                // if noise (cluster_ids[i] is already -1)
                continue;
            } else {
                // new cluster
                cluster_ids[i] = cluster_id;
                std::vector<int> seeds = neighborPts;
                for (size_t j = 0; j < seeds.size(); j++) {
                    int curr_idx = seeds[j];
                    if (!visited[curr_idx]) {
                        visited[curr_idx] = true;
                        std::vector<int> result = regionQuery(curr_idx);
                        if (result.size() >= static_cast<size_t>(minPts)) {
                            // add neighbor to seeds
                            seeds.insert(seeds.end(), result.begin(), result.end());
                        }
                    }
                    if (cluster_ids[curr_idx] == -1) {
                        cluster_ids[curr_idx] = cluster_id;
                    }
                }
                cluster_id++; // next cluster_id
            }
        }

        // classify points by cluster
        clusters.resize(cluster_id);
        for (size_t i = 0; i < count; i++) {
            int id = cluster_ids[i];
            if (id != -1) { // if not noise
                clusters[id].push_back(points[i]);
            }
        }

        std::cout << "Total DBSCAN cluster number: " << clusters.size() << "\n" << std::endl;
        for (size_t i = 0; i < clusters.size(); i++) {
            std::cout << "Cluster " << i << ": " << clusters[i].size() << " number of points" << std::endl;
        }

        // choose cluster of minimum average of min distance
        std::vector<Point> bestCluster;
        float bestAvg = std::numeric_limits<float>::max();

        for (size_t i = 0; i < clusters.size(); i++) {
            float sumDistances = 0.0f;
            for (const auto& p : clusters[i]) {
                float d1 = calculate_distance_to_line(p, best_line1);
                float d2 = calculate_distance_to_line(p, best_line2);
                sumDistances += std::min(d1, d2);
            }
            float avgDistance = sumDistances / clusters[i].size();
            std::cout << "Avg min distance of Cluster " << i << ": " << avgDistance << std::endl;
            if (avgDistance < bestAvg) {
                bestAvg = avgDistance;
                bestCluster = clusters[i];
            }
        }

        std::cout << "" << std::endl;

        return bestCluster;
    }

    void find_center_point(const std::vector<Point>& positions)
    {
        Point estimated_center{0.0f, 0.0f};
        size_t proj_count = 0;

        for (size_t i = 0; i < positions.size(); i++) {
            if (calculate_distance_to_line(positions[i], best_line1) <= threshold_proj) {
                Point projected_point = project_point_onto_line(positions[i], best_line1);
                estimated_center.first += projected_point.first;
                estimated_center.second += projected_point.second;
                proj_count++;
            }
            else if (calculate_distance_to_line(positions[i], best_line2) <= threshold_proj) {
                Point projected_point = project_point_onto_line(positions[i], best_line2);
                estimated_center.first += projected_point.first;
                estimated_center.second += projected_point.second;
                proj_count++;
            }
        }

        if (proj_count > 0) {
            estimated_center.first /= proj_count;
            estimated_center.second /= proj_count;
        } else {
            std::cerr << "Projection count is zero. Center point estimation failed." << std::endl;
            return;
        }

        // Estimation of center point at different conditions
        double pose_deg = rad_to_deg(angle_regulation(pose_estimation - atan2(estimated_center.second, estimated_center.first)));

        if ((pose_deg >= -30 && pose_deg < 30) ||
            (pose_deg >= 150 && pose_deg <= 180) ||
            (pose_deg > -180 && pose_deg < -150)) { // use only short line
            std::cout << "                SHORT" << std::endl;
            // find_center_point_1 함수의 입력도 vector로 전달 (두번째 인자는 positions.size())
            Point center_point = find_center_point_1(positions, best_line2, threshold_proj, length_long / 2);
            robot_center_x = center_point.first;
            robot_center_y = center_point.second;
        }
        else if ((pose_deg >= 60 && pose_deg < 120) ||
                (pose_deg >= -120 && pose_deg < -60)) { // use only long line
            std::cout << "LONG" << std::endl;
            Point center_point = find_center_point_1(positions, best_line1, threshold_proj, length_short / 2);
            robot_center_x = center_point.first;
            robot_center_y = center_point.second;
        }
        else if ((pose_deg >= 30 && pose_deg < 60) ||
                (pose_deg >= 120 && pose_deg < 150) ||
                (pose_deg >= -150 && pose_deg < -120) ||
                (pose_deg >= -60 && pose_deg < -30)) { // use both line
            std::cout << "        BOTH" << std::endl;
            Point center_point = find_center_point_2(best_line1, best_line2, length_long / 2, length_short / 2);
            robot_center_x = center_point.first;
            robot_center_y = center_point.second;
        }

        std::cout.precision(4);
        std::cout << std::fixed << std::showpos << "Center point: (" << robot_center_x << ", " << robot_center_y << ")" << std::endl;
        std::cout << "\n" << std::endl;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // input points near than max_range
        constexpr size_t max_points = 360;
        std::array<Point, max_points> point_positions;
        size_t count = 0;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float distance = msg->ranges[i];

            if (distance > 0 && distance <= max_range) {
                float x = distance * cos(angle);
                float y = distance * sin(angle);
                point_positions[count++] = {x, y};
            }
        }

        // find best line combination
        find_best_combination(point_positions, count, threshold_line);

        // Use only correct point cluster using DBSCAN
        float eps = 0.15;
        int minPts = 3;

        std::vector<Point> cluster = performDBSCAN(point_positions, count, eps, minPts);

        // find center point
        find_center_point(cluster);
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

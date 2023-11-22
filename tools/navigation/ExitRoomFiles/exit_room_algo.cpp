//exit-includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <numeric>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>
#include <thread>

//exit includes

//exit - classes
class DataReader {
public:
    std::vector<std::vector<double>> read_file(const std::string& file_path) {
        std::string file_extension = get_file_extension(file_path);
        if (file_extension == ".xyz") {
            return read_xyz(file_path);
        } else if (file_extension == ".csv") {
            return read_csv(file_path);
        } else {
            throw std::invalid_argument("Unsupported file type. Only .xyz and .csv files are supported.");
        }
    }

private:
    std::string get_file_extension(const std::string& file_path) {
        size_t pos = file_path.find_last_of('.');
        if (pos != std::string::npos) {
            return file_path.substr(pos);
        }
        return "";
    }

    std::vector<std::vector<double>> read_xyz(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file.");
        }

        std::vector<std::vector<double>> data_points;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::vector<double> values;
            double value;
            while (iss >> value) {
                values.push_back(value);
            }
            if (values.size() >= 3) {
                data_points.push_back({values[0], values[1], values[2]});
            }
        }
        return data_points;
    }

    std::vector<std::vector<double>> read_csv(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file.");
        }

        std::vector<std::vector<double>> data_points;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string value_str;
            std::vector<double> values;
            while (std::getline(iss, value_str, ',')) {
                values.push_back(std::stod(value_str));
            }
            if (values.size() >= 3) {
                data_points.push_back({values[0], values[1], values[2]});
            }
        }
        return data_points;
    }
};
class DataProcessor {
public:
    std::vector<std::vector<double>> calculate_zscores(const std::vector<std::vector<double>>& data_points) {
        std::vector<std::vector<double>> zscores(data_points[0].size(), std::vector<double>(data_points.size()));

        for (size_t i = 0; i < data_points[0].size(); ++i) {
            double mean = 0.0;
            for (const auto& point : data_points) {
                mean += point[i];
            }
            mean /= data_points.size();

            double stddev = 0.0;
            for (const auto& point : data_points) {
                stddev += (point[i] - mean) * (point[i] - mean);
            }
            stddev = sqrt(stddev / (data_points.size() - 1));

            for (size_t j = 0; j < data_points.size(); ++j) {
                zscores[i][j] = fabs((data_points[j][i] - mean) / stddev);
            }
        }

        return zscores;
    }

    std::vector<std::vector<double>> clean_data(const std::vector<std::vector<double>>& data_points, double zscore_threshold = 3.0) {
        
        std::vector<std::vector<double>> zscores = calculate_zscores(data_points);

        
        std::vector<bool> is_outlier = find_outliers(zscores, zscore_threshold);

        
        std::vector<std::vector<double>> filtered_points;
        for (size_t i = 0; i < data_points.size(); ++i) {
            if (!is_outlier[i]) {
                filtered_points.push_back(data_points[i]);
            }
        }

        if (filtered_points.empty()) {
            throw std::invalid_argument("All data points are identified as outliers.");
        }

        return filtered_points;
    }

private:

    std::vector<bool> find_outliers(const std::vector<std::vector<double>>& zscores, double threshold) {
        std::vector<bool> is_outlier(zscores[1].size(), false);

        for (size_t i = 0; i < zscores.size(); ++i) {
        for (size_t j = 0; j < zscores[i].size(); ++j) {
            if (std::abs(zscores[i][j]) > threshold) {
                is_outlier[j] = true;
                break; // Exit the inner loop once an outlier is found for this data point
            }
        }
    }
        return is_outlier;
    }
};
class DataAnalyzer {
public:
    std::vector<double> find_middle_point(const std::vector<std::vector<double>>& data_points) {
        std::vector<double> min_point(data_points[0].size(), std::numeric_limits<double>::max());
        std::vector<double> max_point(data_points[0].size(), std::numeric_limits<double>::lowest());

        for (const auto& point : data_points) {
            for (size_t i = 0; i < point.size(); ++i) {
                min_point[i] = std::min(min_point[i], point[i]);
                max_point[i] = std::max(max_point[i], point[i]);
            }
        }

        std::vector<double> middle_point(data_points[0].size());
        for (size_t i = 0; i < middle_point.size(); ++i) {
            middle_point[i] = (min_point[i] + max_point[i]) / 2.0;
        }

        return middle_point;
    }
    std::vector<double> calculate_variances(const std::vector<std::vector<double>>& data_points) {
        size_t num_dimensions = data_points[0].size();
        std::vector<double> variances(num_dimensions, 0.0);

        for (size_t i = 0; i < num_dimensions; ++i) {
            double mean = 0.0;
            for (const auto& point : data_points) {
                mean += point[i];
            }
            mean /= data_points.size();

            double variance = 0.0;
            for (const auto& point : data_points) {
                variance += (point[i] - mean) * (point[i] - mean);
            }
            variance /= data_points.size();

            variances[i] = variance;
        }

        return variances;
    }
        std::vector<size_t> select_best_dimensions(const std::vector<std::vector<double>>& data_points) {
        std::vector<double> variances = calculate_variances(data_points);

        // Sort in descending order
        std::vector<size_t> sorted_indices(data_points[0].size());
        for (size_t i = 0; i < sorted_indices.size(); ++i) {
            sorted_indices[i] = i;
        }
        std::sort(sorted_indices.begin(), sorted_indices.end(), [&](size_t i, size_t j) {
            return variances[i] > variances[j];
        });

        // Select the dimensions with the highest variance
        std::vector<size_t> best_dimensions(sorted_indices.begin(), sorted_indices.begin() + 2);
        return best_dimensions;
    }
    std::vector<std::vector<double>> project_to_dimensions(const std::vector<std::vector<double>>& data_points, const std::vector<size_t>& dimensions) {
        std::vector<std::vector<double>> projected_data(data_points.size(), std::vector<double>(dimensions.size()));

        for (size_t i = 0; i < data_points.size(); ++i) {
            for (size_t j = 0; j < dimensions.size(); ++j) {
                projected_data[i][j] = data_points[i][dimensions[j]];
            }
        }

        return projected_data;
    }

    std::vector<size_t> count_nan_dist_neighbors(
        const std::vector<size_t>& nan_indices,
        const std::vector<double>& avg_distances,
        size_t buffer_size = 10
    ) {
        size_t array_length = avg_distances.size();
        std::vector<size_t> nan_counts(array_length, 0);

        for (size_t nan_index : nan_indices) {
            size_t left_neighbor = array_length;
            size_t right_neighbor = array_length;

            for (size_t offset = 1; offset <= buffer_size; ++offset) {
                size_t left_offset_index = (nan_index + array_length - offset) % array_length;
                size_t right_offset_index = (nan_index + offset) % array_length;

                if (left_neighbor == array_length && !std::isnan(avg_distances[left_offset_index])) {
                    left_neighbor = left_offset_index;
                }
                if (right_neighbor == array_length && !std::isnan(avg_distances[right_offset_index])) {
                    right_neighbor = right_offset_index;
                }
                if (left_neighbor != array_length && right_neighbor != array_length) {
                    break;
                }
            }

            if (left_neighbor == array_length) {
                left_neighbor = (nan_index + array_length - 1) % array_length;
            }
            if (right_neighbor == array_length) {
                right_neighbor = (nan_index + 1) % array_length;
            }

            nan_counts[nan_index] = right_neighbor > left_neighbor ? (right_neighbor - left_neighbor - 1) : 0;
        }

        return nan_counts;
    }
    double calculate_angle(const std::vector<double>& point1, const std::vector<double>& point2) {
    double delta_y = point1[1] - point2[1];
    double delta_x = point1[0] - point2[0];
    return std::atan2(delta_y, delta_x);
}

    std::vector<double> calculate_average_distances(const std::vector<std::vector<double>>& data_points, const std::vector<double>& drone_pos) {
    std::vector<double> distances;
    for (const auto& data_point : data_points) {
        double sum = 0.0;
        for (size_t i = 0; i < data_point.size(); ++i) {
            double diff = data_point[i] - drone_pos[i];
            sum += diff * diff;
    }
        distances.push_back(std::sqrt(sum));
    }
    std::vector<double> angles;
    for (const auto& data_point : data_points) {
        double angle = calculate_angle(data_point, drone_pos);
        angles.push_back(angle);
    }
    for (double& angle : angles) {
    angle = std::fmod(angle + 2 * M_PI, 2 * M_PI);
    }


size_t num_angles = 360;


std::vector<double> avg_distances(num_angles, std::numeric_limits<double>::quiet_NaN());


std::vector<int> count_datapoints_per_angle(num_angles, 0);
for (size_t i = 0; i < num_angles; ++i) {
    double angle_min = i * M_PI / 180.0;
    double angle_max = (i + 1) * M_PI / 180.0;
    std::vector<size_t> indices;

    
    for (size_t j = 0; j < angles.size(); ++j) {
        if (angles[j] >= angle_min && angles[j] < angle_max) {
            indices.push_back(j);
        }
    }

    size_t count = indices.size();
    count_datapoints_per_angle[i] = count;

    
    if (count > 1) {
        double sum_distances = 0.0;
        for (size_t index : indices) {
            sum_distances += distances[index];
        }
        avg_distances[i] = sum_distances / static_cast<double>(count);
    }
    
}
return avg_distances;
    }
    std::pair<std::vector<std::vector<double>>, std::vector<double>> find_exit(
    const std::vector<std::vector<double>>& datapoints,
    const std::vector<double>& drone_pos,
    const std::vector<double>& avg_distances
) {
    size_t num_angles = avg_distances.size();
    std::vector<double> angles(num_angles);

    for (size_t i = 0; i < num_angles; ++i) {
        angles[i] = i * 2.0 * M_PI / num_angles;
    }

    std::vector<size_t> nan_indices;

    for (size_t i = 0; i < avg_distances.size(); ++i) {
        if (std::isnan(avg_distances[i])) {
            nan_indices.push_back(i);
        }
    }

    if (nan_indices.empty()) {
        return std::make_pair(std::vector<std::vector<double>>(), std::vector<double>());
    }

    std::vector<size_t> nan_counts = count_nan_dist_neighbors(nan_indices, avg_distances);

    size_t max_count = 0;
    std::vector<size_t> max_count_indices;

    for (size_t i = 0; i < nan_counts.size(); ++i) {
        if (nan_counts[i] == max_count) {
            max_count_indices.push_back(i);
        } else if (nan_counts[i] > max_count) {
            max_count = nan_counts[i];
            max_count_indices.clear();
            max_count_indices.push_back(i);
        }
    }

    std::vector<size_t> max_count_indices_with_nan;

    for (size_t index : max_count_indices) {
        if (std::isnan(avg_distances[index])) {
            max_count_indices_with_nan.push_back(index);
        }
    }

    std::vector<std::vector<double>> exit_points;
    std::vector<double> exit_angles;

    for (size_t i = 0; i < max_count_indices_with_nan.size(); ++i) {
        size_t middle_index = max_count_indices_with_nan[i];
        size_t exit_angle_index = middle_index;
        double exit_angle = angles[exit_angle_index];

        double mean_distance = 0.0;
        for (const auto& point : datapoints) {
            mean_distance += std::sqrt(std::pow(point[0], 2) + std::pow(point[1], 2));
        }
        mean_distance /= datapoints.size();

        double exit_x = drone_pos[0] + std::cos(exit_angle) * mean_distance;
        double exit_y = drone_pos[1] + std::sin(exit_angle) * mean_distance;

        std::vector<double> exit_point = { exit_x, exit_y };

        exit_points.push_back(exit_point);
        exit_angles.push_back(exit_angle);
    }

    // Implement the remaining logic for finding the exit point with the highest score

    return std::make_pair(exit_points, exit_angles);
}


};
class AlgoRunner{
public:
    std::vector<double> findExit(std::vector<std::vector<double>> data_points, std::vector<double> drone_position){
        DataReader reader;
        DataProcessor processor;
        DataAnalyzer analyzer;
        std::vector<double> ret_point;
        try {
            std::vector<std::vector<double>> zscores = processor.calculate_zscores(data_points);
            std::vector<std::vector<double>> cleaned_data = processor.clean_data(data_points);
            std::vector<double> middle_point = analyzer.find_middle_point(data_points);

            std::vector<size_t> best_dimensions = analyzer.select_best_dimensions(data_points);

            std::vector<std::vector<double>> projected_data = analyzer.project_to_dimensions(data_points, best_dimensions);

            std::cout << drone_position.size() << std::endl;
            
            std::vector<double> avg_distances = analyzer.calculate_average_distances(projected_data, drone_position);
            std::vector<size_t> nan_indices; 

            auto result = analyzer.find_exit(projected_data, drone_position, avg_distances);
            // std::cout << "Exit Points: ";
            int i = 0;
            for (const auto& point : result.first) {
                for (const double& coord : point) {
                    ret_point.push_back(coord);
                }
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        
        return ret_point;
    }
};
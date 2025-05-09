/**
 * @file io.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 05-07-2025
 * @copyright Copyright (c) 2025
 */

#ifndef GFGG_UTILS_IO_H_
#define GFGG_UTILS_IO_H_

#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace ORB_SLAM2 {

class FeatureIO {
public:
    static bool loadFeaturePoints(double                     timestamp,
                                  std::vector<cv::KeyPoint>& left_kpts,
                                  cv::Mat&                   left_desc,
                                  std::vector<cv::KeyPoint>& right_kpts,
                                  cv::Mat&                   right_desc) {
        const std::string filename = timestampToFilename(timestamp);
        std::ifstream     in(filename, std::ios::binary);
        if (!in.is_open()) {
            return false;
        }

        // Load left keypoints
        {
            size_t num_kpts;
            in.read((char*)&num_kpts, sizeof(num_kpts));
            left_kpts.resize(num_kpts);
            for (auto& kp : left_kpts) {
                in.read((char*)&kp.pt.x, sizeof(float));
                in.read((char*)&kp.pt.y, sizeof(float));
                in.read((char*)&kp.size, sizeof(float));
                in.read((char*)&kp.angle, sizeof(float));
                in.read((char*)&kp.response, sizeof(float));
                in.read((char*)&kp.octave, sizeof(int));
            }
        }

        // Load left descriptors
        {
            int rows, cols, type;
            in.read((char*)&rows, sizeof(rows));
            in.read((char*)&cols, sizeof(cols));
            in.read((char*)&type, sizeof(type));
            left_desc.create(rows, cols, type);
            in.read((char*)left_desc.data,
                    left_desc.total() * left_desc.elemSize());
        }

        {
            size_t num_kpts;
            in.read((char*)&num_kpts, sizeof(num_kpts));
            right_kpts.resize(num_kpts);
            for (auto& kp : right_kpts) {
                in.read((char*)&kp.pt.x, sizeof(float));
                in.read((char*)&kp.pt.y, sizeof(float));
                in.read((char*)&kp.size, sizeof(float));
                in.read((char*)&kp.angle, sizeof(float));
                in.read((char*)&kp.response, sizeof(float));
                in.read((char*)&kp.octave, sizeof(int));
            }
        }

        // Load descriptors
        {
            int rows, cols, type;
            in.read((char*)&rows, sizeof(rows));
            in.read((char*)&cols, sizeof(cols));
            in.read((char*)&type, sizeof(type));
            right_desc.create(rows, cols, type);
            in.read((char*)right_desc.data,
                    right_desc.total() * right_desc.elemSize());
        }

        in.close();
        return true;
    }

    static bool saveFeaturePoints(double                           timestamp,
                                  const std::vector<cv::KeyPoint>& left_kpts,
                                  const cv::Mat&                   left_desc,
                                  const std::vector<cv::KeyPoint>& right_kpts,
                                  const cv::Mat&                   right_desc) {
        const std::string filename = timestampToFilename(timestamp);
        std::ofstream     out(filename, std::ios::binary);
        if (!out.is_open()) {
            return false;
        }

        // Save left keypoints
        {
            size_t num_kpts = left_kpts.size();
            out.write((char*)&num_kpts, sizeof(num_kpts));
            for (const auto& kp : left_kpts) {
                out.write((char*)&kp.pt.x, sizeof(float));
                out.write((char*)&kp.pt.y, sizeof(float));
                out.write((char*)&kp.size, sizeof(float));
                out.write((char*)&kp.angle, sizeof(float));
                out.write((char*)&kp.response, sizeof(float));
                out.write((char*)&kp.octave, sizeof(int));
            }
        }

        // Save left descriptors
        {
            int rows = left_desc.rows;
            int cols = left_desc.cols;
            int type = left_desc.type();
            out.write((char*)&rows, sizeof(rows));
            out.write((char*)&cols, sizeof(cols));
            out.write((char*)&type, sizeof(type));
            out.write((char*)left_desc.data,
                      left_desc.total() * left_desc.elemSize());
        }

        // save right kpts.
        {
            size_t num_kpts = right_kpts.size();
            out.write((char*)&num_kpts, sizeof(num_kpts));
            for (const auto& kp : right_kpts) {
                out.write((char*)&kp.pt.x, sizeof(float));
                out.write((char*)&kp.pt.y, sizeof(float));
                out.write((char*)&kp.size, sizeof(float));
                out.write((char*)&kp.angle, sizeof(float));
                out.write((char*)&kp.response, sizeof(float));
                out.write((char*)&kp.octave, sizeof(int));
            }
        }

        // Save right descriptors
        {
            int rows = right_desc.rows;
            int cols = right_desc.cols;
            int type = right_desc.type();
            out.write((char*)&rows, sizeof(rows));
            out.write((char*)&cols, sizeof(cols));
            out.write((char*)&type, sizeof(type));
            out.write((char*)right_desc.data,
                      right_desc.total() * right_desc.elemSize());
        }

        out.close();
        return true;
    }

    static std::string timestampToFilename(double value, int precision = 6) {
        std::string prefix =
            "/mnt/DATA/experiments/good_graph/euroc_feature_db/gfgg/";
        std::ostringstream oss;
        oss << prefix << std::fixed << std::setprecision(precision) << value;
        std::string str = oss.str();

        // Replace '.' with '_' (filesystem-safe)
        std::replace(str.begin(), str.end(), '.', '_');
        return str + ".bin";
    }
};

}  // namespace ORB_SLAM2

#endif  // GFGG_UTILS_IO_H_
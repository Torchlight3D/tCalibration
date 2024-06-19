#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include <gtest/gtest.h>

#include <tCalibration/InertialBasedScaleEstimation>

using namespace tl;

constexpr double kTimeScale = 1e-9;

// Reading data from text files
std::vector<std::string> split(const std::string& subject)
{
    std::istringstream ss{subject};
    using StrIt = std::istream_iterator<std::string>;
    std::vector<std::string> container{StrIt{ss}, StrIt{}};
    return container;
}

void readVisualData(const std::string& path, std::vector<double>& time,
                    std::vector<Eigen::Vector3d>& pos,
                    std::vector<Eigen::Quaterniond>& quat)
{
    std::string fn = path + "poses.csv";
    std::string line;
    std::vector<Eigen::Quaterniond> q_vec;

    std::cout << fn << std::endl;
    {
        std::ifstream data_file(fn);
        if (data_file.is_open()) {
            while (std::getline(data_file, line)) {
                const auto strings = split(line);

                time.push_back(std::stod(strings[0]));
                pos.push_back(Eigen::Vector3d(std::stod(strings[1]),
                                              std::stod(strings[2]),
                                              std::stod(strings[3])));

                Eigen::Quaterniond qq(
                    std::stod(strings[4]), std::stod(strings[5]),
                    std::stod(strings[6]), std::stod(strings[7]));
                qq.normalize();
                quat.push_back(qq);
            }
        }
    }

    // Normalize time data
    double t0 = time[0];
    for (auto& tt : time) {
        tt = kTimeScale * (tt - t0);
    }
}

void readImuData(const std::string& fn, std::vector<double>& time,
                 std::vector<Eigen::Vector3d>& vec)
{
    std::string line;
    {
        std::ifstream data_file(fn);
        if (data_file.is_open()) {
            while (std::getline(data_file, line)) {
                const std::vector<std::string> strings = split(line);
                time.push_back(kTimeScale * std::stod(strings[0]));
                vec.push_back(Eigen::Vector3d(std::stod(strings[1]),
                                              std::stod(strings[2]),
                                              std::stod(strings[3])));
            }
        }
    }
}

double readGroundTruth(const std::string& fn)
{
    double scale{0.};
    std::string line;
    {
        std::ifstream data_file(fn);
        if (data_file.is_open()) {
            if (std::getline(data_file, line)) {
                std::vector<std::string> strings = split(line);
                scale = std::stod(strings[0].c_str());
            }
        }
    }

    return scale;
}

double testFullSystem(const std::string& dataset)
{
    InertialBasedScaleEstimation ibse;

    std::cout << "Running full system on '" << dataset << "' dataset"
              << std::endl;
    // TODO: Use std::filesystem
    const std::string data_path =
        "/home/bobblelaw/Playground/ibse/data/" + dataset + "/";

    // Read visual data
    std::vector<double> tVis;
    std::vector<Eigen::Vector3d> posVis;
    std::vector<Eigen::Quaterniond> qtVis;
    readVisualData(data_path, tVis, posVis, qtVis);
    ibse.setVisualData(tVis, posVis, qtVis);

    // Read IMU data
    std::vector<double> tAcc;
    std::vector<Eigen::Vector3d> linAcc;
    std::string fn = data_path + "accelerometer.csv";
    readImuData(fn, tAcc, linAcc);

    std::vector<double> tGyr;
    std::vector<Eigen::Vector3d> gyrImu;
    fn = data_path + "gyroscope.csv";
    readImuData(fn, tGyr, gyrImu);

    ibse.setInertialData(tAcc, linAcc, tGyr, gyrImu);

    // Read ground truth
    bool has_ground_truth = false;
    double true_scale = 1.0;
    std::ifstream data_file(data_path + "groundtruth.txt");
    if (data_file.good()) {
        true_scale = readGroundTruth(data_path + "groundtruth.txt");
        has_ground_truth = true;
    }

    double dtAcc = 0;
    for (size_t ii = 1; ii < tAcc.size(); ++ii) {
        dtAcc += tAcc[ii] - tAcc[ii - 1];
    }
    dtAcc /= (tAcc.size() - 1);

    double dtGyr = 0;
    for (size_t ii = 1; ii < tGyr.size(); ++ii) {
        dtGyr += tGyr[ii] - tGyr[ii - 1];
    }
    dtGyr /= (tGyr.size() - 1);

    std::cout << "Sampling rate of the accelerometer = " << 1.0 / dtAcc << " Hz"
              << std::endl;
    std::cout << "Sampling rate of the gyroscope = " << 1.0 / dtGyr << " Hz"
              << std::endl;

    // Run the full system
    //
    auto full_begin = std::chrono::high_resolution_clock::now();
    std::cout << "Estimate visual and IMU alignment.." << std::flush;
    ibse.initialAlignmentEstimation();
    std::cout << " done" << std::endl << std::endl;

    std::cout << "Final estimation in the frequency domain.." << std::endl;
    const double fMax = 1.2; // Emperically determined (in Hz)
    // const double fMax = 2.5; // Emperically determined (in
    // Hz)/home/bobblelaw/Playground/ibse/data

    double scale;
    Eigen::Vector3d g;
    Eigen::Vector3d bias;
    auto estimate_scale_begin = std::chrono::high_resolution_clock::now();

    ibse.estimateScale(scale, g, bias, fMax);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        end - estimate_scale_begin)
                        .count();
    std::cout << "..done in " << 1e-9 * duration << " sec" << std::endl
              << std::endl;

    std::cout << "Final values:" << std::endl;
    std::cout << "  scale = " << scale << std::endl;
    std::cout << "  g     = [" << g.x() << " " << g.y() << " " << g.z() << "]"
              << std::endl;
    std::cout << "  bias  = [" << bias.x() << " " << bias.y() << " " << bias.z()
              << "]" << std::endl;

    double err = 0.0;
    if (has_ground_truth) {
        err = 100.0f * fabs(fabs(scale) - fabs(true_scale)) / true_scale;
        std::cout << "Ground truth scale: " << true_scale << "   error=" << err
                  << "%" << std::endl;
    }

    std::cout << std::endl << std::endl;

    duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - full_begin)
            .count();
    std::cout << "Full system took " << 1e-9 * duration << " sec" << std::endl
              << std::endl;

    return err;
}

TEST(IBSE, Overall)
{
    GTEST_SKIP() << "Dataset is not ready";

    testFullSystem("D1");
    EXPECT_TRUE(true);
}

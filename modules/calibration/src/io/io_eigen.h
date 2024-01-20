#pragma once

#include <string>

#include <Eigen/Core>
#include <json/json.hpp>
#include <yaml-cpp/yaml.h>

namespace tl::io {

namespace key {
inline constexpr char kMatRows[]{"rows"};
inline constexpr char kMatCols[]{"cols"};
inline constexpr char kMatDataType[]{"dt"};
inline constexpr char kMatDataTypeId[]{"typeid"};
inline constexpr char kMatData[]{"data"};
} // namespace key

namespace {
inline constexpr char kMatTag[]{"opencv-matrix"}; // not used
inline constexpr char kDoubleType[]{"d"};
} // namespace

template <typename Scalar, int Rows, int Cols>
YAML::Node toCVYamlNode(const Eigen::Matrix<Scalar, Rows, Cols>& matrix)
{
    YAML::Node node;
    node[key::kMatRows] = Rows;
    node[key::kMatCols] = Cols;
    node[key::kMatDataType] = kDoubleType;

    node[key::kMatData].SetStyle(YAML::EmitterStyle::Flow);
    if constexpr (Cols == 1 || Rows == 1) { // Vector
        const auto* data = matrix.data();
        for (auto it = data; it < data + matrix.size(); ++it) {
            node[key::kMatData].push_back(*it);
        }
    }
    else { // Matrix
        Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor> mat{matrix};
        const auto* data = mat.data();
        for (auto it = data; it < data + mat.size(); ++it) {
            node[key::kMatData].push_back(*it);
        }
    }

    return node;
}

template <typename Scalar, int Rows, int Cols>
bool fromCVYamlNode(const YAML::Node& node,
                    Eigen::Matrix<Scalar, Rows, Cols>& matrix)
{
    // Assume all the keys exist.
    const int rows = node[key::kMatRows].as<int>();
    const int cols = node[key::kMatCols].as<int>();
    if (rows != Rows || cols != Cols) {
        return false;
    }

    // TODO: Check type

    matrix = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor>>(
        node[key::kMatData].as<std::vector<Scalar>>().data());

    return false;
}

} // namespace tl::io

// For nlohmann::json
namespace Eigen {

template <typename Scalar, int Rows, int Cols>
void to_json(nlohmann::json& json, const Matrix<Scalar, Rows, Cols>& camera)
{
    // TODO
}

template <typename Scalar, int Rows, int Cols>
void from_json(const nlohmann::json& json, Matrix<Scalar, Rows, Cols>& camera)
{
    // TODO
}

} // namespace Eigen

// For yaml-cpp
namespace YAML {

template <typename Scalar, int Rows, int Cols>
struct convert<Eigen::Matrix<Scalar, Rows, Cols>>
{
    using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;
    using RowMatrix = Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor>;

    static Node encode(const Matrix& matrix)
    {
        namespace key = tl::io::key;

        Node node;
        node[key::kMatRows] = Rows;
        node[key::kMatCols] = Cols;
        node[key::kMatData].SetStyle(YAML::EmitterStyle::Flow);

        // TODO: How to save recoverable scalar type

        if constexpr (Cols == 1 || Rows == 1) { // Vector
            const auto* data = matrix.data();
            for (auto it = data; it < data + matrix.size(); ++it) {
                node[key::kMatData].push_back(*it);
            }
        }
        else { // Matrix
            RowMatrix mat{matrix};
            const auto* data = mat.data();
            for (auto it = data; it < data + mat.size(); ++it) {
                node[key::kMatData].push_back(*it);
            }
        }

        return node;
    }

    static bool decode(const Node& node, Matrix& matrix)
    {
        namespace key = tl::io::key;

        // Assume all the keys exist.
        const int rows = node[key::kMatRows].as<int>();
        const int cols = node[key::kMatCols].as<int>();
        if (rows != Rows || cols != Cols) {
            return false;
        }

        // TODO: Check std::is_arithmetic_v<T>()

        matrix = Eigen::Map<RowMatrix>(
            node[key::kMatData].as<std::vector<Scalar>>().data());

        return false;
    }
};

} // namespace YAML

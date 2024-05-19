#pragma once

#include "calibboardbase.h"

#include <json/json.hpp>

namespace tl {

class Chessboard : public CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<Chessboard>;
    using ConstPtr = std::shared_ptr<const Chessboard>;

    struct Options
    {
        int rows = 11;
        int cols = 8;
        double rowSpacing = 0.045;
        double colSpacing = 0.045;

        Options() {}
    };

    struct DetectOptions
    {
        // Subpixel post-process, not neccessary after OpenCV 4
        int subPixWinSize{11};
        bool doSubPix{true};

        // OpenCV parameters
        bool useAdaptiveThreshold{true};
        bool useFastCheck{true};
        bool filterQuads{false};
        bool normalizeImage{true};

        DetectOptions() {}
    };

    explicit Chessboard(const Options& opts = {},
                        const DetectOptions& detectOpts = {});
    virtual ~Chessboard() = default;

    inline static constexpr char kType[11]{"Chessboard"};

    double tagDistance() const override;
    double tagSize() const override;

    TargetDetection computeObservation(
        cv::InputArray image,
        cv::OutputArray viz = cv::noArray()) const override;

protected:
    void createBoardPoints() override;

private:
    Chessboard::DetectOptions m_detectOpts;
    double m_rowSpacing; // meter
    double m_colSpacing; // meter
};

// nlohmann::json interface
void to_json(nlohmann::json& json, const Chessboard::Options& opts);
void from_json(const nlohmann::json& json, Chessboard::Options& opts);

void to_json(nlohmann::json& json, const Chessboard::DetectOptions& opts);
void from_json(const nlohmann::json& json, Chessboard::DetectOptions& opts);

} // namespace tl

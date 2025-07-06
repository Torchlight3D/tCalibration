#pragma once

#include "calibboardbase.h"

#include <nlohmann/json.hpp>

namespace tl {

class CircleGridBoard : public CalibBoardBase
{
public:
    using Ptr = std::shared_ptr<CircleGridBoard>;
    using ConstPtr = std::shared_ptr<const CircleGridBoard>;

    struct Options
    {
        int rows = 11;
        int cols = 8;
        double spacing = 0.045;
        bool asymmetricGrid = false;

        Options() {}
    };

    // FIXME: A circular board should be defined by
    // 1. Symetric, center spacing and circle diameter
    // 2. Asymetric, diagonal center spacing and circle diameter
    explicit CircleGridBoard(const Options& opts = {});
    virtual ~CircleGridBoard() = default;

    inline static constexpr char kType[16]{"CircleGridBoard"};

    double tagDistance() const override;
    double tagSize() const override;

    TargetDetection computeObservation(
        cv::InputArray image,
        cv::OutputArray viz = cv::noArray()) const override;

protected:
    void createBoardPoints() override;

private:
    CircleGridBoard::Options m_options;
    double m_spacing; // meter
};

void to_json(nlohmann::json& json, const CircleGridBoard::Options& opts);
void from_json(const nlohmann::json& json, CircleGridBoard::Options& opts);

} // namespace tl

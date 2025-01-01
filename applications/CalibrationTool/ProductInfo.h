#pragma once

#include <json/json.hpp>

namespace tl {

// TODO:
// 1. Consider more types of devices. Now assume product is stereo camera with
// motion sensor.
// 2.
struct ProductInfo
{
    std::string name;

    double focalLength = 300.;

    std::array<double, 2> principalPoint = {320., 219.};

    std::array<double, 3> t_ic0{0.};

    double baseline{0.};

    static bool loadProductsFromJson(const std::string& json);
    static std::optional<ProductInfo> of(const std::string& name);

private:
    static std::unordered_map<std::string, ProductInfo>& rProducts();
    inline static const std::unordered_map<std::string, ProductInfo>& products()
    {
        return rProducts();
    }
};

} // namespace tl

namespace nlohmann {

void to_json(json& j, const tl::ProductInfo& info);
void from_json(const json& j, tl::ProductInfo& info);

} // namespace nlohmann

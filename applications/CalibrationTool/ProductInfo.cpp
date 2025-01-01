#include "ProductInfo.h"

namespace tl {

namespace key {
constexpr char kName[]{"name"};
constexpr char kFocalLength[]{"focal_length"};
constexpr char kPrincipalPoint[]{"principal_point"};
constexpr char kTic0[]{"tic0"};
constexpr char kBaseline[]{"baseline"};

constexpr char kType[]{"type"};
constexpr char kInfo[]{"info"};
} // namespace key

bool ProductInfo::loadProductsFromJson(const std::string& json)
{
    const auto j = nlohmann::json::parse(json);
    if (!j.is_array()) {
        return false;
    }

    auto& products = rProducts();
    products.clear();
    for (const auto& e : j) {
        // TODO: Use "type" to create ProductInfo
        ProductInfo info;
        e.at(key::kInfo).get_to(info);
        products.insert({info.name, info});
    }

    return true;
}

std::optional<ProductInfo> ProductInfo::of(const std::string& name)
{
    if (!products().contains(name)) {
        return {};
    }

    return products().at(name);
}

std::unordered_map<std::string, ProductInfo>& ProductInfo::rProducts()
{
    static std::unordered_map<std::string, ProductInfo> _products;
    return _products;
}

} // namespace tl

namespace nlohmann {

using namespace tl;

void to_json(json& j, const tl::ProductInfo& info)
{
    j[key::kName] = info.name;
    j[key::kFocalLength] = info.focalLength;
    j[key::kPrincipalPoint] = info.principalPoint;
    j[key::kTic0] = info.t_ic0;
    j[key::kBaseline] = info.baseline;
}

void from_json(const json& j, tl::ProductInfo& info)
{
    j.at(key::kName).get_to(info.name);
    j.at(key::kFocalLength).get_to(info.focalLength);
    j.at(key::kPrincipalPoint).get_to(info.principalPoint);
    j.at(key::kTic0).get_to(info.t_ic0);
    j.at(key::kBaseline).get_to(info.baseline);
}

} // namespace nlohmann

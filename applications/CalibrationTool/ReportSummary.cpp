#include "ReportSummary.h"

#include <QJsonArray>

namespace tl {

namespace key {
constexpr char kName[]{"name"};
constexpr char kValue[]{"value"};
constexpr char kExpectation[]{"expectation"};
constexpr char kTooltip[]{"tooltip"};
constexpr char kPass[]{"pass"};
constexpr char kUsed[]{"used"};

constexpr char kItems[]{"items"};
} // namespace key

QJsonObject ReportItem::toJson() const
{
    QJsonObject jo;
    jo[key::kName] = name;
    jo[key::kValue] = value;
    jo[key::kExpectation] = expectation;
    jo[key::kTooltip] = toolTip;
    jo[key::kPass] = pass;
    jo[key::kUsed] = used;
    return jo;
}

ReportItem ReportItem::fromJson(const QJsonObject &jo)
{
    ReportItem item;

    if (const auto jv = jo.value(key::kName); jv.isString()) {
        item.name = jv.toString();
    }
    if (const auto jv = jo.value(key::kValue); jv.isString()) {
        item.value = jv.toString();
    }
    if (const auto jv = jo.value(key::kExpectation); jv.isString()) {
        item.expectation = jv.toString();
    }
    if (const auto jv = jo.value(key::kTooltip); jv.isString()) {
        item.toolTip = jv.toString();
    }
    if (const auto jv = jo.value(key::kPass); jv.isBool()) {
        item.pass = jv.toBool();
    }
    if (const auto jv = jo.value(key::kUsed); jv.isBool()) {
        item.used = jv.toBool();
    }

    return item;
}

///------- ReportSummary starts from here
ReportSummary::ReportSummary() {}

ReportSummary::ReportSummary(Items items) { setItem(items); }

const ReportSummary::Items &ReportSummary::items() const { return m_items; }

void ReportSummary::setItem(Items items) { m_items = std::move(items); }

int ReportSummary::itemCount() const
{
    return static_cast<int>(items().size());
}

bool ReportSummary::pass() const
{
    return !m_items.empty() &&
           std::ranges::all_of(m_items, [](const auto &item) {
               return !item.used || item.pass;
           });
}

QJsonObject ReportSummary::toJson() const
{
    QJsonObject jo;
    jo[key::kItems] = [this]() {
        QJsonArray ja;
        for (const auto &item : m_items) {
            ja << item.toJson();
        }
        return ja;
    }();

    return jo;
}

ReportSummary ReportSummary::fromJson(const QJsonObject &jo)
{
    Items items;
    if (const auto jv = jo.value(key::kItems); jv.isArray()) {
        const auto ja = jv.toArray();
        items.reserve(ja.size());
        for (const auto &v : ja) {
            items.push_back(ReportItem::fromJson(v.toObject()));
        }
    }

    return ReportSummary{items};
}

} // namespace tl

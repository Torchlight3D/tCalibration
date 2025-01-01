#pragma once

#include <QJsonObject>

namespace tl {

struct ReportItem
{
    QString name;
    QString value;
    QString expectation;
    QString toolTip;
    bool pass{false};
    bool used{true};

    QJsonObject toJson() const;
    static ReportItem fromJson(const QJsonObject& jo);
};

class ReportSummary
{
public:
    using Items = std::vector<ReportItem>;

    // Summary takes ownership of Items
    ReportSummary();
    explicit ReportSummary(Items items);

    const Items& items() const;
    void setItem(Items items);
    int itemCount() const;

    bool pass() const;

    QJsonObject toJson() const;
    static ReportSummary fromJson(const QJsonObject& jo);

private:
    Items m_items; // Owned
};

} // namespace tl

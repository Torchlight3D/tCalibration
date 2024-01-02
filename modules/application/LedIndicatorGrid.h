#pragma once

#include <QWidget>
#include <QColormap>

#include "gui/guiutils.h"
#include "gui/LedIndicator.h"

namespace thoht {

class LedIndicatorGridPrivate;
class LedIndicatorGrid : public QWidget
{
    Q_OBJECT

public:
    explicit LedIndicatorGrid(QWidget* parent = nullptr);
    ~LedIndicatorGrid();

    int ledCount() const;
    void setLedCount(int count);

    void setLedStatus(int index, LedIndicator::Status status);
    void setLedColor(int index, const QColor& color);
    void setLed(int index, LedIndicator::Status status, const QColor& color);

    void setLedStatuses(const std::vector<LedIndicator::Status>& statuses);
    void setLedColors(const std::vector<QColor>& colors);

    QColor ledOnColor() const;

private:
    Q_DECLARE_PIMPL(LedIndicatorGrid)
};

} // namespace thoht

#pragma once

#include "NumericalDisplay.h"

namespace thoht {

class StateDisplay : public NumericalDisplay
{
    Q_OBJECT

public:
    explicit StateDisplay(Board *dashboard = nullptr);

    void edit() override;

    void initialize(Board *dashboard) override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;

    void displayData() override;

private:
    QPen mTextBorderPen;
};

} // namespace thoht

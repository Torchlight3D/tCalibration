#pragma once

#include "SingleDisplay.h"
#include "../BoardParameter.h"

namespace thoht {

class NumericalDisplay : public SingleDisplay
{
    Q_OBJECT

public:
    explicit NumericalDisplay(Board *dashboard = nullptr);

    // reimplemented virtual methods:
    void initialize(Board *dashboard) override;
    void resetElement() override;
    void buildElement() override;
    void edit() override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;

    void loadData() override;
    void displayData() override;
    void applySizeConstraints() override;

    void setValueFormat(ValueStringFormat valueFormat);
    ValueStringFormat valueFormat() const;

    enum DisplayOrientation
    {
        doVerticalAlignCenter,
        doVerticalAlignLeftRight
    };
    void setOrientation(DisplayOrientation orientation);
    DisplayOrientation orientation() const;

    enum StatMode
    {
        smNone,
        smMin,
        smMax
    };
    void setStatMode(StatMode statMode);
    StatMode statMode() const;

protected:
    DisplayOrientation mOrientation;
    ValueStringFormat mValueFormat;
    StatMode mStatMode;
    LayoutGrid *mUnitLayout{};
    AdaptiveTextElement *mTextMode;

    double mStatMinValue;
    double mStatMaxValue;
};

} // namespace thoht

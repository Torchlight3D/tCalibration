#pragma once

#include "../BoardElement.h"

#define LABEL_HEIGHT_RATIO 0.3
#define UNIT_HEIGHT_RATIO 0.25
#define VALUE_HEIGHT_RATIO 0.45

namespace thoht {

class AdaptiveTextElement;
class LayoutGrid;

class SingleDisplay : public BoardElement
{
    Q_OBJECT

public:
    explicit SingleDisplay(Board *dashboard = nullptr);

    // reimplemented virtual methods
    int defaultWidth() override;
    int defaultHeight() override;

    void initialize(Board *dashboard) override;
    void resetElement() override;
    void buildElement() override;
    void clearElement() override;

    void edit() override;
    void checkParameters() override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;

    void loadData() override;
    void displayData() override;
    void update(UpdatePhase phase) override;

    bool headerVisible() const;
    bool valueVisible() const;

    virtual void applySizeConstraints();

    void setHeaderVisible(bool visible);
    void setValueVisible(bool visible);

protected:
    AdaptiveTextElement *mTextLabel;
    AdaptiveTextElement *mTextUnit;
    AdaptiveTextElement *mTextValue;
    QCPLayoutElement *mEmptyElementBottom{};
    QCPLayoutElement *mEmptyElementTop{};

    LayoutGrid *mMainLayout{};
    LayoutGrid *mSubLayout{};

    int mSpacing;

    bool mHeaderVisible;
    bool mValueVisible;
};

} // namespace thoht

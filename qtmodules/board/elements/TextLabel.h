#pragma once

#include "../BoardElement.h"

namespace tl {

class AdaptiveTextElement;
class LayoutGrid;

class TextLabel : public BoardElement
{
    Q_OBJECT

public:
    explicit TextLabel(Board *dashboard = nullptr);

    // reimplemented virtual methods:
    void update(UpdatePhase phase) override;
    void initialize(Board *dashboard) override;
    void resetElement() override;
    void buildElement() override;

    void clearElement() override;
    int defaultWidth() override;
    int defaultHeight() override;
    void edit() override;

    void saveSettings(QSettings *settings) override;
    void loadSettings(QSettings *settings) override;

    void setText(const QString &text);
    QString text() const;

    ColorSettings &colorSettingsRef();

protected:
    QString mText;
    AdaptiveTextElement *mTextElement;
    ColorSettings mColorSettings;
    LayoutGrid *mMainLayout{};
};

} // namespace tl

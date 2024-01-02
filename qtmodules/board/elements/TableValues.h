#pragma once

#include "../BoardElement.h"
#include "../BoardParameter.h"

namespace thoht {

class AdaptiveTextElement;
class LayoutGrid;

class TableValues : public BoardElement
{
    Q_OBJECT

public:
    explicit TableValues(Board *dashboard = nullptr);
    ~TableValues() override;

    // reimplemented virtual methods:
    void update(UpdatePhase phase) override;
    void initialize(Board *dashboard) override;
    int defaultWidth() override;
    int defaultHeight() override;
    void resetElement() override;
    void clearElement() override;
    void buildElement() override;
    void edit() override;

    void loadSettings(QSettings *settings) override;
    void saveSettings(QSettings *settings) override;

    void loadConfigurations(QSettings *settings) override;
    void checkParameters() override;

    QSharedPointer<BoardParameter> addParameter(
        ParameterConfiguration::Ptr parameterSettings)
        override;
    QSharedPointer<BoardParameter> addParameter(QString paramLabel) override;
    void addParameter(QSharedPointer<BoardParameter> boardParameter) override;
    void removeBoardParameter(int index, bool update = true) override;

    void displayData() override;

    virtual void applySizeConstraints();

    void setAlternatingRowColors(bool on);
    bool alternatingRowColors() const;

    void setUnitVisible(bool visible);
    bool unitVisible() const;

    void setValueFormat(ValueStringFormat format);
    ValueStringFormat valueFormat() const;

    void setPrecision(int precision);
    int precision() const;

    void setTitle(const QString &title);
    QString title() const;

    void setTitleVisible(bool visible);
    bool titleVisible() const;

protected:
    QTableWidget *mTableWidget;
    QMargins mTableMargins;

    bool mAlternatingRowColors;
    bool mUnitVisible;
    bool mTitleVisible;
    QString mTitle;
    ValueStringFormat mValueFormat;
    int mPrecision;

    LayoutGrid *mMainLayout{};
    AdaptiveTextElement *mTextLabel;
    QCPLayoutElement *mEmptyElementBottom{};
};

} // namespace thoht

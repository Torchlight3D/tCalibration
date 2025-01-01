#pragma once

#include <QWidget>

class QSettings;

namespace tl {

class ToolView : public QWidget
{
    Q_OBJECT

public:
    using QWidget::QWidget;
    virtual ~ToolView() = default;

    virtual void setDevMode(bool enable)
    {
        // ...
    }

    virtual bool setFromJson(const std::string& json) { return true; }

    virtual void restoreSettings(QSettings& settings)
    {
        // ...
    }

    virtual void saveSettings(QSettings& settings) const
    {
        // ...
    }
};

inline auto viewTitle(const QString& windowTitle,
                      const QString& productName = {})
{
    using namespace Qt::Literals::StringLiterals;

    return u"%1 - %2"_s.arg(windowTitle, productName.isEmpty()
                                             ? ToolView::tr("Unknown Product")
                                             : productName);
}

} // namespace tl

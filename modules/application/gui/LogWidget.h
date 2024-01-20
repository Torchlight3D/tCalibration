#pragma once

#include <QWidget>
#include "guiutils.h"

namespace tl {

// For Standard output stream display.
// TODO: Not very good performance, freeze if log keeps coming in.
class LogWidgetPrivate;
class LogWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LogWidget(QWidget* parent = nullptr);
    ~LogWidget();

    void setMaxBlockCount(int count);
    void setToolBarVisible(bool visible);

public slots:
    void append(const QByteArray& message);
    void flush();
    void clear();

private:
    Q_DECLARE_PIMPL(LogWidget)
};

} // namespace tl

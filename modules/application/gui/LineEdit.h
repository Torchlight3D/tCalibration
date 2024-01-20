#pragma once

#include <QIcon>
#include <QLineEdit>

#include "TheStyleTypes.h"

namespace tl {

/// A QLineEdit that draws a search icon
class LineEdit : public QLineEdit
{
    Q_OBJECT
    Q_PROPERTY(QIcon icon READ icon WRITE setIcon NOTIFY iconChanged)
    Q_PROPERTY(Status status READ status WRITE setStatus NOTIFY statusChanged)

public:
    using QLineEdit::QLineEdit;

    const QIcon& icon() const;
    Q_SLOT void setIcon(const QIcon& icon);
    Q_SIGNAL void iconChanged();

    void setUseMonoSpaceFont(bool use);
    bool useMonoSpaceFont() const;

    Status status() const;
    Q_SLOT void setStatus(Status status);
    Q_SIGNAL void statusChanged();

protected:
    void paintEvent(QPaintEvent* event) override;
    bool event(QEvent* event) override;

private:
    QPixmap getPixmap() const;
    void updateFont();

private:
    QIcon _icon;
    bool _useMonospaceFont{false};
    Status _status{Status::Default};
};

} // namespace tl

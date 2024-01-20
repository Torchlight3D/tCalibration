#pragma once

#include <QLineEdit>

#include "../settings/ColorSettings.h"

namespace tl {

class ColorLineEdit : public QLineEdit
{
    Q_OBJECT

public:
    explicit ColorLineEdit(QWidget *parent = nullptr);
    ~ColorLineEdit();

    void setMode(const ColorSettings::ColorMode &mode);
    ColorSettings::ColorMode mode() const;

    void setColor(const QColor &color);
    QColor color() const;

    void setColorText(bool colorText);
    bool colorText() const;

signals:
    void clicked();

protected:
    QColor colorFromString(const QString &string);
    bool eventFilter(QObject *obj, QEvent *ev) override;

private:
    bool mColorText;
    ColorSettings *mColorSettings;
};

} // namespace tl

#pragma once

#include <QAbstractButton>

#include "qtcoreutils.h"

namespace tl {

class QtSwitchButtonPrivate;
class QtSwitchButton final : public QAbstractButton
{
    Q_OBJECT
    Q_PROPERTY(int handlePosition READ handlePosition WRITE setHandlePosition
                   NOTIFY handlePositionChanged FINAL)

public:
    explicit QtSwitchButton(QWidget *parent = nullptr);
    explicit QtSwitchButton(const QString &text, QWidget *parent = nullptr);
    ~QtSwitchButton();

    QSize sizeHint() const override;

    /// Properties
    inline static constexpr QColor kDefaultTrackOnColor{26, 115, 232, 128};
    inline static constexpr QColor kDefaultTrackOffColor{189, 193, 198};
    inline static constexpr QColor kDefaultHandleOnColor{26, 115, 232, 255};
    inline static constexpr QColor kDefaultHandleOffColor{255, 255, 255};

    void setTrackColor(const QColor &on, const QColor &off);
    void setHandleColor(const QColor &on, const QColor &off);
    inline void setDefaultColor()
    {
        setTrackColor(kDefaultTrackOnColor, kDefaultTrackOffColor);
        setHandleColor(kDefaultHandleOnColor, kDefaultHandleOffColor);
    }

signals:
    void handlePositionChanged();

protected:
    void paintEvent(QPaintEvent *event) override;
    void nextCheckState() override;
    void checkStateSet() override;

private:
    int handlePosition() const;
    void setHandlePosition(int pos);

private:
    Q_DECLARE_PIMPL(QtSwitchButton)
};

} // namespace tl

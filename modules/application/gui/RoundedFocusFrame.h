#pragma once

#include <QFocusFrame>

#include "CornerRadius.h"

namespace thoht {

class RoundedFocusFrame : public QFocusFrame
{
    Q_OBJECT
    Q_PROPERTY(CornerRadius radiuses READ radiuses WRITE setRadiuses NOTIFY
                   radiusesChanged)

public:
    using QFocusFrame::QFocusFrame;

    const CornerRadius& radiuses() const;
    Q_SLOT void setRadiuses(const CornerRadius& cornerRadius);
    Q_SIGNAL void radiusesChanged();

private:
    CornerRadius _radiuses;
};

} // namespace thoht

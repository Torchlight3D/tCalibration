#pragma once

#include <QWidget>

namespace thoht {

class StatusBadge : public QWidget
{
    Q_OBJECT

public:
    enum class Type
    {
        Success,
        Info,
        Warning,
        Error,
    };

    enum class Size
    {
        Small,
        Medium,
    };

    explicit StatusBadge(Type type = Type::Info, Size size = Size::Medium,
                         QWidget* parent = nullptr);

    Type badgeType() const;
    Q_SLOT void setBadgeType(Type type);
    Q_SIGNAL void badgeTypeChanged();

    Size badgeSize() const;
    Q_SLOT void setBadgeSize(Size size);
    Q_SIGNAL void badgeSizeChanged();

    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    Size m_size{Size::Medium};
    Type m_type{Type::Info};
};

} // namespace thoht

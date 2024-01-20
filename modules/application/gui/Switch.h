#pragma once

#include <QAbstractButton>
#include <QVariantAnimation>

namespace tl {

class QStyleOptionFocusRoundedRect;
class RoundedFocusFrame;

// TODO: Use pimpl

class Switch : public QAbstractButton
{
    Q_OBJECT

public:
    Switch(QWidget* parent = nullptr);

    void initStyleOptionFocus(QStyleOptionFocusRoundedRect& opt) const;

    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* e) override;
    void enterEvent(QEnterEvent* e) override;
    void leaveEvent(QEvent* e) override;
    void changeEvent(QEvent* e) override;
    void focusInEvent(QFocusEvent* e) override;
    void focusOutEvent(QFocusEvent* e) override;
    void checkStateSet() override;

private slots:
    void setupAnimation();
    void startAnimation();

private:
    QRect switchRect() const;
    const QColor& backgroundColor() const;
    const QColor& borderColor() const;
    const QColor& foregroundColor() const;
    const QColor& textColor() const;

private:
    QVariantAnimation m_animHandle;
    QVariantAnimation m_animBg;
    QVariantAnimation m_animFg;
    QVariantAnimation m_animBorder;
    RoundedFocusFrame* m_focusFrame{nullptr};
    int m_handlePadding{2};
    bool m_isMouseOver{false};
};

} // namespace tl

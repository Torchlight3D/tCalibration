#pragma once

#include <QObject>

class QAbstractItemView;
class QCommandLinkButton;
class QMenu;
class QTabBar;
class QToolButton;

namespace tl {

class QlementineStyle;
class WidgetAnimationManager;

class LineEditButtonEventFilter : public QObject
{
    Q_OBJECT
public:
    LineEditButtonEventFilter(QlementineStyle& style,
                              WidgetAnimationManager& animManager,
                              QToolButton* button);

    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    QlementineStyle& _style;
    WidgetAnimationManager& _animManager;
    QToolButton* _button{nullptr};
};

class CommandLinkButtonPaintEventFilter : public QObject
{
    Q_OBJECT
public:
    CommandLinkButtonPaintEventFilter(QlementineStyle& style,
                                      WidgetAnimationManager& animManager,
                                      QCommandLinkButton* button);

    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    QlementineStyle& _style;
    WidgetAnimationManager& _animManager;
    QCommandLinkButton* _button{nullptr};
};

class MouseWheelBlockerEventFilter : public QObject
{
    Q_OBJECT
public:
    explicit MouseWheelBlockerEventFilter(QWidget* widget);

    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    QWidget* m_widget{nullptr};
};

class TabBarEventFilter : public QObject
{
    Q_OBJECT
public:
    TabBarEventFilter(QlementineStyle& style, QTabBar* tabBar);

    bool eventFilter(QObject* watchedObject, QEvent* evt) override;

private:
    QTabBar* m_tabBar{nullptr};
    QToolButton* m_leftButton{nullptr};
    QToolButton* m_rightButton{nullptr};
};

class MenuEventFilter : public QObject
{
    Q_OBJECT
public:
    explicit MenuEventFilter(QMenu* menu);

    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    QMenu* m_menu{nullptr};
};

class ComboboxItemViewFilter : public QObject
{
    Q_OBJECT
public:
    explicit ComboboxItemViewFilter(QAbstractItemView* view);

    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    QAbstractItemView* m_view{nullptr};
};

} // namespace tl

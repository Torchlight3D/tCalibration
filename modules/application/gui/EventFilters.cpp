#include "EventFilters.h"

#include <QAbstractItemView>
#include <QCommandLinkButton>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMouseEvent>
#include <QPainter>
#include <QTimer>
#include <QToolButton>

#include "primitiveutils.h"
#include "stylestateutils.h"
#include "TheStyle.h"
#include "WidgetAnimationManager.h"

namespace tl {

///------- LineEditButtonEventFilter starts from here
LineEditButtonEventFilter::LineEditButtonEventFilter(
    QlementineStyle& style, WidgetAnimationManager& animManager,
    QToolButton* button)
    : QObject(button), _style(style), _animManager(animManager), _button(button)
{
    // Qt doesn't emit this signal so we emit it by ourselves.
    if (auto* parent = button->parentWidget()) {
        if (auto* lineEdit = qobject_cast<QLineEdit*>(parent)) {
            connect(_button, &QAbstractButton::clicked, lineEdit,
                    &QLineEdit::returnPressed);
        }
    }
}

bool LineEditButtonEventFilter::eventFilter(QObject* watched, QEvent* e)
{
    switch (e->type()) {
        case QEvent::Resize: {
            // Prevent resizing from qlineedit_p.cpp:540
            e->ignore();
            return true;
        } break;
        case QEvent::Move: { // Prevent moving from qlineedit_p.cpp:540
            e->ignore();
            // Instead, place the button by ourselves.
            const auto* parentLineEdit = _button->parentWidget();
            const auto parentRect = parentLineEdit->rect();
            const auto& theme = _style.theme();
            const auto buttonH = theme.controlHeightMedium;
            const auto buttonW = buttonH;
            const auto spacing = theme.spacing / 2;
            const auto buttonX =
                parentRect.x() + parentRect.width() - buttonW - spacing;
            const auto buttonY =
                parentRect.y() + (parentRect.height() - buttonH) / 2;
            _button->setGeometry(buttonX, buttonY, buttonW, buttonH);
            return true;
        }
        case QEvent::Paint: {
            // Draw the button by ourselves to bypass
            // QLineEditIconButton::paintEvent in qlineedit_p.cpp:353
            const auto enabled = _button->isEnabled();
            if (!enabled) {
                e->accept();
                return true;
            }

            const auto hovered = _button->underMouse();
            const auto pressed = _button->isDown();
            const auto mouse = getMouseState(pressed, hovered, enabled);
            const auto& theme = _style.theme();
            const auto rect = _button->rect();
            const auto& bgColor =
                _style.toolButtonBackgroundColor(mouse, ColorRole::Secondary);
            const auto circleH = theme.controlHeightMedium;
            const auto circleW = circleH;
            const auto circleX = rect.x() + (rect.width() - circleW) / 2;
            const auto circleY = rect.y() + (rect.height() - circleH) / 2;
            const auto circleRect =
                QRect(QPoint{circleX, circleY}, QSize{circleW, circleH});
            // Get opacity animated in qlinedit_p.cpp:436
            const auto opacity =
                _button->property(QByteArrayLiteral("opacity")).toDouble();
            const auto pixmap = getPixmap(_button->icon(), theme.iconSize,
                                          mouse, Qt::Unchecked);
            const auto pixmapX =
                circleRect.x() +
                (circleRect.width() - theme.iconSize.width()) / 2;
            const auto pixmapY =
                circleRect.y() +
                (circleRect.height() - theme.iconSize.height()) / 2;
            const auto pixmapRect = QRect{{pixmapX, pixmapY}, theme.iconSize};
            const auto& currentBgColor = _animManager.animateBackgroundColor(
                _button, bgColor, theme.animationDuration);

            QPainter p(_button);
            p.setOpacity(opacity);
            p.setPen(Qt::NoPen);
            p.setBrush(currentBgColor);
            p.setRenderHint(QPainter::Antialiasing, true);
            p.drawEllipse(circleRect);
            p.drawPixmap(pixmapRect, pixmap);

            e->accept();
            return true;
        }

        default:
            break;
    }

    return QObject::eventFilter(watched, e);
}

///------- CommandLinkButtonPaintEventFilter starts from here
CommandLinkButtonPaintEventFilter::CommandLinkButtonPaintEventFilter(
    QlementineStyle& style, WidgetAnimationManager& animManager,
    QCommandLinkButton* button)
    : QObject(button), _style(style), _animManager(animManager), _button(button)
{
}

bool CommandLinkButtonPaintEventFilter::eventFilter(QObject* watched, QEvent* e)
{
    const auto type = e->type();
    if (type == QEvent::Paint) {
        // Draw the button by ourselves to bypass
        // QLineEditIconButton::paintEvent in qlineedit_p.cpp:353
        const auto enabled = _button->isEnabled();
        const auto hovered = _button->underMouse();
        const auto pressed = _button->isDown();
        const auto mouse = getMouseState(pressed, hovered, enabled);
        const auto& theme = _style.theme();
        const auto rect = _button->rect();
        const auto spacing = theme.spacing;
        const auto hPadding = spacing * 2;
        const auto fgRect = rect.marginsRemoved({hPadding, 0, hPadding, 0});
        const auto& bgColor =
            _style.toolButtonBackgroundColor(mouse, ColorRole::Secondary);
        const auto& currentBgColor = _animManager.animateBackgroundColor(
            _button, bgColor, theme.animationDuration);
        const auto radius = theme.borderRadius;

        const auto iconSize = theme.iconSize;
        const auto pixmap =
            getPixmap(_button->icon(), iconSize, mouse, Qt::Unchecked);
        const auto pixmapX = fgRect.x();
        const auto pixmapY =
            fgRect.y() + (fgRect.height() - iconSize.height()) / 2;
        const auto pixmapRect = QRect{{pixmapX, pixmapY}, iconSize};

        QPainter p(_button);
        p.setPen(Qt::NoPen);
        p.setBrush(currentBgColor);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.drawRoundedRect(rect, radius, radius);
        p.drawPixmap(pixmapRect, pixmap);

        e->accept();
        return true;
    }

    return QObject::eventFilter(watched, e);
}

///------- MouseWheelBlockerEventFilter starts from here
MouseWheelBlockerEventFilter::MouseWheelBlockerEventFilter(QWidget* widget)
    : QObject(widget), m_widget(widget)
{
}

bool MouseWheelBlockerEventFilter::eventFilter(QObject* watched, QEvent* e)
{
    const auto type = e->type();
    if (type == QEvent::Wheel) {
        if (!m_widget->hasFocus()) {
            e->ignore();
            return true;
        }
    }

    return QObject::eventFilter(watched, e);
}

///------- TabBarEventFilter starts from here
class TabBarButtonEventFilter : public QObject
{
public:
    explicit TabBarButtonEventFilter(QTabBar* tabBar)
        : QObject(tabBar), _tabBar(tabBar)
    {
    }

protected:
    bool eventFilter(QObject* /*watched*/, QEvent* e) override
    {
        const auto type = e->type();
        if (type == QEvent::Leave || type == QEvent::Enter) {
            if (_tabBar) {
                _tabBar->update();
            }
        }

        return false;
    }

private:
    QTabBar* _tabBar{nullptr};
};

TabBarEventFilter::TabBarEventFilter(QlementineStyle& style, QTabBar* tabBar)
    : QObject(tabBar), m_tabBar(tabBar)
{
    // Tweak left/right buttons.
    const auto toolButtons = tabBar->findChildren<QToolButton*>();
    if (toolButtons.size() == 2) {
        auto* eventFilter = new TabBarButtonEventFilter(m_tabBar);

        m_leftButton = toolButtons.at(0);
        m_leftButton->setFocusPolicy(Qt::NoFocus);
        m_leftButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        m_leftButton->setFixedSize(m_leftButton->sizeHint());
        style.setAutoIconColorEnabled(m_leftButton, false);
        m_leftButton->installEventFilter(eventFilter);

        m_rightButton = toolButtons.at(1);
        m_rightButton->setFocusPolicy(Qt::NoFocus);
        m_rightButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        m_rightButton->setFixedSize(m_rightButton->sizeHint());
        style.setAutoIconColorEnabled(m_rightButton, false);
        m_rightButton->installEventFilter(eventFilter);
    }
}

bool TabBarEventFilter::eventFilter(QObject* watched, QEvent* e)
{
    switch (e->type()) {
        case QEvent::MouseButtonRelease: {
            const auto* event = static_cast<QMouseEvent*>(e);
            switch (event->button()) {
                case Qt::MiddleButton: { // Close tab
                    const auto tabIndex = m_tabBar->tabAt(event->pos());
                    if (tabIndex != -1 && m_tabBar->isTabVisible(tabIndex)) {
                        e->accept();
                        emit m_tabBar->tabCloseRequested(tabIndex);
                        return true;
                    }
                } break;
                case Qt::RightButton: { // Tab context menu
                    const auto tabIndex = m_tabBar->tabAt(event->pos());
                    if (tabIndex != -1 && m_tabBar->isTabVisible(tabIndex)) {
                        e->accept();
                        emit m_tabBar->customContextMenuRequested(event->pos());
                        return true;
                    }
                } break;
                default:
                    break;
            }

            // Trigger a whole painting refresh because the tabs painting order
            // and masking creates undesired visual artifacts.
            QTimer::singleShot(0, this, [this]() { m_tabBar->update(); });
        } break;
        case QEvent::Wheel: {
            const auto* event = static_cast<QWheelEvent*>(e);
            auto delta = event->pixelDelta().x();

            // If delta is null, it might be because we are on MacOS, using a
            // trackpad. So let's use angleDelta instead.
            if (delta == 0) {
                delta = event->angleDelta().y();
            }

            // Invert the value if necessary.
            if (event->inverted()) {
                delta = -delta;
            }

            if (delta > 0 && m_rightButton) {
                // delta > 0 : scroll to the right.
                m_rightButton->click();
                e->accept();
                return true;
            }
            if (delta < 0 && m_leftButton) {
                // delta < 0 : scroll to the left.
                m_leftButton->click();
                e->accept();
                return true;
            }

            e->ignore();
            return true;
        } break;
        case QEvent::HoverMove: {
            const auto* event = static_cast<QHoverEvent*>(e);
            const auto beginX = m_leftButton->x();
            if (event->position().x() > beginX) {
                m_tabBar->update();
            }
        } break;
        default:
            break;
    }

    return QObject::eventFilter(watched, e);
}

///------- MenuEventFilter starts from here
MenuEventFilter::MenuEventFilter(QMenu* menu) : QObject(menu), m_menu(menu)
{
    menu->installEventFilter(this);
}

bool MenuEventFilter::eventFilter(QObject* /*watch*/, QEvent* e)
{
    const auto type = e->type();
    if (type == QEvent::Type::Show) {
        // Place the QMenu correctly by making up for the drop shadow margins.
        // It'll be reset before every show, so we can safely move it every
        // time. Submenus should already be placed correctly, so there's no need
        // to translate their geometry. Also, make up for the menu item padding
        // so the texts are aligned.
        const bool isMenuBarMenu =
            qobject_cast<QMenuBar*>(m_menu->parentWidget());
        const bool isSubMenu = qobject_cast<QMenu*>(m_menu->parentWidget());
        const bool alignForMenuBar = isMenuBarMenu && !isSubMenu;
        const auto* theStyle = qobject_cast<QlementineStyle*>(m_menu->style());
        const auto menuItemHPadding = theStyle ? theStyle->theme().spacing : 0;
        const auto menuDropShadowWidth =
            theStyle ? theStyle->theme().spacing : 0;
        const auto menuRect = m_menu->geometry();
        const auto menuBarTranslation =
            alignForMenuBar ? QPoint(-menuItemHPadding, 0) : QPoint(0, 0);
        const QPoint shadowTranslation{-menuDropShadowWidth,
                                       -menuDropShadowWidth};
        const auto newMenuRect =
            menuRect.translated(menuBarTranslation + shadowTranslation);
        m_menu->setGeometry(newMenuRect);
    }

    return false;
}

///------- ComboboxItemViewFilter starts from here
ComboboxItemViewFilter::ComboboxItemViewFilter(QAbstractItemView* view)
    : QObject(view), m_view(view)
{
    view->installEventFilter(this);
}

bool ComboboxItemViewFilter::eventFilter(QObject* /*watched*/, QEvent* e)
{
    const auto type = e->type();
    if (type == QEvent::Type::Show) {
        // Fix Qt bug.
        const auto width = m_view->sizeHintForColumn(0);
        m_view->setMinimumWidth(width);
    }
    return false;
}

} // namespace tl

#include "moc_EventFilters.cpp"

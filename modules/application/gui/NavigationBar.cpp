#include "NavigationBar.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QToolTip>

NavigationBar::NavigationBar(QWidget* parent) : QWidget(parent)
{
    m_currentIndex = 0;
    m_bgColor = {0xE4, 0xE4, 0xE4};
    m_selectedColor = {0x2C, 0xA7, 0xF8};
    m_rowHeight = 40;

    setMouseTracking(true);
    setFixedWidth(150);
}

NavigationBar::~NavigationBar() = default;

void NavigationBar::setBackgroundColor(const QColor& color)
{
    if (color.isValid()) {
        m_bgColor = color;
    }

    update();
}

void NavigationBar::setSelectedColor(const QColor& color)
{
    if (color.isValid()) {
        m_selectedColor = color;
    }

    update();
}

void NavigationBar::setWidth(int width) { setFixedWidth(width); }

void NavigationBar::setRowHeight(int height)
{
    if (m_rowHeight) {
        m_rowHeight = height;
    }

    update();
}

int NavigationBar::addItem(const QString& item, const QString& toolTips)
{
    return insertItem(-1, item, toolTips);
}

int NavigationBar::insertItem(int index, const QString& item,
                              const QString& toolTips)
{
    int newIndex = index;
    if (!item.isEmpty()) {
        if (index != -1) {
            m_tabNames.insert(index, item);
            m_tabToolTips.insert(index, toolTips);
        }
        else // Add
        {
            m_tabNames << item;
            m_tabToolTips << toolTips;

            newIndex = m_tabNames.size() - 1;
        }
    }

    update();

    return newIndex;
}

void NavigationBar::deleteItem(int index)
{
    if (index < m_tabNames.size()) {
        if (index == m_currentIndex) {
            m_currentIndex = 0; // Reset the currentIndex
        }

        m_tabNames.removeAt(index);
        m_tabToolTips.removeAt(index);

        update(); // Repaint
    }
}

void NavigationBar::setItemText(int index, const QString& text)
{
    if (index < m_tabNames.size()) {
        m_tabNames[index] = text;

        update();
    }
}

void NavigationBar::setItemToolTip(int index, const QString& itemToolTip)
{
    if (index < m_tabNames.size()) {
        m_tabToolTips[index] = itemToolTip;
    }
}

int NavigationBar::currentIndex() const { return m_currentIndex; }

int NavigationBar::count() const { return m_tabNames.size(); }

void NavigationBar::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // Draw background
    painter.setPen(Qt::NoPen);
    painter.setBrush(m_bgColor);
    painter.drawRect(rect());

    // Draw items
    int count = 0;
    for (const auto& item : m_tabNames) {
        QPainterPath itemPath;
        // QRect(int x, int y, int width, int height)
        // Constructs a rectangle with (x, y) as its top-left corner and the
        // given width and height.
        itemPath.addRect(QRect(0, count * m_rowHeight, width(), m_rowHeight));

        if (count == m_currentIndex) {
            painter.setPen(Qt::white);
            painter.fillPath(itemPath, QBrush{m_selectedColor});
        }
        else {
            painter.setPen(QColor{0x20, 0x20, 0x20});
            painter.fillPath(itemPath, QBrush{m_bgColor});
        }

        // Draw text
        painter.drawText(QRect(0, count * m_rowHeight, width(), m_rowHeight),
                         Qt::AlignVCenter | Qt::AlignHCenter, item);

        ++count;
    }
}

void NavigationBar::mousePressEvent(QMouseEvent* event)
{
    // Get the cursor position
    if (event->position().y() / m_rowHeight < m_tabNames.count()) {
        m_currentIndex = event->position().y() / m_rowHeight;

        emit currentTabChanged(m_currentIndex);

        update();
    }
}

void NavigationBar::mouseReleaseEvent(QMouseEvent* event) { Q_UNUSED(event); }

void NavigationBar::mouseMoveEvent(QMouseEvent* event)
{
    if (event->position().y() / m_rowHeight < m_tabNames.count()) {
        // Do something here.
        int index = event->position().y() / m_rowHeight;
        QToolTip::showText(event->globalPosition().toPoint(),
                           m_tabToolTips.at(index));
    }
}

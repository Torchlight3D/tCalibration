#include "qtlistwidget.h"

#include <QBoxLayout>
#include <QDrag>
#include <QDragMoveEvent>
#include <QLabel>
#include <QTimer>
#include <QWindow>

namespace tl {

namespace {
QPalette kDefaultBackground;
QPalette kHighlightBackground;
QPalette kSelectedBackground;
std::once_flag kInitPalettesFlag;

static std::vector<QtListWidgetItem*> kHighlightedItems;
} // namespace

QtListWidgetItem::QtListWidgetItem(QWidget* parent) : QWidget(parent)
{
    setBackgroundRole(QPalette::ColorRole::Base);
    setAutoFillBackground(true);

    std::call_once(kInitPalettesFlag, [this]() {
        kDefaultBackground = palette();

        kHighlightBackground = palette();
        kHighlightBackground.setColor(
            QPalette::ColorRole::Base,
            kHighlightBackground.color(QPalette::Highlight).lighter(210));

        kSelectedBackground = palette();
        kSelectedBackground.setColor(
            QPalette::ColorRole::Base,
            kSelectedBackground.color(QPalette::Highlight));
    });
}

QtListWidgetItem::~QtListWidgetItem()
{
    if (auto found = std::ranges::find(kHighlightedItems, this);
        found != kHighlightedItems.end()) {
        kHighlightedItems.erase(found);
    }
}

bool QtListWidgetItem::isSelected() const { return m_selected; }

void QtListWidgetItem::select(bool on)
{
    m_selected = on;
    update();
}

void QtListWidgetItem::enterEvent(QEnterEvent* event)
{
    QWidget::enterEvent(event);
    highlightBackground(true);
}

void QtListWidgetItem::leaveEvent(QEvent* event)
{
    QWidget::leaveEvent(event);
    highlightBackground(false);
}

void QtListWidgetItem::highlightBackground(bool on)
{
    if (on) {
        if (!kHighlightedItems.empty()) {
            auto& item = kHighlightedItems.back();
            item->setPalette(item->m_selected ? kSelectedBackground
                                              : kDefaultBackground);
            item->update();
        }

        kHighlightedItems.push_back(this);

        setPalette(kHighlightBackground);
    }
    else {
        setPalette(m_selected ? kSelectedBackground : kDefaultBackground);

        if (auto found = std::ranges::find(kHighlightedItems, this);
            found != kHighlightedItems.end()) {
            kHighlightedItems.erase(found);
        }

        if (!kHighlightedItems.empty()) {
            auto& item = kHighlightedItems.back();
            item->setPalette(kHighlightBackground);
            item->update();
        }
    }

    update();
}

class QtListWidgetPrivate
{
    Q_DEFINE_PIMPL(QtListWidget);

public:
    explicit QtListWidgetPrivate(QtListWidget* q);

    void init();

    QtListWidgetItem* findItemAt(const QPoint& pos) const;

    // Placeholder is showed when items is empty
    void updatePlaceholder();

    // Ensure the widget has enough width to show its items.
    void propagateMinimumDimension();

public:
    QVBoxLayout* m_layout{nullptr};
    QLabel* m_placeholder{nullptr};
    QFrame* m_list{nullptr};
    QList<QtListWidgetItem*> m_items; // Owned
    bool m_selectable{false};
};

QtListWidgetPrivate::QtListWidgetPrivate(QtListWidget* q) : q_ptr(q) {}

void QtListWidgetPrivate::init()
{
    // ...
}

void QtListWidgetPrivate::updatePlaceholder()
{
    Q_Q(QtListWidget);
    // FIXME: Still some overlay left on the GUI
    m_placeholder->setVisible(q->allItems().empty());
}

void QtListWidgetPrivate::propagateMinimumDimension()
{
    Q_Q(QtListWidget);

    if (m_items.empty()) {
        return;
    }

    const auto maxMinWidthItem =
        std::ranges::max(m_items, [](const auto* lhs, const auto* rhs) {
            return lhs->sizeHint().width() < rhs->sizeHint().width();
        });

    const auto newWidth = maxMinWidthItem->sizeHint().width() +
                          q->contentsMargins().left() +
                          q->contentsMargins().right();

    q->setMinimumWidth(newWidth);
}

QtListWidgetItem* QtListWidgetPrivate::findItemAt(const QPoint& pos) const
{
    Q_Q(const QtListWidget);
    for (auto child = m_list->childAt(pos); child;
         child = child->parentWidget()) {
        if (auto item = dynamic_cast<QtListWidgetItem*>(child)) {
            return item;
        }
    }

    return nullptr;
}

QtListWidget::QtListWidget(bool stretch, QWidget* parent)
    : QScrollArea(parent), d_ptr(new QtListWidgetPrivate(this))
{
    Q_D(QtListWidget);

    d->m_list = new QFrame(this);
    d->m_list->setBackgroundRole(QPalette::ColorRole::Base);
    d->m_list->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    d->m_list->setMinimumSize(20, 20);
    if (!stretch) {
        d->m_list->setFrameStyle(QFrame::Box);
    }

    d->m_placeholder = new QLabel(this);
    d->m_placeholder->setForegroundRole(QPalette::ColorRole::Mid);
    d->m_placeholder->setVisible(true);

    d->m_layout = new QVBoxLayout(this);
    d->m_layout->setSizeConstraint(QLayout::SetMinimumSize);
    d->m_layout->setContentsMargins(2, 2, 2, 2);
    d->m_layout->setSpacing(0);
    d->m_layout->addWidget(d->m_placeholder);
    if (stretch) {
        d->m_layout->addStretch(0);
    }

    setWidget(d->m_list);

    setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    setWidgetResizable(false);
    setSizeAdjustPolicy(SizeAdjustPolicy::AdjustToContentsOnFirstShow);
}

QtListWidget::~QtListWidget() = default;

QString QtListWidget::placeholderText() const
{
    Q_D(const QtListWidget);
    return d->m_placeholder->text();
}

void QtListWidget::setPlaceholderText(const QString& text)
{
    Q_D(QtListWidget);
    d->m_placeholder->setText(text);
}

bool QtListWidget::selectable() const
{
    Q_D(const QtListWidget);
    return d->m_selectable;
}

void QtListWidget::setSelectable(bool on)
{
    Q_D(QtListWidget);
    if (d->m_selectable == on) {
        return;
    }

    d->m_selectable = on;
}

QtListWidgetItem* QtListWidget::addItem(QtListWidgetItem* item, int index)
{
    if (!item) {
        return nullptr;
    }

    Q_D(QtListWidget);
    if (const auto found = std::ranges::find(d->m_items, item);
        found != d->m_items.end()) {
        return *found;
    }

    // Invalid index go to the end.
    if (index < 0 || index > d->m_items.count()) {
        index = d->m_items.count();
    }

    item->setParent(this);

    d->m_items.insert(index, item);

    // NOTE: There are two item already in the layout:
    // Empty placeholder at the beginning, and spacing in the end.
    d->m_layout->insertWidget(index + 1, item);
    d->m_list->setMinimumWidth(std::max(
        d->m_list->minimumWidth(), item->sizeHint().width() +
                                       d->m_list->contentsMargins().left() +
                                       d->m_list->contentsMargins().right()));

    d->propagateMinimumDimension();

    emit itemChanged();

    return item;
}

void QtListWidget::removeItem(QtListWidgetItem* item)
{
    Q_D(QtListWidget);
    if (!d->m_items.removeOne(item)) {
        return;
    }

    d->m_layout->removeWidget(item);
    d->m_layout->update();

    item->deleteLater();

    emit itemChanged();
}

QList<QtListWidgetItem*> QtListWidget::allItems(bool onlySelected) const
{
    Q_D(const QtListWidget);
    if (!onlySelected) {
        return d->m_items;
    }

    QList<QtListWidgetItem*> items;
    for (auto& item : d->m_items) {
        items << item;
    }
    return items;
}

void QtListWidget::clear()
{
    Q_D(QtListWidget);
    for (auto& item : d->m_items) {
        d->m_layout->removeWidget(item);
        item->deleteLater();
    }
    d->m_items.clear();

    d->m_layout->update();

    emit itemChanged();
}

void QtListWidget::mousePressEvent(QMouseEvent* event)
{
    Base::mousePressEvent(event);

    Q_D(QtListWidget);
    auto* item = d->findItemAt(event->pos());
    if (!item) {
        return;
    }

    // TODO: Do something here?
}

void QtListWidget::mouseReleaseEvent(QMouseEvent* event)
{
    Q_D(QtListWidget);
    if (auto* item = d->findItemAt(event->pos())) {
        item->select(!item->isSelected());
    }

    // TODO: Do something here?
}

void QtListWidget::childEvent(QChildEvent* event)
{
    Base::childEvent(event);

    Q_D(QtListWidget);
    if (event->type() == QEvent::ChildRemoved ||
        event->type() == QEvent::ChildAdded) {
        QTimer::singleShot(1, this, [d]() {
            d->updatePlaceholder();
            d->propagateMinimumDimension();
        });
    }
}

} // namespace tl

#include "moc_qtlistwidget.cpp"

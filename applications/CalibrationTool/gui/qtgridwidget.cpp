#include "qtgridwidget.h"

#include <QEvent>
#include <QGridLayout>
#include <QMenu>
#include <QStyle>

namespace tl {

namespace {
constexpr QChar kBullet{0x2022};
}

class QtGridWidgetPrivate
{
    Q_DEFINE_PIMPL(QtGridWidget)

public:
    explicit QtGridWidgetPrivate(QtGridWidget *q);

    void init();

    void addLayoutSubmenu(QMenu *menu, QtGridWidget::LayoutStyle type);

    void doLayoutAndShowPage(int viewsPerPage, qsizetype page,
                             bool force = false);
    inline void restoreLayout()
    {
        doLayoutAndShowPage(m_currViewsPerPage, m_currPage, true);
    }
    void doLayout(int viewsPerPage, qsizetype start);
    inline void doLayoutSingleView(qsizetype start)
    {
        doLayoutSquare(start, 1);
    }
    void doLayoutDualViews(qsizetype start);
    inline void doLayoutFourViews(qsizetype start) { doLayoutSquare(start, 2); }
    void doLayoutSixViews(qsizetype start);
    void doLayoutEightViews(qsizetype start);
    inline void doLayoutNineViews(qsizetype start) { doLayoutSquare(start, 3); }
    void doLayoutSquare(qsizetype start, int edge);

    void hideAllWidgets();

public:
    QGridLayout *m_grid;   // Owned, manage by QObject
    QWidgetList m_widgets; // Not owned
    int m_currViewsPerPage{QtGridWidget::DualViews};
    qsizetype m_currPage{0};
    bool m_maximized{false};
};

QtGridWidgetPrivate::QtGridWidgetPrivate(QtGridWidget *q)
    : q_ptr(q), m_grid(new QGridLayout(q))
{
}

void QtGridWidgetPrivate::init()
{
    Q_Q(QtGridWidget);

    /// Parent configs
    q->setContextMenuPolicy(Qt::CustomContextMenu);

    /// Self configs
    m_grid->setContentsMargins(9, 9, 9, 9);
    m_grid->setSpacing(6);
}

void QtGridWidgetPrivate::addLayoutSubmenu(QMenu *menu,
                                           QtGridWidget::LayoutStyle type)
{
    Q_Q(QtGridWidget);

    const int viewsPerPage = type;

    const bool sameLayout = type == m_currViewsPerPage;

    auto subMenu = menu->addMenu(
        QtGridWidget::tr("%1 Views").arg(QString::number(viewsPerPage)));
    if (sameLayout) {
        subMenu->setIcon(q->style()->standardIcon(QStyle::SP_CommandLink));
    }

    if (m_widgets.empty()) {
        subMenu->setEnabled(false);
        return;
    }

    const auto numViews = m_widgets.size();
    const auto numPages =
        numViews / viewsPerPage + (numViews % viewsPerPage != 0);
    for (qsizetype page{0}; page < numPages; ++page) {
        const auto start = page * viewsPerPage;
        const auto end = std::min(start + viewsPerPage, numViews) - 1;

        const auto name =
            QtGridWidget::tr("View %1-%2")
                .arg(QString::number(start + 1), QString::number(end + 1));

        auto *action = subMenu->addAction(name, q, [this, q]() {
            const auto *action = qobject_cast<const QAction *>(q->sender());
            const auto viewsPerPage =
                action->property("views_per_page").toInt();
            const auto page = action->property("page").toLongLong();

            doLayoutAndShowPage(viewsPerPage, page);
        });
        action->setProperty("views_per_page", viewsPerPage);
        action->setProperty("page", page);
        if (sameLayout && page == m_currPage) {
            action->setIcon(q->style()->standardIcon(QStyle::SP_CommandLink));
        }
    }
}

void QtGridWidgetPrivate::doLayoutAndShowPage(int viewsPerPage, qsizetype page,
                                              bool forceUpdate)
{
    const bool sameLayout = m_currViewsPerPage == viewsPerPage;
    const bool samePage = m_currPage == page;

    if (!forceUpdate && sameLayout && samePage) {
        return;
    }

    if (!sameLayout) {
        m_currViewsPerPage = viewsPerPage;
    }
    if (!samePage) {
        m_currPage = page;
    }

    doLayout(m_currViewsPerPage, m_currPage * m_currViewsPerPage);
}

void QtGridWidgetPrivate::doLayout(int viewsPerPage, qsizetype start)
{
    switch (viewsPerPage) {
        case 1:
            doLayoutSingleView(start);
            break;
        case 2:
            doLayoutDualViews(start);
            break;
        case 4:
            doLayoutFourViews(start);
            break;
        case 6:
            doLayoutSixViews(start);
            break;
        case 8:
            doLayoutEightViews(start);
            break;
        case 9:
            doLayoutNineViews(start);
            break;
        default:
            break;
    }
}

void QtGridWidgetPrivate::doLayoutDualViews(qsizetype start)
{
    hideAllWidgets();

    m_grid->addWidget(m_widgets[start], 0, 0);
    m_widgets[start]->setVisible(true);

    if (auto next = start + 1; next < m_widgets.size()) {
        m_grid->addWidget(m_widgets[next], 0, 1);
        m_widgets[next]->setVisible(true);
    }
}

//
//   -------------
//   |       | 1 |
//   -   0   -----
//   |       | 2 |
//   -------------
//   | 5 | 4 | 3 |
//   -------------
//
// FIXME: The layout in last page is not ideal if it can't be filled.
void QtGridWidgetPrivate::doLayoutSixViews(qsizetype start)
{
    hideAllWidgets();

    std::vector<qsizetype> indices;
    indices.reserve(6);
    for (auto i = start, end = std::min(start + 6, m_widgets.size()); i < end;
         ++i) {
        indices.push_back(i);
    }

    constexpr std::array kRows{0, 0, 1, 2, 2, 2};
    constexpr std::array kCols{0, 2, 2, 2, 1, 0};
    constexpr std::array kRowSpan{2, 1, 1, 1, 1, 1};
    constexpr std::array kColSpan{2, 1, 1, 1, 1, 1};

    for (size_t i{0}; i < indices.size(); ++i) {
        auto &widget = m_widgets[indices[i]];
        m_grid->addWidget(widget, kRows[i], kCols[i], kRowSpan[i], kColSpan[i]);
        widget->setVisible(true);
    }
}

//
//   -----------------
//   |           | 1 |
//   -           -----
//   |     0     | 2 |
//   -           -----
//   |           | 3 |
//   -----------------
//   | 7 | 6 | 5 | 4 |
//   -----------------
//
// FIXME: The layout in last page is not ideal if it can't be filled.
void QtGridWidgetPrivate::doLayoutEightViews(qsizetype start)
{
    hideAllWidgets();

    std::vector<qsizetype> indices;
    indices.reserve(8);
    for (auto i = start, end = std::min(start + 8, m_widgets.size()); i < end;
         ++i) {
        indices.push_back(i);
    }

    constexpr std::array kRows{0, 0, 1, 2, 3, 3, 3, 3};
    constexpr std::array kCols{0, 3, 3, 3, 3, 2, 1, 0};
    constexpr std::array kRowSpan{3, 1, 1, 1, 1, 1, 1, 1};
    constexpr std::array kColSpan{3, 1, 1, 1, 1, 1, 1, 1};

    for (size_t i{0}; i < indices.size(); ++i) {
        auto &widget = m_widgets[indices[i]];
        m_grid->addWidget(widget, kRows[i], kCols[i], kRowSpan[i], kColSpan[i]);
        widget->setVisible(true);
    }
}

// From top-left corner to bottom-right corner by rows.
void QtGridWidgetPrivate::doLayoutSquare(qsizetype start, int edge)
{
    hideAllWidgets();

    const auto end = std::min(start + m_currViewsPerPage, m_widgets.size());
    for (auto i = start; i < end; ++i) {
        const auto i_norm = i - start;
        auto &widget = m_widgets[i];
        m_grid->addWidget(widget, i_norm / edge, i_norm % edge);
        widget->setVisible(true);
    }
}

void QtGridWidgetPrivate::hideAllWidgets()
{
    for (auto &widget : m_widgets) {
        m_grid->removeWidget(widget);
        widget->setVisible(false);
    }
}

///------- QtGridWidget starts from here
QtGridWidget::QtGridWidget(QWidget *parent)
    : QWidget(parent), d_ptr(new QtGridWidgetPrivate(this))
{
    Q_D(QtGridWidget);
    d->init();

    connect(this, &QWidget::customContextMenuRequested, this,
            [this, d](const QPoint &pos) {
                QMenu menu;
                menu.addAction(tr("Lock"), this, []() {
                    qDebug() << "Lock layout is not implemented.";
                });
                menu.addAction(tr("Rotate"), this, []() {
                    qDebug() << "Rotate views is not implemented.";
                });

                menu.addSeparator();

                for (const auto &type : {SingleView, DualViews, FourViews,
                                         SixViews, EightViews, NineViews}) {
                    d->addLayoutSubmenu(&menu, type);
                }

                menu.exec(mapToGlobal(pos));
            });
}

QtGridWidget::~QtGridWidget() = default;

QtGridWidget::LayoutStyle QtGridWidget::layoutStyle() const
{
    Q_D(const QtGridWidget);
    return static_cast<LayoutStyle>(d->m_currViewsPerPage);
}

void QtGridWidget::setLayoutStyle(LayoutStyle style)
{
    Q_D(QtGridWidget);
    if (d->m_currViewsPerPage == style) {
        return;
    }

    d->m_currViewsPerPage = style;

    if (!d->m_widgets.empty()) {
        d->restoreLayout();
    }
}

void QtGridWidget::bindWidgets(QWidgetList widgets)
{
    if (std::ranges::any_of(widgets, [](const auto &w) { return !w; })) {
        return;
    }

    Q_D(QtGridWidget);
    d->m_widgets = widgets;
    d->m_currPage = 0;

    d->restoreLayout();
}

void QtGridWidget::showWidgetAt(qsizetype index)
{
    Q_D(QtGridWidget);
    if (!d->m_widgets.empty() || index >= d->m_widgets.size()) {
        return;
    }

    d->doLayoutAndShowPage(d->m_currViewsPerPage,
                           index / d->m_currViewsPerPage);
}

bool QtGridWidget::eventFilter(QObject *watched, QEvent *event)
{
    Q_D(QtGridWidget);
    // Double click view to maximize, or restore layout
    // WARNING: Currently not available
    if (event->type() == QEvent::MouseButtonDblClick) {
        auto *widget = qobject_cast<QWidget *>(watched);
        if (widget == this) {
            return true;
        }

        if (!d->m_maximized) {
            d->m_maximized = true;

            d->hideAllWidgets();
            d->m_grid->addWidget(widget, 0, 0);
            widget->setVisible(true);
        }
        else {
            d->m_maximized = false;

            d->restoreLayout();
        }

        return true;
    }

    return QWidget::eventFilter(watched, event);
}

} // namespace tl

#include "moc_qtgridwidget.cpp"

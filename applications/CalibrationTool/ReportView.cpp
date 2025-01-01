#include "ReportView.h"

#include <QAbstractTableModel>
#include <QBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTableView>

namespace tl {

using namespace Qt::Literals::StringLiterals;

///------- ReportModel starts from here
class ReportModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum class Section
    {
        Item = 0,
        Value,
        Expectation,

        Count,
    };

    using QAbstractTableModel::QAbstractTableModel;

    int rowCount(const QModelIndex &parent) const override;
    int columnCount(const QModelIndex &parent) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role) const override;

    QVariant data(const QModelIndex &index, int role) const override;

    void show(const ReportSummary &summary);

    inline void resetModel()
    {
        beginResetModel();
        endResetModel();
    }

private:
    static QString sectionName(Section section)
    {
        static const std::unordered_map<Section, QString> map{
            {Section::Item, tr("Item")},
            {Section::Value, tr("Value")},
            {Section::Expectation, tr("Expectation")}};
        return map.at(section);
    }

private:
    ReportSummary m_summary;
};

int ReportModel::rowCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? m_summary.itemCount() : 0;
}

int ReportModel::columnCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? static_cast<int>(Section::Count) : 0;
}

Qt::ItemFlags ReportModel::flags(const QModelIndex &index) const
{
    if (!index.isValid()) {
        return Qt::ItemIsEnabled;
    }

    return QAbstractTableModel::flags(index);
}

QVariant ReportModel::headerData(int section, Qt::Orientation orientation,
                                 int role) const
{
    if (role != Qt::DisplayRole) {
        return {};
    }

    if (orientation == Qt::Horizontal) {
        return sectionName(static_cast<Section>(section));
    }

    return {};
}

QVariant ReportModel::data(const QModelIndex &index, int role) const
{
    const int row = index.row();
    if (!index.isValid() || index.row() >= m_summary.itemCount()) {
        return {};
    }

    const auto &item = m_summary.items().at(row);
    switch (role) {
        case Qt::DisplayRole: {
            switch (static_cast<Section>(index.column())) {
                case Section::Item:
                    return item.name;
                case Section::Value:
                    return item.value;
                case Section::Expectation:
                    return item.expectation;
                default:
                    break;
            }
        } break;
        case Qt::BackgroundRole: {
            if (!item.used) {
                return QColor{Qt::lightGray};
            }
            if (!item.pass) {
                return QColor{Qt::red};
            }
        } break;
        case Qt::ToolTipRole: {
            return item.toolTip;
        }
        default:
            break;
    }

    return {};
}

void ReportModel::show(const ReportSummary &summary) { m_summary = summary; }

///------- ReportViewPrivate starts from here
class ReportViewPrivate
{
    Q_DEFINE_PIMPL(ReportView);

public:
    explicit ReportViewPrivate(ReportView *q);

    void init();

    void setPlaceholderVisible(bool visible);

public:
    QStackedWidget *m_stack;        // Owned
    QLabel *m_placeholder;          // Owned
    QTabWidget *m_reportTabs;       // Owned
    QList<QWidget *> m_reportViews; // Owned
};

ReportViewPrivate::ReportViewPrivate(ReportView *q)
    : q_ptr(q),
      m_stack(new QStackedWidget(q)),
      m_placeholder(new QLabel(q)),
      m_reportTabs(new QTabWidget(q))
{
}

void ReportViewPrivate::init()
{
    Q_Q(ReportView);
    m_placeholder->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    m_placeholder->setText(ReportView::tr("Waiting for result summaries."));
    m_placeholder->setStyleSheet(u"font: bold italic 13px; "
                                 "background-color: #d4d4d4; "_s);

    m_stack->addWidget(m_placeholder);
    m_stack->addWidget(m_reportTabs);

    auto *layout = new QVBoxLayout(q);
    layout->setContentsMargins({6, 6, 6, 6});
    layout->setSpacing(3);
    layout->addWidget(m_stack);

    setPlaceholderVisible(true);
}

void ReportViewPrivate::setPlaceholderVisible(bool visible)
{
    m_stack->setCurrentIndex(visible ? m_stack->indexOf(m_placeholder)
                                     : m_stack->indexOf(m_reportTabs));
}

///------- ReportView starts from here
ReportView::ReportView(QWidget *parent)
    : QWidget(parent), d_ptr(new ReportViewPrivate(this))
{
    d_ptr->init();
}

ReportView::~ReportView() = default;

void ReportView::showSummaries(const std::vector<ReportSummary> &summaries)
{
    Q_D(ReportView);
    if (summaries.empty()) {
        return;
    }

    if (d->m_reportTabs->count() != 0) {
        clear();
    }

    for (size_t i{0}; i < summaries.size(); ++i) {
        // NOTE: QTabWidget will take ownership of tab in addTab(), but it won't
        // delete the tab in removeTab()/clear()
        auto *page = new QTableView;
        auto *model = new ReportModel(page);
        model->show(summaries[i]);
        page->setModel(model);
        page->setTextElideMode(Qt::ElideNone);
        page->resizeRowsToContents();
        page->horizontalHeader()->setSectionResizeMode(
            QHeaderView::ResizeToContents);

        [[maybe_unused]] const auto index = d->m_reportTabs->addTab(
            page, tr("Task %1").arg(QString::number(i + 1)));
        d->m_reportViews.push_back(page);
    }

    d->m_reportTabs->setCurrentIndex(0);

    hidePlaceholder();
}

void ReportView::clear()
{
    Q_D(ReportView);
    // Clear all ReportViews
    for (auto &page : d->m_reportViews) {
        page->deleteLater();
        page = nullptr;
    }
    d->m_reportViews.clear();
    d->m_reportTabs->clear();

    showPlaceholder();
}

void ReportView::showPlaceholder(bool show)
{
    Q_D(ReportView);
    d->setPlaceholderVisible(show);
}

} // namespace tl

#include "ReportView.moc"
#include "moc_ReportView.cpp"

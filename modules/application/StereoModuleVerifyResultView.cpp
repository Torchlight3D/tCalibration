#include "StereoModuleVerifyResultView.h"

#include <array>

#include <QAbstractTableModel>
#include <QBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QStackedWidget>
#include <QTableView>

#include "gui/qstringutils.h"

namespace thoht {

///------- VerifyResultReportModel starts from here
class VerifyResultReportModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    using QAbstractTableModel::QAbstractTableModel;

    int rowCount(const QModelIndex &parent) const override;
    int columnCount(const QModelIndex &parent) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role) const override;

    QVariant data(const QModelIndex &index, int role) const override;

    void show(const StereoCameraVerification::Reference &ref,
              const StereoCameraVerification::Summary &summary);

    inline void resetModel()
    {
        beginResetModel();
        endResetModel();
    }

    static QString toTextTable(
        const StereoCameraVerification::Reference &ref,
        const StereoCameraVerification::Summary &summary);

private:
    enum class Section
    {
        Item = 0,
        Status,
        Value,
        Expectation,

        Count,
    };
    static QString sectionName(Section section)
    {
        static const std::unordered_map<Section, QString> map{
            {Section::Item, tr("Item")},
            {Section::Status, tr("Status")},
            {Section::Value, tr("Value")},
            {Section::Expectation, tr("Expectation")}};
        return map.at(section);
    }

private:
    using Item = StereoModuleVerifyResultView::VerifyItem;
    using Status = StereoModuleVerifyResultView::VerifyStatus;

    struct VerifyItem
    {
        QString name;
        QString valueString;
        QString refString;
        Status status{Status::NA};
    };
    using Data = std::array<VerifyItem, Item::Count>;

    Data m_items;
};

int VerifyResultReportModel::rowCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? m_items.size() : 0;
}

int VerifyResultReportModel::columnCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? static_cast<int>(Section::Count) : 0;
}

Qt::ItemFlags VerifyResultReportModel::flags(const QModelIndex &index) const
{
    if (!index.isValid()) {
        return Qt::ItemIsEnabled;
    }

    return QAbstractTableModel::flags(index);
}

QVariant VerifyResultReportModel::headerData(int section,
                                             Qt::Orientation orientation,
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

QVariant VerifyResultReportModel::data(const QModelIndex &index, int role) const
{
    const int row = index.row();
    if (!index.isValid() || index.row() >= m_items.size()) {
        return {};
    }

    const auto &item = m_items.at(row);
    switch (role) {
        case Qt::DisplayRole: {
            switch (static_cast<Section>(index.column())) {
                case Section::Item:
                    return item.name;
                case Section::Status:
                    return StereoModuleVerifyResultView::verifyStatusName(
                        item.status);
                case Section::Value:
                    return item.valueString;
                case Section::Expectation:
                    return item.refString;
                default:
                    break;
            }
        } break;
        case Qt::BackgroundRole: {
            switch (item.status) {
                case Status::Failed:
                    return QColor{Qt::red};
                case Status::NA:
                    return QColor{Qt::lightGray};
                default:
                    break;
            }
        } break;
        default:
            break;
    }

    return {};
}

void VerifyResultReportModel::show(
    const StereoCameraVerification::Reference &ref,
    const StereoCameraVerification::Summary &summary)
{
    using Error = StereoCameraVerification::Summary::Error;

    const auto &error = summary.error;
    if (error == Error::InvalidImage ||
        error == Error::FailedFindChessboardCorner) {
        // TODO: Do sth here
        return;
    }

    auto updateItem = [this, &ref, &summary](Item itemType) {
        auto &item = m_items[itemType];
        item.status =
            StereoModuleVerifyResultView::verifyItemStatus(summary, itemType);
        item.name = StereoModuleVerifyResultView::verifyItemName(itemType);
        item.valueString =
            StereoModuleVerifyResultView::verifyItemValue(summary, itemType);
        item.refString =
            StereoModuleVerifyResultView::verifyItemReference(ref, itemType);
    };

    updateItem(Item::TrackerRejectRate);
    updateItem(Item::EstimatorRejectRate);
    updateItem(Item::PoseOptimizationCost);
    updateItem(Item::RejectRelativeSizeMedian);
    updateItem(Item::RejectRelativePositionMedian);

    resetModel();
}

QString VerifyResultReportModel::toTextTable(
    const StereoCameraVerification::Reference &ref,
    const StereoCameraVerification::Summary &summary)
{
    QString report;
    QTextStream out{&report};
    out << tr("Verification result report:") << "\n";

    const auto &error = summary.error;

    constexpr int kReportFieldWidth{30};
    out.setFieldWidth(kReportFieldWidth);
    out.setPadChar(' ');

    /// Header
    out.setFieldAlignment(QTextStream::AlignCenter);
    out << sectionName(Section::Item) << sectionName(Section::Value)
        << sectionName(Section::Expectation) << sectionName(Section::Status);
    qstr::noPaddingNewline(out, kReportFieldWidth);

    /// Body
    auto appendItem = [&ref, &summary](QTextStream &out, Item item) {
        out << StereoModuleVerifyResultView::verifyItemName(item)
            << StereoModuleVerifyResultView::verifyItemValue(summary, item)
            << StereoModuleVerifyResultView::verifyItemReference(ref, item)
            << StereoModuleVerifyResultView::verifyItemStatusName(summary,
                                                                  item);
    };

    appendItem(out, Item::TrackerRejectRate);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::EstimatorRejectRate);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::PoseOptimizationCost);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::RejectRelativeSizeMedian);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::RejectRelativePositionMedian);
    out << Qt::endl;

    return report;
}

///------- StereoModuleVerifyResultViewPrivate starts from here
class StereoModuleVerifyResultViewPrivate
{
    Q_DEFINE_PIMPL(StereoModuleVerifyResultView)

public:
    explicit StereoModuleVerifyResultViewPrivate(
        StereoModuleVerifyResultView *q);

    void init();

    void setPlaceholderVisible(bool visible);

public:
    StereoCameraVerification::Reference m_ref;
    QStackedWidget *m_stack;
    QTableView *m_view;
    int m_placeHolderIndex{-1};
    int m_viewIndex{-1};
};

StereoModuleVerifyResultViewPrivate::StereoModuleVerifyResultViewPrivate(
    StereoModuleVerifyResultView *q)
    : q_ptr(q), m_stack(new QStackedWidget(q)), m_view(new QTableView(q))
{
}

void StereoModuleVerifyResultViewPrivate::init()
{
    Q_Q(StereoModuleVerifyResultView);

    m_view->setTextElideMode(Qt::ElideNone);
    m_view->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);

    auto placeHolder = new QLabel(
        StereoModuleVerifyResultView::tr("Waiting for verification results."),
        q);
    placeHolder->setProperty("placeholder", true);
    placeHolder->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

    m_placeHolderIndex = m_stack->addWidget(placeHolder);
    m_viewIndex = m_stack->addWidget(m_view);

    auto *layout = new QVBoxLayout(q);
    layout->setContentsMargins({6, 6, 6, 6});
    layout->setSpacing(0);
    layout->addWidget(m_stack);

    q->setStyleSheet(u"QLabel[placeholder=true] { "
                     "font: bold italic 13px;"
                     "background-color: #d4d4d4; }"_s);

    setPlaceholderVisible(true);
}

void StereoModuleVerifyResultViewPrivate::setPlaceholderVisible(bool visible)
{
    m_stack->setCurrentIndex(visible ? m_placeHolderIndex : m_viewIndex);
}

///------- StereoModuleVerifyResultView starts from here
StereoModuleVerifyResultView::StereoModuleVerifyResultView(QWidget *parent)
    : QWidget(parent), d_ptr(new StereoModuleVerifyResultViewPrivate(this))
{
    d_ptr->init();
}

StereoModuleVerifyResultView::~StereoModuleVerifyResultView() = default;

void StereoModuleVerifyResultView::setReference(
    const StereoCameraVerification::Reference &ref)
{
    Q_D(StereoModuleVerifyResultView);
    d->m_ref = ref;
}

void StereoModuleVerifyResultView::showSummary(
    const StereoCameraVerification::Summary &summary)
{
    Q_D(StereoModuleVerifyResultView);

    if (auto oldModel = d->m_view->model(); oldModel) {
        oldModel->deleteLater();
        clearReport();
    }

    // Make model parentless on purpose
    auto *model = new VerifyResultReportModel;
    model->show(d->m_ref, summary);
    d->m_view->setModel(model);

    d->setPlaceholderVisible(false);
}

void StereoModuleVerifyResultView::clearReport()
{
    Q_D(StereoModuleVerifyResultView);

    d->setPlaceholderVisible(true);
}

QString StereoModuleVerifyResultView::verifyItemName(VerifyItem item)
{
    static const std::unordered_map<VerifyItem, QString> map{
        {VerifyItem::TrackerRejectRate, tr("Tracker Reject Rate")},
        {VerifyItem::EstimatorRejectRate, tr("Estimator Reject Rate")},
        {VerifyItem::PoseOptimizationCost, tr("Stereo Optimization Cost")},
        {VerifyItem::RejectRelativeSizeMedian,
         tr("Reject Relative Size Median")},
        {VerifyItem::RejectRelativePositionMedian,
         tr("Reject Relative Pose Median")}};
    return map.at(item);
}

QString StereoModuleVerifyResultView::verifyItemValue(
    const StereoCameraVerification::Summary &summary, VerifyItem item)
{
    constexpr int kPrecision = 2;
    switch (item) {
        case VerifyItem::TrackerRejectRate:
            return qstr::percentage(summary.trackerRejectRate, kPrecision);
        case VerifyItem::EstimatorRejectRate:
            return qstr::percentage(summary.estimatorRejectRate, kPrecision);
        case VerifyItem::PoseOptimizationCost:
            return QString::number(summary.stereoOptimzationCost);
        case VerifyItem::RejectRelativeSizeMedian:
            return qstr::permille(summary.rejectRelativeSizeMedian, kPrecision);
        case VerifyItem::RejectRelativePositionMedian:
            return qstr::permille(summary.rejectRelativePostitionMedian,
                                  kPrecision);
        default:
            break;
    }

    return {};
}

QString StereoModuleVerifyResultView::verifyItemReference(
    const StereoCameraVerification::Reference &ref, VerifyItem item)
{
    constexpr int kPrecision = 2;
    switch (item) {
        case VerifyItem::TrackerRejectRate:
            return qstr::lessThanPercentage(ref.maxTrackerRejectRate,
                                            kPrecision);
        case VerifyItem::EstimatorRejectRate:
            return qstr::lessThanPercentage(ref.maxEstimatorRejectRate,
                                            kPrecision);
        case VerifyItem::PoseOptimizationCost:
            return qstr::lessThan(ref.maxStereoOptimizationCost);
        case VerifyItem::RejectRelativeSizeMedian:
            return qstr::lessThanPermille(ref.maxRejectRelativeSizeMedian,
                                          kPrecision);
        case VerifyItem::RejectRelativePositionMedian:
            return qstr::lessThanPermille(ref.maxRejectRelativePositionMedian,
                                          kPrecision);
        default:
            break;
    }

    return {};
}

QString StereoModuleVerifyResultView::verifyStatusName(VerifyStatus status)
{
    static const std::unordered_map<VerifyStatus, QString> map{
        {VerifyStatus::Failed, tr("Failed")},
        {VerifyStatus::Passed, tr("Passed")},
        {VerifyStatus::NA, tr("NA")}};
    return map.at(status);
}

StereoModuleVerifyResultView::VerifyStatus
StereoModuleVerifyResultView::verifyItemStatus(
    const StereoCameraVerification::Summary &summary, VerifyItem item)
{
    using Error = StereoCameraVerification::Summary::Error;

    auto failedToStatus = [](bool failed) -> VerifyStatus {
        return failed ? VerifyStatus::Failed : VerifyStatus::Passed;
    };

    const auto &error = summary.error;
    switch (item) {
        case VerifyItem::TrackerRejectRate:
            return failedToStatus(error & Error::RejectByTracker);
        case VerifyItem::EstimatorRejectRate:
            return failedToStatus(error & Error::RejectByEstimator);
        case VerifyItem::PoseOptimizationCost:
            return VerifyStatus::NA;
        case VerifyItem::RejectRelativeSizeMedian:
            return failedToStatus(error & Error::LargeRelativeSize);
        case VerifyItem::RejectRelativePositionMedian:
            return failedToStatus(error & Error::LargeRelativePosition);
        default:
            break;
    }

    return VerifyStatus::NA;
}

QString StereoModuleVerifyResultView::verifySummaryToReport(
    const StereoCameraVerification::Reference &ref,
    const StereoCameraVerification::Summary &summary)
{
    return VerifyResultReportModel::toTextTable(ref, summary);
}

} // namespace thoht

#include "StereoModuleVerifyResultView.moc"
#include "moc_StereoModuleVerifyResultView.cpp"

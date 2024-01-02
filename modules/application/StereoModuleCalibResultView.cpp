#include "StereoModuleCalibResultView.h"

#include <array>

#include <QAbstractTableModel>
#include <QBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QStackedWidget>
#include <QSplitter>
#include <QTableView>
#include <QTabWidget>

#include "gui/qstringutils.h"

namespace thoht {

namespace qstr {

inline auto inRange2D(double x, double y, double x_tol, double y_tol) -> QString
{
    return u"(%1, %2) %3 (%4, %5)"_s.arg(
        QString::number(x, 'f', 2), QString::number(y, 'f', 2), QChar{0x00B1},
        QString::number(x_tol), QString::number(y_tol));
};

inline auto inRange3D(double x, double y, double z, double x_tol, double y_tol,
                      double z_tol) -> QString
{
    return u"(%1, %2, %3) %4 (%5, %6, %7)"_s.arg(
        QString::number(x, 'f', 4), QString::number(y, 'f', 4),
        QString::number(z, 'f', 4), QChar{0x00B1}, QString::number(x_tol),
        QString::number(y_tol), QString::number(z_tol));
}

inline auto point2(double x, double y) -> QString
{
    return u"(%1, %2)"_s.arg(QString::number(x, 'f', 2),
                             QString::number(y, 'f', 2));
};

inline auto point3(double x, double y, double z) -> QString
{
    return u"(%1, %2, %3)"_s.arg(QString::number(x, 'f', 4),
                                 QString::number(y, 'f', 4),
                                 QString::number(z, 'f', 4));
}

} // namespace qstr

///------- CalibResultReportModel starts from here
class CalibResultReportModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum class Section
    {
        Item = 0,
        Status,
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

    void show(const StereoModuleTask::ResultReference &ref,
              const StereoModuleTask::ResultSummary &summary);

    inline void resetModel()
    {
        beginResetModel();
        endResetModel();
    }

    static QString toTextTable(const StereoModuleTask::ResultReference &ref,
                               const StereoModuleTask::ResultSummary &summary);

private:
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
    using Item = StereoModuleCalibResultView::VerifyItem;
    using Status = StereoModuleCalibResultView::VerifyStatus;

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

int CalibResultReportModel::rowCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? m_items.size() : 0;
}

int CalibResultReportModel::columnCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? static_cast<int>(Section::Count) : 0;
}

Qt::ItemFlags CalibResultReportModel::flags(const QModelIndex &index) const
{
    if (!index.isValid()) {
        return Qt::ItemIsEnabled;
    }

    return QAbstractTableModel::flags(index);
}

QVariant CalibResultReportModel::headerData(int section,
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

QVariant CalibResultReportModel::data(const QModelIndex &index, int role) const
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
                    return StereoModuleCalibResultView::verifyStatusName(
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

void CalibResultReportModel::show(
    const StereoModuleTask::ResultReference &ref,
    const StereoModuleTask::ResultSummary &summary)
{
    const auto &error = summary.error;
    if (error == StereoModuleTask::ResultSummary::NotReady) {
        // TODO: Do sth here
        return;
    }

    auto updateItem = [this, &ref, &summary](Item itemType) {
        auto &item = m_items[itemType];
        item.name = StereoModuleCalibResultView::verifyItemName(itemType);
        item.status =
            StereoModuleCalibResultView::verifyItemStatus(summary, itemType);
        item.valueString =
            StereoModuleCalibResultView::verifyItemValue(summary, itemType);
        item.refString =
            StereoModuleCalibResultView::verifyItemReference(ref, itemType);
    };

    updateItem(Item::LeftRPE);
    updateItem(Item::RightRPE);
    updateItem(Item::LeftFocalLength);
    updateItem(Item::RightFocalLength);
    updateItem(Item::LeftPrincipalPoint);
    updateItem(Item::RightPrincipalPoint);
    updateItem(Item::PrincipalPointConsistency);
    updateItem(Item::Baseline);
    updateItem(Item::InterCameraRotationInAngleAxis);
    updateItem(Item::InterCameraRotationInEuler);
    updateItem(Item::ImuCameraRotatation);

    resetModel();
}

QString CalibResultReportModel::toTextTable(
    const StereoModuleTask::ResultReference &ref,
    const StereoModuleTask::ResultSummary &summary)
{
    QString report;
    QTextStream out{&report};
    out << tr("Calibration result values verification report:") << "\n";

    const auto &error = summary.error;
    if (error == StereoModuleTask::ResultSummary::NotReady) {
        out << tr("Calibration is not ready for values verification.");
        return report;
    }

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
        out << StereoModuleCalibResultView::verifyItemName(item)
            << StereoModuleCalibResultView::verifyItemValue(summary, item)
            << StereoModuleCalibResultView::verifyItemReference(ref, item)
            << StereoModuleCalibResultView::verifyItemStatusName(summary, item);
    };

    appendItem(out, Item::LeftRPE);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::RightRPE);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::LeftFocalLength);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::RightFocalLength);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::LeftPrincipalPoint);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::RightPrincipalPoint);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::PrincipalPointConsistency);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::Baseline);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::InterCameraRotationInAngleAxis);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::InterCameraRotationInEuler);
    qstr::noPaddingNewline(out, kReportFieldWidth);
    appendItem(out, Item::ImuCameraRotatation);
    out << Qt::endl;

    return report;
}

///------- StereoModuleCalibResultViewPrivate starts from here
class StereoModuleCalibResultViewPrivate
{
    Q_DEFINE_PIMPL(StereoModuleCalibResultView)

public:
    explicit StereoModuleCalibResultViewPrivate(StereoModuleCalibResultView *q);

    void init();

    void setPlaceholderVisible(bool visible);

public:
    StereoModuleTask::ResultReference m_ref;
    QStackedWidget *m_stack;
    QTabWidget *m_tabs;
    QList<QWidget *> m_pages;
    int m_placeholderIndex{-1}, m_tabsIndex{-1};
};

StereoModuleCalibResultViewPrivate::StereoModuleCalibResultViewPrivate(
    StereoModuleCalibResultView *q)
    : q_ptr(q), m_stack(new QStackedWidget(q)), m_tabs(new QTabWidget(q))
{
}

void StereoModuleCalibResultViewPrivate::init()
{
    Q_Q(StereoModuleCalibResultView);

    auto placeholder = new QLabel(
        StereoModuleCalibResultView::tr("Waiting for calibration results."), q);
    placeholder->setProperty("placeholder", true);
    placeholder->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

    m_placeholderIndex = m_stack->addWidget(placeholder);
    m_tabsIndex = m_stack->addWidget(m_tabs);

    auto *layout = new QVBoxLayout(q);
    layout->setContentsMargins({6, 6, 6, 6});
    layout->setSpacing(3);
    layout->addWidget(m_stack);

    q->setStyleSheet(u"QLabel[placeholder=true] { "
                     "font: bold italic 13px;"
                     "background-color: #d4d4d4; }"_s);

    setPlaceholderVisible(true);
}

void StereoModuleCalibResultViewPrivate::setPlaceholderVisible(bool visible)
{
    m_stack->setCurrentIndex(visible ? m_placeholderIndex : m_tabsIndex);
}

///------- StereoModuleCalibResultView starts from here
StereoModuleCalibResultView::StereoModuleCalibResultView(QWidget *parent)
    : QWidget(parent), d_ptr(new StereoModuleCalibResultViewPrivate(this))
{
    d_ptr->init();
}

StereoModuleCalibResultView::~StereoModuleCalibResultView() = default;

void StereoModuleCalibResultView::setReference(
    const StereoModuleTask::ResultReference &ref)
{
    Q_D(StereoModuleCalibResultView);
    d->m_ref = ref;
}

void StereoModuleCalibResultView::append(
    const StereoModuleTask::ResultSummary &summary)
{
    // TODO
}

void StereoModuleCalibResultView::showSummaries(
    const std::vector<StereoModuleTask::ResultSummary> &summaries)
{
    Q_D(StereoModuleCalibResultView);
    if (summaries.empty()) {
        return;
    }

    if (d->m_tabs->count() != 0) {
        clearReport();
    }

    for (size_t i{0}; i < summaries.size(); ++i) {
        // QTabWidget will take ownership
        auto *page = new QTableView;
        // TODO: Take care of the model life time
        auto *model = new CalibResultReportModel(page);
        model->show(d->m_ref, summaries[i]);
        page->setModel(model);
        page->setTextElideMode(Qt::ElideNone);
        page->horizontalHeader()->setSectionResizeMode(
            QHeaderView::ResizeToContents);

        [[maybe_unused]] const auto index =
            d->m_tabs->addTab(page, tr("Task %1").arg(QString::number(i + 1)));
        d->m_pages.push_back(page);
    }

    d->m_tabs->setCurrentIndex(0);
}

void StereoModuleCalibResultView::clearReport()
{
    Q_D(StereoModuleCalibResultView);
    // Clear result pages
    d->m_tabs->clear();
    qDeleteAll(d->m_pages);
    d->m_pages.clear();

    showPlaceholder();
}

void StereoModuleCalibResultView::showPlaceholder(bool show)
{
    Q_D(StereoModuleCalibResultView);
    d->setPlaceholderVisible(show);
}

QString StereoModuleCalibResultView::verifyItemName(VerifyItem item)
{
    static const std::unordered_map<VerifyItem, QString> map{
        {VerifyItem::LeftRPE, tr("Left RPE")},
        {VerifyItem::RightRPE, tr("Right RPE")},
        {VerifyItem::LeftFocalLength, tr("Left focal length")},
        {VerifyItem::RightFocalLength, tr("Right focal length")},
        {VerifyItem::LeftPrincipalPoint, tr("Left principal point")},
        {VerifyItem::RightPrincipalPoint, tr("Right principal point")},
        {VerifyItem::PrincipalPointConsistency,
         tr("Principal point consistency")},
        {VerifyItem::Baseline, tr("Baseline")},
        {VerifyItem::InterCameraRotationInAngleAxis,
         tr("Inter-Camera Rotation")},
        {VerifyItem::InterCameraRotationInEuler,
         tr("Inter-Camera Rotation (RPY)")},
        {VerifyItem::ImuCameraRotatation, tr("Imu-Camera Rotation")}};
    return map.at(item);
}

QString StereoModuleCalibResultView::verifyItemValue(
    const StereoModuleTask::ResultSummary &summary, VerifyItem item)
{
    switch (item) {
        case VerifyItem::LeftRPE:
            return QString::number(summary.leftRPE);
        case VerifyItem::RightRPE:
            return QString::number(summary.rightRPE);
        case VerifyItem::LeftFocalLength:
            return QString::number(summary.leftFocalLength);
        case VerifyItem::RightFocalLength:
            return QString::number(summary.rightFocalLength);
        case VerifyItem::LeftPrincipalPoint:
            return qstr::point2(summary.leftCx, summary.leftCy);
        case VerifyItem::RightPrincipalPoint:
            return qstr::point2(summary.rightCx, summary.rightCy);
        case VerifyItem::PrincipalPointConsistency:
            return qstr::point2(summary.diffCx, summary.diffCy);
        case VerifyItem::Baseline:
            return QString::number(summary.baseline);
        case VerifyItem::InterCameraRotationInAngleAxis:
            return QString::number(summary.interCameraRotation);
        case VerifyItem::InterCameraRotationInEuler:
            return qstr::point3(summary.interCameraRoatationRPY[0],
                                summary.interCameraRoatationRPY[1],
                                summary.interCameraRoatationRPY[2]);
        case VerifyItem::ImuCameraRotatation:
            // Format left and right rotation as point: (left, right)
            return qstr::point2(summary.imuLeftCameraRotation,
                                summary.imuRightCameraRotation);
        default:
            break;
    }

    return {};
}

QString StereoModuleCalibResultView::verifyItemReference(
    const StereoModuleTask::ResultReference &ref, VerifyItem item)
{
    switch (item) {
        case VerifyItem::LeftRPE:
            return qstr::lessEqualThan(ref.maxRPE);
        case VerifyItem::RightRPE:
            return qstr::lessEqualThan(ref.maxRPE);
        case VerifyItem::LeftFocalLength:
        case VerifyItem::RightFocalLength:
            return qstr::inRange(ref.expectedFocalLength,
                                 ref.focalLengthTolerance);
        case VerifyItem::LeftPrincipalPoint:
        case VerifyItem::RightPrincipalPoint:
            return qstr::inRange2D(
                ref.expectedPrincipalPointX, ref.expectedPrincipalPointY,
                ref.principalPointXTolerance, ref.principalPointYTolerance);
        case VerifyItem::PrincipalPointConsistency:
            return u"abs(y) <= %1"_s.arg(
                QString::number(ref.principalPointDiffTolerance));
        case VerifyItem::Baseline:
            return qstr::inRange(ref.expectedBaseline, ref.baselineTolerance);
        case VerifyItem::InterCameraRotationInAngleAxis:
            return qstr::inRange(ref.expectedInterCameraRotation,
                                 ref.interCameraRotationTolerance);
        case VerifyItem::InterCameraRotationInEuler:
            // TODO: Dont use hardcode
            return qstr::inRange3D(0., 0., 0., 0.1, 0.1, 0.1);
        case VerifyItem::ImuCameraRotatation:
            return qstr::inRange(ref.expectedImuCameraRotation,
                                 ref.imuCameraRotationTolerance);
        default:
            break;
    }

    return {};
}

QString StereoModuleCalibResultView::verifyStatusName(VerifyStatus status)
{
    static const std::unordered_map<VerifyStatus, QString> map{
        {VerifyStatus::Failed, tr("Failed")},
        {VerifyStatus::Passed, tr("Passed")},
        {VerifyStatus::NA, tr("NA")}};
    return map.at(status);
}

StereoModuleCalibResultView::VerifyStatus
StereoModuleCalibResultView::verifyItemStatus(
    const StereoModuleTask::ResultSummary &summary, VerifyItem item)
{
    using Error = StereoModuleTask::ResultSummary::Error;

    auto errorToStatus = [](Error error, Error item, Error naError) {
        if (naError & item) {
            return VerifyStatus::NA;
        }

        return (error & item) ? VerifyStatus::Failed : VerifyStatus::Passed;
    };

    const auto naError = StereoModuleTask::notAppliedErrors();
    const auto &error = summary.error;
    switch (item) {
        case VerifyItem::LeftRPE:
            return errorToStatus(error, Error::PoorLeftRPE, naError);
        case VerifyItem::RightRPE:
            return errorToStatus(error, Error::PoorRightRPE, naError);
        case VerifyItem::LeftFocalLength:
            return errorToStatus(error, Error::PoorLeftFocalLength, naError);
        case VerifyItem::RightFocalLength:
            return errorToStatus(error, Error::PoorRightFocalLength, naError);
        case VerifyItem::LeftPrincipalPoint:
            return errorToStatus(error, Error::PoorLeftPrincipalPoint, naError);
        case VerifyItem::RightPrincipalPoint:
            return errorToStatus(error, Error::PoorRightPrincipalPoint,
                                 naError);
        case VerifyItem::PrincipalPointConsistency:
            return errorToStatus(error, Error::PoorPrincipalPointConsistency,
                                 naError);
        case VerifyItem::Baseline:
            return errorToStatus(error, Error::PoorBaseline, naError);
        case VerifyItem::InterCameraRotationInAngleAxis:
            return errorToStatus(error, Error::PoorInterCameraRotation,
                                 naError);
        case VerifyItem::InterCameraRotationInEuler:
            return errorToStatus(error, Error::PoorInterCameraRotationInEuler,
                                 naError);
        case VerifyItem::ImuCameraRotatation:
            return errorToStatus(error, Error::PoorImuCameraRotation, naError);
        default:
            break;
    }

    return VerifyStatus::NA;
}

QString StereoModuleCalibResultView::verifySummaryToReport(
    const StereoModuleTask::ResultReference &ref,
    const StereoModuleTask::ResultSummary &summary)
{
    return CalibResultReportModel::toTextTable(ref, summary);
}

} // namespace thoht

#include "StereoModuleCalibResultView.moc"
#include "moc_StereoModuleCalibResultView.cpp"

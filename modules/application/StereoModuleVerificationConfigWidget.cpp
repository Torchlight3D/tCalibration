#include "StereoModuleVerificationConfigWidget.h"
#include "ui_StereoModuleVerificationConfigWidget.h"

#include <QBoxLayout>
#include <QDialogButtonBox>

#include <AxMath/MathBase>

namespace thoht {

///------- StereoModuleVerificationConfigWidgetPrivate starts from here
class StereoModuleVerificationConfigWidgetPrivate
{
    Q_DEFINE_PIMPL(StereoModuleVerificationConfigWidget)

public:
    explicit StereoModuleVerificationConfigWidgetPrivate(
        StereoModuleVerificationConfigWidget* q);
    ~StereoModuleVerificationConfigWidgetPrivate();

    void init();

public:
    Ui::StereoModuleVerificationConfigWidget* ui;
};

StereoModuleVerificationConfigWidgetPrivate::
    StereoModuleVerificationConfigWidgetPrivate(
        StereoModuleVerificationConfigWidget* q)
    : q_ptr(q), ui(new Ui::StereoModuleVerificationConfigWidget)
{
}

StereoModuleVerificationConfigWidgetPrivate::
    ~StereoModuleVerificationConfigWidgetPrivate()
{
    delete ui;
}

void StereoModuleVerificationConfigWidgetPrivate::init()
{
    Q_Q(StereoModuleVerificationConfigWidget);
    ui->setupUi(q);

    ui->maxStereoOptCost->setEnabled(false);
}

///------- StereoModuleVerificationConfigWidget starts from here
StereoModuleVerificationConfigWidget::StereoModuleVerificationConfigWidget(
    QWidget* parent)
    : QWidget(parent),
      d_ptr(new StereoModuleVerificationConfigWidgetPrivate(this))
{
    Q_D(StereoModuleVerificationConfigWidget);
    d->init();

    connect(d->ui->useMaxStereoOptCost, &QAbstractButton::toggled, this,
            [d](bool on) { d->ui->maxStereoOptCost->setEnabled(on); });
}

StereoModuleVerificationConfigWidget::~StereoModuleVerificationConfigWidget() =
    default;

void StereoModuleVerificationConfigWidget::setDefaultValues(
    const StereoCameraVerification::Options& opts)
{
    Q_D(StereoModuleVerificationConfigWidget);
    d->ui->patternSizeRow->setValue(opts.chessPatternSize.height);
    d->ui->patternSizeCol->setValue(opts.chessPatternSize.width);
    d->ui->checkerSizeWidth->setValue(math::m2cm(opts.chessSize.width));
    d->ui->checkerSizeHeight->setValue(math::m2cm(opts.chessSize.height));

    const auto& ref = opts.reference;
    d->ui->maxTrackerRR->setValue(math::toPercent(ref.maxTrackerRejectRate));
    d->ui->maxEstimatorRR->setValue(
        math::toPercent(ref.maxEstimatorRejectRate));
    d->ui->maxRejectSizeMedian->setValue(
        math::toPercent(ref.maxRejectRelativeSizeMedian));
    d->ui->maxRejectPoseMedian->setValue(
        math::toPercent(ref.maxRejectRelativePositionMedian));
    d->ui->maxStereoOptCost->setValue(ref.maxStereoOptimizationCost);
}

StereoCameraVerification::Options
StereoModuleVerificationConfigWidget::options() const
{
    Q_D(const StereoModuleVerificationConfigWidget);
    StereoCameraVerification::Options opts;
    opts.chessPatternSize = {d->ui->patternSizeCol->value(),
                             d->ui->patternSizeRow->value()};
    opts.chessSize = {math::cm2m(d->ui->checkerSizeWidth->value()),
                      math::cm2m(d->ui->checkerSizeHeight->value())};

    auto& ref = opts.reference;
    ref.maxTrackerRejectRate = math::fromPercent(d->ui->maxTrackerRR->value());
    ref.maxEstimatorRejectRate =
        math::fromPercent(d->ui->maxEstimatorRR->value());
    ref.maxStereoOptimizationCost = d->ui->maxStereoOptCost->value();
    ref.maxRejectRelativeSizeMedian =
        math::fromPercent(d->ui->maxRejectSizeMedian->value());
    ref.maxRejectRelativePositionMedian =
        math::fromPercent(d->ui->maxRejectPoseMedian->value());

    return opts;
}

///------- StereoModuleVerificationConfigDialog starts from here
StereoModuleVerificationConfigDialog::StereoModuleVerificationConfigDialog(
    QWidget* parent)
    : QDialog(parent), w(new StereoModuleVerificationConfigWidget(this))
{
    setWindowTitle(w->windowTitle());

    using StandardButton = QDialogButtonBox::StandardButton;
    auto btns = new QDialogButtonBox(this);
    btns->setStandardButtons(StandardButton::Apply | StandardButton::Cancel);
    btns->setCenterButtons(true);
    connect(
        btns, &QDialogButtonBox::clicked, this, [this](QAbstractButton* btn) {
            switch (qobject_cast<QDialogButtonBox*>(sender())->standardButton(
                btn)) {
                case StandardButton::Apply:
                    accept();
                    break;
                case StandardButton::Cancel:
                    reject();
                    break;
                default:
                    break;
            }
        });

    auto layout = new QVBoxLayout(this);
    layout->setContentsMargins({0, 0, 0, 6});
    layout->setSpacing(0);
    layout->addWidget(w);
    layout->addWidget(btns);
}

StereoModuleVerificationConfigWidget* const
StereoModuleVerificationConfigDialog::widget() const
{
    return w;
}

} // namespace thoht

#include "moc_StereoModuleVerificationConfigWidget.cpp"

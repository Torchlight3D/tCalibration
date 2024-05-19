#include "StereoModuleCalibrationConfigWidget.h"
#include "ui_StereoModuleCalibrationConfigWidget.h"

#include <QDialogButtonBox>

#include <tCore/Math>

namespace tl {

///------- StereoModuleCalibrationConfigWidgetPrivate starts from here
class StereoModuleCalibrationConfigWidgetPrivate
{
    Q_DEFINE_PIMPL(StereoModuleCalibrationConfigWidget)

public:
    explicit StereoModuleCalibrationConfigWidgetPrivate(
        StereoModuleCalibrationConfigWidget* q);
    ~StereoModuleCalibrationConfigWidgetPrivate();

    void init();

public:
    Ui::StereoModuleCalibrationConfigWidget* ui;
};

StereoModuleCalibrationConfigWidgetPrivate::
    StereoModuleCalibrationConfigWidgetPrivate(
        StereoModuleCalibrationConfigWidget* q)
    : q_ptr(q), ui(new Ui::StereoModuleCalibrationConfigWidget)
{
}

StereoModuleCalibrationConfigWidgetPrivate::
    ~StereoModuleCalibrationConfigWidgetPrivate()
{
    delete ui;
}

void StereoModuleCalibrationConfigWidgetPrivate::init()
{
    Q_Q(StereoModuleCalibrationConfigWidget);
    ui->setupUi(q);
}

///------- StereoModuleCalibrationConfigWidget starts from here
StereoModuleCalibrationConfigWidget::StereoModuleCalibrationConfigWidget(
    QWidget* parent)
    : QWidget(parent),
      d_ptr(new StereoModuleCalibrationConfigWidgetPrivate(this))
{
    d_ptr->init();
}

StereoModuleCalibrationConfigWidget::~StereoModuleCalibrationConfigWidget() =
    default;

void StereoModuleCalibrationConfigWidget::setDefaultValues(
    const StereoModuleTask::ResultReference& ref)
{
    Q_D(StereoModuleCalibrationConfigWidget);
    d->ui->maxRPE->setValue(ref.maxRPE);
    d->ui->expectedFocalLength->setValue(ref.expectedFocalLength);
    d->ui->focalLengthTolerence->setValue(ref.focalLengthTolerance);
    d->ui->expectedPrincipalPointX->setValue(ref.expectedPrincipalPointX);
    d->ui->expectedPrincipalPointY->setValue(ref.expectedPrincipalPointY);
    d->ui->principalPointXTolerance->setValue(ref.principalPointXTolerance);
    d->ui->principalPointYTolerance->setValue(ref.principalPointYTolerance);
    d->ui->principalPointDiffY->setValue(ref.principalPointDiffTolerance);
    d->ui->expectedBaseline->setValue(ref.expectedBaseline);
    d->ui->baselineTolerance->setValue(ref.baselineTolerance);
    d->ui->expectedInterCameraRotation->setValue(
        math::radToDeg(ref.expectedInterCameraRotation));
    d->ui->interCameraRotationTolerance->setValue(
        math::radToDeg(ref.interCameraRotationTolerance));
    d->ui->expectedImuCameraRotation->setValue(
        math::radToDeg(ref.expectedImuCameraRotation));
    d->ui->imuCameraRotationTolerance->setValue(
        math::radToDeg(ref.imuCameraRotationTolerance));
}

StereoModuleTask::Options StereoModuleCalibrationConfigWidget::options() const
{
    StereoModuleTask::Options opts;
    // TODO
    return opts;
}

StereoModuleTask::ResultReference
StereoModuleCalibrationConfigWidget::resultReference() const
{
    Q_D(const StereoModuleCalibrationConfigWidget);
    StereoModuleTask::ResultReference ref;
    ref.maxRPE = d->ui->maxRPE->value();
    ref.expectedFocalLength = d->ui->expectedFocalLength->value();
    ref.focalLengthTolerance = d->ui->focalLengthTolerence->value();
    ref.expectedPrincipalPointX = d->ui->expectedPrincipalPointX->value();
    ref.expectedPrincipalPointY = d->ui->expectedPrincipalPointY->value();
    ref.principalPointXTolerance = d->ui->principalPointXTolerance->value();
    ref.principalPointYTolerance = d->ui->principalPointYTolerance->value();
    ref.principalPointDiffTolerance = d->ui->principalPointDiffY->value();
    ref.expectedBaseline = d->ui->expectedBaseline->value();
    ref.baselineTolerance = d->ui->baselineTolerance->value();
    ref.expectedInterCameraRotation =
        math::degToRad(d->ui->expectedInterCameraRotation->value());
    ref.interCameraRotationTolerance =
        math::degToRad(d->ui->interCameraRotationTolerance->value());
    ref.expectedImuCameraRotation =
        math::degToRad(d->ui->expectedImuCameraRotation->value());
    ref.imuCameraRotationTolerance =
        math::degToRad(d->ui->imuCameraRotationTolerance->value());

    return ref;
}

StereoModuleCalibrationConfigDialog::StereoModuleCalibrationConfigDialog(
    QWidget* parent)
    : QDialog(parent), w(new StereoModuleCalibrationConfigWidget(this))
{
    setWindowTitle(w->windowTitle());

    using StandardButton = QDialogButtonBox::StandardButton;
    auto btns = new QDialogButtonBox(this);
    btns->setStandardButtons(StandardButton::Apply | StandardButton::Cancel);
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
    layout->setContentsMargins({6, 0, 6, 6});
    layout->setSpacing(0);
    layout->addWidget(w);
    layout->addWidget(btns);
}

StereoModuleCalibrationConfigWidget* const
StereoModuleCalibrationConfigDialog::widget() const
{
    return w;
}

} // namespace tl

#include "moc_StereoModuleCalibrationConfigWidget.cpp"

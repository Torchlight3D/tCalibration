#include "StereoModuleImageQualityView.h"

namespace thoht {

///------- StereoModuleImageQualityViewPrivate starts from here
class StereoModuleImageQualityViewPrivate
{
    AX_Q_DEFINE_PIMPL(StereoModuleImageQualityView)

public:
    explicit StereoModuleImageQualityViewPrivate(
        StereoModuleImageQualityView *q);
    ~StereoModuleImageQualityViewPrivate();

    void init();
    void updateUiByDevMode(bool on);

public:
    bool m_devMode{false};
};

StereoModuleImageQualityViewPrivate::StereoModuleImageQualityViewPrivate(
    StereoModuleImageQualityView *q)
    : q_ptr(q)
{
}

StereoModuleImageQualityViewPrivate::~StereoModuleImageQualityViewPrivate()
{
    // Maybe delete ui later
}

void StereoModuleImageQualityViewPrivate::init()
{
    Q_Q(StereoModuleImageQualityView);
    q->setWindowTitle(
        StereoModuleImageQualityView::tr("Stereo Module Image Quality"));

    updateUiByDevMode(m_devMode);
}

void StereoModuleImageQualityViewPrivate::updateUiByDevMode(bool on)
{
    // TODO
}

StereoModuleImageQualityView::StereoModuleImageQualityView(QWidget *parent)
    : QWidget(parent), d_ptr(new StereoModuleImageQualityViewPrivate(this))
{
    Q_D(StereoModuleImageQualityView);
    d->init();
}

StereoModuleImageQualityView::~StereoModuleImageQualityView() = default;

void StereoModuleImageQualityView::setDevMode(bool on)
{
    // TODO
}

} // namespace thoht

#include "moc_StereoModuleImageQualityView.cpp"

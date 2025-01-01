#include "MonoCameraVerificationView.h"
#include "ui_MonoCameraVerificationView.h"

namespace tl {

Q_LOGGING_CATEGORY(MonoVerification, "mono.verification")

class MonoCameraVerificationViewPrivate
{
    Q_DECLARE_TR_FUNCTIONS(tl::MonoCameraVerificationView)
    Q_DEFINE_PIMPL(MonoCameraVerificationView)

public:
    explicit MonoCameraVerificationViewPrivate(MonoCameraVerificationView* q);
    ~MonoCameraVerificationViewPrivate();

    void init();

public:
    Ui::MonoCameraVerificationView* ui;
};

MonoCameraVerificationViewPrivate::MonoCameraVerificationViewPrivate(
    MonoCameraVerificationView* q)
    : q_ptr(q), ui(new Ui::MonoCameraVerificationView)
{
}

MonoCameraVerificationViewPrivate::~MonoCameraVerificationViewPrivate()
{
    delete ui;
}

void MonoCameraVerificationViewPrivate::init()
{
    Q_Q(MonoCameraVerificationView);
    ui->setupUi(q);

    q->setWindowTitle(viewTitle(tr("Mono Camera Verification")));
}

MonoCameraVerificationView::MonoCameraVerificationView(QWidget* parent)
    : ToolView(parent), d_ptr(new MonoCameraVerificationViewPrivate(this))
{
    Q_D(MonoCameraVerificationView);
    d->init();
}

MonoCameraVerificationView::~MonoCameraVerificationView() = default;

bool MonoCameraVerificationView::setFromJson(const std::string& json)
{
    // ...
    return true;
}

void MonoCameraVerificationView::restoreSettings(QSettings& settings)
{
    // ...
}

void MonoCameraVerificationView::saveSettings(QSettings& settings) const
{
    // ...
}

} // namespace tl

#include "moc_MonoCameraVerificationView.cpp"

#include "CalibBoardConfigsWidget.h"
#include "ui_CalibBoardConfigsWidget.h"

#include <QDialogButtonBox>

namespace thoht {

///------- CalibBoardConfigsWidgetPrivate starts from here
class CalibBoardConfigsWidgetPrivate
{
    Q_DEFINE_PIMPL(CalibBoardConfigsWidget)

public:
    explicit CalibBoardConfigsWidgetPrivate(CalibBoardConfigsWidget* q);
    ~CalibBoardConfigsWidgetPrivate();

    void init();

public:
    Ui::CalibBoardConfigsWidget* ui;
};

CalibBoardConfigsWidgetPrivate::CalibBoardConfigsWidgetPrivate(
    CalibBoardConfigsWidget* q)
    : q_ptr(q), ui(new Ui::CalibBoardConfigsWidget)
{
}

CalibBoardConfigsWidgetPrivate::~CalibBoardConfigsWidgetPrivate() { delete ui; }

void CalibBoardConfigsWidgetPrivate::init()
{
    Q_Q(CalibBoardConfigsWidget);
    ui->setupUi(q);
}

///------- CalibBoardConfigsWidget starts from here
CalibBoardConfigsWidget::CalibBoardConfigsWidget(QWidget* parent)
    : QWidget(parent), d_ptr(new CalibBoardConfigsWidgetPrivate(this))
{
    d_ptr->init();
}

CalibBoardConfigsWidget::~CalibBoardConfigsWidget() = default;

///------- CalibBoardConfigsDialog starts from here
CalibBoardConfigsDialog::CalibBoardConfigsDialog(QWidget* parent)
    : QDialog(parent), w(new CalibBoardConfigsWidget(this))
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
    layout->setContentsMargins({0, 0, 0, 6});
    layout->setSpacing(0);
    layout->addWidget(w);
    layout->addWidget(btns);
}

CalibBoardConfigsWidget* const CalibBoardConfigsDialog::widget() const
{
    return w;
}

} // namespace thoht

#include "moc_CalibBoardConfigsWidget.cpp"

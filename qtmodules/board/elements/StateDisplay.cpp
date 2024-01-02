#include "StateDisplay.h"
#include "editor/ui_StateDisplayEditor.h"

#include <QDialog>

#include "../Board.h"
#include "../BoardElement.h"
#include "../utils/AdaptiveTextElement.h"
#include "../data/DataManager.h"
#include "../widgets/ElementPropertiesWidget.h"

namespace thoht {

///------- StateDisplay Editor starts from here
class StateDisplayEditor : public QDialog
{
    Q_OBJECT

public:
    explicit StateDisplayEditor(StateDisplay *display,
                                QWidget *parent = nullptr);
    ~StateDisplayEditor() override;

    void accept() override;

private slots:
    void updateElement();
    void connectProperties(bool connected);
    void newParameter();
    void removeParameter();
    void replaceParameter();

private:
    Ui::StateDisplayEditor *ui;
    StateDisplay *mStateIndicator;
    ElementPropertiesWidget *mPropertiesWidget;
    QToolButton *mNewParamButton;
    QToolButton *mRemoveParamButton;
    QToolButton *mReplaceParamButton;
};

StateDisplayEditor::StateDisplayEditor(StateDisplay *display, QWidget *parent)
    : QDialog(parent), ui(new Ui::StateDisplayEditor), mStateIndicator(display)
{
    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(0);
    ui->listWidget->setCurrentRow(0);
    connect(ui->buttonBox->button(QDialogButtonBox::Apply),
            &QPushButton::clicked, this, &StateDisplayEditor::updateElement);

    QWidget *cornerWidget = new QWidget(this);
    mNewParamButton = new QToolButton(cornerWidget);
    mNewParamButton->setCursor(Qt::ArrowCursor);
    mNewParamButton->setAutoRaise(true);
    mNewParamButton->setIconSize(QSize(20, 20));
    mNewParamButton->setMinimumSize(QSize(30, 30));
    mNewParamButton->setIcon({}); // FIXME
    QObject::connect(mNewParamButton, SIGNAL(clicked()), this,
                     SLOT(newParameter()));
    mNewParamButton->setToolTip(tr("Add parameter"));

    mRemoveParamButton = new QToolButton(cornerWidget);
    mRemoveParamButton->setCursor(Qt::ArrowCursor);
    mRemoveParamButton->setAutoRaise(true);
    mRemoveParamButton->setIconSize(QSize(20, 20));
    mRemoveParamButton->setMinimumSize(QSize(30, 30));
    mRemoveParamButton->setIcon({}); // FIXME
    QObject::connect(mRemoveParamButton, SIGNAL(clicked()), this,
                     SLOT(removeParameter()));
    mRemoveParamButton->setToolTip(tr("Remove parameter"));
    mRemoveParamButton->setVisible(false);

    mReplaceParamButton = new QToolButton(cornerWidget);
    mReplaceParamButton->setCursor(Qt::ArrowCursor);
    mReplaceParamButton->setAutoRaise(true);
    mReplaceParamButton->setIconSize(QSize(20, 20));
    mReplaceParamButton->setMinimumSize(QSize(30, 30));
    mReplaceParamButton->setIcon({}); // FIXME
    QObject::connect(mReplaceParamButton, SIGNAL(clicked()), this,
                     SLOT(replaceParameter()));
    mReplaceParamButton->setToolTip(tr("Replace parameter"));
    mReplaceParamButton->setVisible(false);

    auto *lay = new QHBoxLayout(cornerWidget);
    lay->setContentsMargins({});
    lay->addWidget(mReplaceParamButton);
    lay->addWidget(mRemoveParamButton);
    lay->addWidget(mNewParamButton);

    cornerWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    ui->tabWidget->setCornerWidget(cornerWidget, Qt::TopRightCorner);

    mPropertiesWidget = new ElementPropertiesWidget(this);
    mPropertiesWidget->setVisible(false);
    mPropertiesWidget->setProject(display->board()->project());

    connect(mPropertiesWidget, SIGNAL(connectProperties(bool)), this,
            SLOT(connectProperties(bool)));

    connect(ui->tabWidget, &QTabWidget::currentChanged, this, [this]() {
        const bool visible = ui->tabWidget->currentIndex() > 0;
        mRemoveParamButton->setVisible(visible);
        mReplaceParamButton->setVisible(visible);
    });

    if (mStateIndicator) {
        ui->orientationComboBox->setCurrentIndex(
            mStateIndicator->orientation());
        ui->headerCheckBox->setChecked(mStateIndicator->headerVisible());
        ui->backgroundComboBox->setCurrentIndex(
            mStateIndicator->backgroundMode());

        ;
        if (auto dashParam = mStateIndicator->boardParameter(0)) {
            if (dashParam->connected()) {
                mPropertiesWidget->setEditionMode(
                    ElementPropertiesWidget::emElementConnected);
                mPropertiesWidget->updateUi(
                    dashParam->sharedParameterConfiguration());
            }
            else {
                if (dashParam->sharedParameterConfiguration())
                    mPropertiesWidget->setEditionMode(
                        ElementPropertiesWidget::emElementDisconnected);
                else
                    mPropertiesWidget->setEditionMode(
                        ElementPropertiesWidget::emElementStandAlone);
                mPropertiesWidget->updateUi(
                    dashParam->exclusiveParameterConfiguration());
            }
            mPropertiesWidget->setPropertiesMode(
                ParameterConfiguration::cmState);
            mPropertiesWidget->setVisible(true);
            ui->tabWidget->insertTab(1, mPropertiesWidget,
                                     dashParam->getDisplayedLabel());
        }
    }
    setMinimumSize(600, 400);
}

StateDisplayEditor::~StateDisplayEditor() { delete ui; }

void StateDisplayEditor::accept()
{
    updateElement();
    QDialog::accept();
}

void StateDisplayEditor::updateElement()
{
    if (mStateIndicator) {
        mStateIndicator->setOrientation(StateDisplay::DisplayOrientation(
            ui->orientationComboBox->currentIndex()));
        mStateIndicator->setHeaderVisible(ui->headerCheckBox->isChecked());

        mStateIndicator->setBackgroundMode(BoardElement::BackgroundMode(
            ui->backgroundComboBox->currentIndex()));
        QSharedPointer<BoardParameter> dashParam =
            mStateIndicator->boardParameter(0);
        if (dashParam) {
            if (!mPropertiesWidget->isConnected()) {
                mPropertiesWidget->updateParameterSettings(
                    dashParam->exclusiveParameterConfiguration());
                dashParam->disconnectSharedConfiguration();
            }
            else {
                dashParam->setSharedParameterConfiguration(
                    mPropertiesWidget->currentSettings(), true);
            }
        }
        mStateIndicator->rebuildElement(true);
    }
}

void StateDisplayEditor::connectProperties(bool connected)
{
    QSharedPointer<BoardParameter> dashParam =
        mStateIndicator->boardParameter(0);
    if (dashParam) {
        if (connected) {
            mPropertiesWidget->setEditionMode(
                ElementPropertiesWidget::emElementConnected);
            mPropertiesWidget->updateUi(
                dashParam->sharedParameterConfiguration());
        }
        else {
            mPropertiesWidget->setEditionMode(
                ElementPropertiesWidget::emElementDisconnected);
            mPropertiesWidget->updateUi(
                dashParam->exclusiveParameterConfiguration());
        }
        mPropertiesWidget->setPropertiesMode(ParameterConfiguration::cmState);
    }
}

void StateDisplayEditor::newParameter()
{
    QString paramLabel =
        QInputDialog::getText(this, "Parameter Label", "Parameter Label");
    if (!paramLabel.isEmpty()) {
        mStateIndicator->addParameter(paramLabel);

        QSharedPointer<BoardParameter> dashParam =
            mStateIndicator->boardParameter(0);
        if (dashParam) {
            mPropertiesWidget->setEditionMode(
                ElementPropertiesWidget::emElementStandAlone);
            mPropertiesWidget->setPropertiesMode(
                ParameterConfiguration::cmState);
            mPropertiesWidget->updateUi(
                dashParam->exclusiveParameterConfiguration());
            mPropertiesWidget->setVisible(true);
            ui->tabWidget->insertTab(1, mPropertiesWidget,
                                     dashParam->getDisplayedLabel());
        }
        if (!mStateIndicator->board()->dataManager()->liveDataEnabled()) {
            mStateIndicator->board()->resetPlayback();
            mStateIndicator->board()->replot(QCustomPlot::rpQueuedReplot);
        }
    }
}

void StateDisplayEditor::removeParameter()
{
    mStateIndicator->removeBoardParameter(0);
    ui->tabWidget->removeTab(1);
}

void StateDisplayEditor::replaceParameter()
{
    if (ui->tabWidget->currentIndex() > 0) {
        QString paramLabel =
            QInputDialog::getText(this, "Parameter Label", "Parameter Label");
        if (!paramLabel.isEmpty()) {
            QSharedPointer<BoardParameter> dashParam =
                mStateIndicator->replaceParameter(
                    ui->tabWidget->currentIndex() - 1, paramLabel);
            if (dashParam) {
                mPropertiesWidget->setEditionMode(
                    ElementPropertiesWidget::emElementStandAlone);
                mPropertiesWidget->setPropertiesMode(
                    ParameterConfiguration::cmState);
                mPropertiesWidget->updateUi(
                    dashParam->exclusiveParameterConfiguration());
                mPropertiesWidget->setVisible(true);
                ui->tabWidget->insertTab(1, mPropertiesWidget,
                                         dashParam->getDisplayedLabel());
            }
            if (!mStateIndicator->board()->dataManager()->liveDataEnabled()) {
                mStateIndicator->board()->resetPlayback();
                mStateIndicator->board()->replot(QCustomPlot::rpQueuedReplot);
            }
        }
    }
}

///------- StateDisplay starts from here
StateDisplay::StateDisplay(Board *dashboard) : NumericalDisplay(dashboard)
{
    setConfigurationMode(ParameterConfiguration::cmState);
}

void StateDisplay::edit()
{
    StateDisplayEditor editor(this);
    editor.exec();
}

void StateDisplay::initialize(Board *dashboard)
{
    NumericalDisplay::initialize(dashboard);

    mTextBorderPen = QPen(QApplication::palette().midlight().color(), 0,
                          Qt::SolidLine, Qt::SquareCap);
}

void StateDisplay::loadSettings(QSettings *settings)
{
    SingleDisplay::loadSettings(settings);

    settings->beginGroup("SpecDisplay");
    if (settings->contains("Orientation"))
        mOrientation =
            DisplayOrientation(settings->value("Orientation").toInt());
    settings->endGroup();
}

void StateDisplay::saveSettings(QSettings *settings)
{
    SingleDisplay::saveSettings(settings);
    settings->beginGroup("SpecDisplay");
    settings->setValue("Orientation", mOrientation);
    settings->endGroup();
}

void StateDisplay::displayData()
{
    QSharedPointer<BoardParameter> dashParam = boardParameter(0);
    if (dashParam && dashParam->getTimeSeries().parameterId() > 0) {
        dashParam->processStateData();

        auto text = dashParam->getStateString();
        auto brush = dashParam->getBackgroundBrush();

        if (text.isEmpty() && brush == Qt::NoBrush)
            mTextValue->setBorderPen(mTextBorderPen);
        else
            mTextValue->setBorderPen(Qt::NoPen);

        mTextValue->setText(text);
        mTextValue->setTextColor(dashParam->getForegroundColor());
        mTextValue->setBackgroundBrush(brush);
    }
}

} // namespace thoht

#include "StateDisplay.moc"

#include "NumericalDisplay.h"
#include "editor/ui_NumericalDisplayEditor.h"

#include <QDialog>

#include "../Board.h"
#include "../BoardElement.h"
#include "../BoardLayout.h"
#include "../utils/AdaptiveTextElement.h"
#include "../utils/LayoutGrid.h"
#include "../data/DataManager.h"
#include "../widgets/ElementPropertiesWidget.h"

namespace tl {

///------- NumericalDisplayEditor starts from here
class NumericalDisplayEditor : public QDialog
{
    Q_OBJECT

public:
    explicit NumericalDisplayEditor(NumericalDisplay *display,
                                    QWidget *parent = nullptr);
    ~NumericalDisplayEditor() override;

    void accept() override;

private slots:
    void updateElement();
    void connectProperties(bool connected);
    void newParameter();
    void removeParameter();
    void replaceParameter();

private:
    Ui::NumericalDisplayEditor *ui;
    NumericalDisplay *mValueDisplay;
    ElementPropertiesWidget *mPropertiesWidget;
    QToolButton *mNewParamButton;
    QToolButton *mRemoveParamButton;
    QToolButton *mReplaceParamButton;
};

NumericalDisplayEditor::NumericalDisplayEditor(NumericalDisplay *display,
                                               QWidget *parent)
    : QDialog(parent),
      ui(new Ui::NumericalDisplayEditor),
      mValueDisplay(display),
      mPropertiesWidget(nullptr)
{
    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(0);
    ui->listWidget->setCurrentRow(0);
    connect(ui->buttonBox->button(QDialogButtonBox::Apply),
            &QPushButton::clicked, this,
            &NumericalDisplayEditor::updateElement);

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

    QHBoxLayout *lay = new QHBoxLayout(cornerWidget);
    lay->setContentsMargins({});
    lay->addWidget(mReplaceParamButton);
    lay->addWidget(mRemoveParamButton);
    lay->addWidget(mNewParamButton);

    cornerWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    ui->tabWidget->setCornerWidget(cornerWidget, Qt::TopRightCorner);

    connect(ui->tabWidget, &QTabWidget::currentChanged, this, [this]() {
        const bool visible = ui->tabWidget->currentIndex() > 0;
        mRemoveParamButton->setVisible(visible);
        mReplaceParamButton->setVisible(visible);
    });

    if (mValueDisplay) {
        ui->modeComboBox->setCurrentIndex(mValueDisplay->statMode());
        ui->formatComboBox->setCurrentIndex(mValueDisplay->valueFormat());
        ui->headerCheckBox->setChecked(mValueDisplay->headerVisible());
        ui->backgroundComboBox->setCurrentIndex(
            mValueDisplay->backgroundMode());

        mPropertiesWidget = new ElementPropertiesWidget(this);
        mPropertiesWidget->setVisible(false);
        mPropertiesWidget->setProject(display->board()->project());
        connect(mPropertiesWidget, SIGNAL(connectProperties(bool)), this,
                SLOT(connectProperties(bool)));

        QSharedPointer<BoardParameter> dashParam =
            mValueDisplay->boardParameter(0);
        if (dashParam) {
            if (dashParam->connected()) {
                mPropertiesWidget->setEditionMode(
                    ElementPropertiesWidget::emElementConnected);
                mPropertiesWidget->updateUi(
                    dashParam->sharedParameterConfiguration());
            }
            else {
                if (dashParam->sharedParameterConfiguration()) {
                    mPropertiesWidget->setEditionMode(
                        ElementPropertiesWidget::emElementDisconnected);
                }
                else {
                    mPropertiesWidget->setEditionMode(
                        ElementPropertiesWidget::emElementStandAlone);
                }
                mPropertiesWidget->updateUi(
                    dashParam->exclusiveParameterConfiguration());
            }
            mPropertiesWidget->setPropertiesMode(
                ParameterConfiguration::cmValue);
            mPropertiesWidget->setVisible(true);
            ui->tabWidget->insertTab(1, mPropertiesWidget,
                                     dashParam->getTimeSeries().name());
        }
    }
    setMinimumSize(600, 400);
}

NumericalDisplayEditor::~NumericalDisplayEditor()
{
    disconnect(ui->tabWidget, &QTabWidget::currentChanged, this, nullptr);
    if (mPropertiesWidget)
        delete mPropertiesWidget;
    delete ui;
}

void NumericalDisplayEditor::accept()
{
    updateElement();
    QDialog::accept();
}

void NumericalDisplayEditor::updateElement()
{
    if (mValueDisplay) {
        mValueDisplay->setStatMode(
            NumericalDisplay::StatMode(ui->modeComboBox->currentIndex()));
        mValueDisplay->setValueFormat(
            ValueStringFormat(ui->formatComboBox->currentIndex()));
        mValueDisplay->setHeaderVisible(ui->headerCheckBox->isChecked());
        mValueDisplay->setBackgroundMode(BoardElement::BackgroundMode(
            ui->backgroundComboBox->currentIndex()));
        QSharedPointer<BoardParameter> dashParam =
            mValueDisplay->boardParameter(0);
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
        mValueDisplay->rebuildElement(true);
    }
}

void NumericalDisplayEditor::connectProperties(bool connected)
{
    QSharedPointer<BoardParameter> dashParam = mValueDisplay->boardParameter(0);
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
        mPropertiesWidget->setPropertiesMode(ParameterConfiguration::cmValue);
    }
}

void NumericalDisplayEditor::newParameter()
{
    QString paramLabel =
        QInputDialog::getText(this, "Parameter Label", "Parameter Label");
    if (!paramLabel.isEmpty()) {
        mValueDisplay->addParameter(paramLabel);

        QSharedPointer<BoardParameter> dashParam =
            mValueDisplay->boardParameter(0);
        if (dashParam) {
            mPropertiesWidget->setEditionMode(
                ElementPropertiesWidget::emElementStandAlone);
            mPropertiesWidget->setPropertiesMode(
                ParameterConfiguration::cmValue);
            mPropertiesWidget->updateUi(
                dashParam->exclusiveParameterConfiguration());
            mPropertiesWidget->setVisible(true);
            ui->tabWidget->insertTab(1, mPropertiesWidget,
                                     dashParam->getDisplayedLabel());
        }
        if (!mValueDisplay->board()->dataManager()->liveDataEnabled()) {
            mValueDisplay->board()->resetPlayback();
            mValueDisplay->board()->replot(QCustomPlot::rpQueuedReplot);
        }
    }
}

void NumericalDisplayEditor::removeParameter()
{
    mValueDisplay->removeBoardParameter(0);
    ui->tabWidget->removeTab(1);
}

void NumericalDisplayEditor::replaceParameter()
{
    if (ui->tabWidget->currentIndex() > 0) {
        QString paramLabel =
            QInputDialog::getText(this, "Parameter Label", "Parameter Label");
        if (!paramLabel.isEmpty()) {
            QSharedPointer<BoardParameter> dashParam =
                mValueDisplay->replaceParameter(
                    ui->tabWidget->currentIndex() - 1, paramLabel);
            if (dashParam) {
                mPropertiesWidget->setEditionMode(
                    ElementPropertiesWidget::emElementStandAlone);
                mPropertiesWidget->setPropertiesMode(
                    ParameterConfiguration::cmValue);
                mPropertiesWidget->updateUi(
                    dashParam->exclusiveParameterConfiguration());
                mPropertiesWidget->setVisible(true);
                ui->tabWidget->insertTab(1, mPropertiesWidget,
                                         dashParam->getDisplayedLabel());
            }
            if (!mValueDisplay->board()->dataManager()->liveDataEnabled()) {
                mValueDisplay->board()->resetPlayback();
                mValueDisplay->board()->replot(QCustomPlot::rpQueuedReplot);
            }
        }
    }
}

///------- NumericalDisplay starts from here
NumericalDisplay::NumericalDisplay(Board *dashboard)
    : SingleDisplay(dashboard),
      mOrientation(doVerticalAlignCenter),
      mValueFormat(vsfDecimal),
      mStatMode(smNone)
{
    setConfigurationMode(ParameterConfiguration::cmValue);
    setName("_NumericalDisplay");
}

void NumericalDisplay::initialize(Board *dashboard)
{
    if (dashboard) {
        SingleDisplay::initialize(dashboard);

        mTextMode = new AdaptiveTextElement(dashboard);
        mTextMode->setTextFlags(Qt::AlignCenter);
        mTextMode->setMaxPointSize(8);
        mTextMode->setMinPointSize(1);
        mTextMode->setLayer(QLatin1String("main"));
        mTextMode->setCachedPixmap(true);

        mUnitLayout = new LayoutGrid();
        mUnitLayout->initializeParentPlot(dashboard);
        mUnitLayout->setParentLayerable(this);
        mUnitLayout->setLayer(QLatin1String("main"));
    }
}

void NumericalDisplay::resetElement()
{
    SingleDisplay::resetElement();

    mStatMaxValue = -1. * std::numeric_limits<double>::max();
    mStatMinValue = std::numeric_limits<double>::max();

    for (int i = 0; i < mUnitLayout->elementCount(); i++)
        mUnitLayout->takeAt(i);
    mUnitLayout->simplify();
}

void NumericalDisplay::buildElement()
{
    SingleDisplay::buildElement();

    mUnitLayout->setFillOrder(QCPLayoutGrid::foColumnsFirst);
    mMainLayout->setFillOrder(QCPLayoutGrid::foRowsFirst);

    switch (mOrientation) {
        case doVerticalAlignCenter:
            mTextLabel->setTextFlags(Qt::AlignCenter);
            mTextUnit->setTextFlags(Qt::AlignCenter);
            mTextMode->setTextFlags(Qt::AlignCenter);
            mTextValue->setTextFlags(Qt::AlignCenter);
            break;
        case doVerticalAlignLeftRight:
            mTextLabel->setTextFlags(Qt::AlignVCenter | Qt::AlignLeft);
            mTextUnit->setTextFlags(Qt::AlignVCenter | Qt::AlignLeft);
            mTextMode->setTextFlags(Qt::AlignVCenter | Qt::AlignRight);
            mTextValue->setTextFlags(Qt::AlignCenter);
            break;
    }

    mTextLabel->needUpdate(true);
    mTextUnit->needUpdate(true);
    mTextMode->needUpdate(true);

    ColorSettings colorSet;
    switch (mStatMode) {
        case smNone:
            mTextMode->setText("");
            colorSet.setMode(ColorSettings::cmNoBackground);
            mTextMode->setBackgroundBrush(colorSet.backgroundBrush());
            mTextMode->setTextColor(colorSet.foregroundColor());
            mUnitLayout->addElement(mTextUnit);
            break;
        case smMin:
            mTextMode->setText("Min");
            if (mStatModeEnabled)
                colorSet.setMode(ColorSettings::cmFilledBackground);
            else
                colorSet.setMode(ColorSettings::cmNoBackground);

            mTextMode->setBackgroundBrush(colorSet.backgroundBrush());
            mTextMode->setTextColor(colorSet.foregroundColor());

            mUnitLayout->addElement(mTextUnit);
            mUnitLayout->addElement(mTextMode);
            break;
        case smMax:
            mTextMode->setText("Max");
            if (mStatModeEnabled)
                colorSet.setMode(ColorSettings::cmFilledBackground);
            else
                colorSet.setMode(ColorSettings::cmNoBackground);

            mTextMode->setBackgroundBrush(colorSet.backgroundBrush());
            mTextMode->setTextColor(colorSet.foregroundColor());

            mUnitLayout->addElement(mTextUnit);
            mUnitLayout->addElement(mTextMode);
            break;
    }

    mTextMode->needUpdate(true);
    mUnitLayout->setColumnSpacing(0);

    mTextMode->setVisible(mHeaderVisible | mStatModeEnabled);
    mTextLabel->setVisible(mHeaderVisible);
    mTextUnit->setVisible(mHeaderVisible);

    if (mHeaderVisible) {
        mMainLayout->addElement(mTextLabel);
        mMainLayout->addElement(mUnitLayout);
        mMainLayout->setRowStretchFactor(0, 0.001);
        mMainLayout->setRowStretchFactor(1, 0.001);
    }
    mMainLayout->addElement(mTextValue);
    mMainLayout->setRowSpacing(0);
    mMainLayout->setRowStretchFactor(mMainLayout->elementCount() - 1, 1);

    mUnitLayout->needUpdate(true);
    mMainLayout->needUpdate(true);
}

void NumericalDisplay::edit()
{
    NumericalDisplayEditor editor(this);
    editor.exec();
}

void NumericalDisplay::loadSettings(QSettings *settings)
{
    SingleDisplay::loadSettings(settings);

    settings->beginGroup("SpecDisplay");
    if (settings->contains("Orientation"))
        mOrientation =
            DisplayOrientation(settings->value("Orientation").toInt());
    if (settings->contains("Format"))
        mValueFormat = ValueStringFormat(settings->value("Format").toInt());
    if (settings->contains("StatMode"))
        mStatMode = StatMode(settings->value("StatMode").toInt());
    settings->endGroup();
}

void NumericalDisplay::saveSettings(QSettings *settings)
{
    SingleDisplay::saveSettings(settings);
    settings->beginGroup("SpecDisplay");
    settings->setValue("Orientation", mOrientation);
    settings->setValue("Format", mValueFormat);
    settings->setValue("StatMode", mStatMode);
    settings->endGroup();
}

void NumericalDisplay::loadData()
{
    QSharedPointer<BoardParameter> dashParam = boardParameter(0);
    if (dashParam) {
        dashParam->updateData();
    }
}

void NumericalDisplay::displayData()
{
    QSharedPointer<BoardParameter> dashParam = boardParameter(0);
    if (dashParam && dashParam->getTimeSeries().parameterId() > 0) {
        if (!mStatModeEnabled) {
            dashParam->processValueData();
            mTextValue->setText(dashParam->getValueString(mValueFormat));
            mTextValue->setTextColor(dashParam->getForegroundColor());
            mTextValue->setBackgroundBrush(dashParam->getBackgroundBrush());
        }
        else {
            switch (mStatMode) {
                case smNone:
                    dashParam->processValueData();
                    mTextValue->setText(
                        dashParam->getValueString(mValueFormat));
                    mTextValue->setTextColor(dashParam->getForegroundColor());
                    mTextValue->setBackgroundBrush(
                        dashParam->getBackgroundBrush());
                    break;
                case smMin: {
                    bool found = false;
                    QVector<DataValue> values;
                    QVector<double> timestamps;
                    if (mBoard->dataManager()->lastBoardData(
                            dashParam->getTimeSeries().parameterId(), values,
                            timestamps)) {
                        QVector<DataValue>::const_iterator itValues;
                        QVector<double>::const_iterator itTimestamps;

                        for (itValues = values.constBegin(),
                            itTimestamps = timestamps.constBegin();
                             (itValues != values.constEnd()) &&
                             (itTimestamps != timestamps.constEnd());
                             ++itValues, ++itTimestamps) {
                            if (itValues->toDouble() < mStatMinValue) {
                                mStatMinValue = itValues->toDouble();
                                found = true;
                            }
                        }
                    }

                    if (found) {
                        dashParam->processValueData(mStatMinValue);
                        mTextValue->setText(QString::number(
                            mStatMinValue, 'f',
                            dashParam->parameterConfiguration()->precision()));

                        mTextValue->setTextColor(
                            dashParam->getForegroundColor());
                        mTextValue->setBackgroundBrush(
                            dashParam->getBackgroundBrush());
                    }
                } break;
                case smMax: {
                    bool found = false;
                    QVector<DataValue> values;
                    QVector<double> timestamps;
                    if (mBoard->dataManager()->lastBoardData(
                            dashParam->getTimeSeries().parameterId(), values,
                            timestamps)) {
                        QVector<DataValue>::const_iterator itValues;
                        QVector<double>::const_iterator itTimestamps;

                        for (itValues = values.constBegin(),
                            itTimestamps = timestamps.constBegin();
                             (itValues != values.constEnd()) &&
                             (itTimestamps != timestamps.constEnd());
                             ++itValues, ++itTimestamps) {
                            if (itValues->toDouble() > mStatMaxValue) {
                                mStatMaxValue = itValues->toDouble();
                                found = true;
                            }
                        }
                    }
                    if (found) {
                        dashParam->processValueData(mStatMaxValue);
                        mTextValue->setText(QString::number(
                            mStatMaxValue, 'f',
                            dashParam->parameterConfiguration()->precision()));

                        mTextValue->setTextColor(
                            dashParam->getForegroundColor());
                        mTextValue->setBackgroundBrush(
                            dashParam->getBackgroundBrush());
                    }
                } break;
            }
        }
    }
}

void NumericalDisplay::applySizeConstraints()
{
    QSizeF defaultSize = mBoard->dashboardLayout()->singleElementSize();
    defaultSize.rwidth() -= mMargins.left() + mMargins.right() +
                            mMainLayout->margins().left() +
                            mMainLayout->margins().right();
    defaultSize.rheight() -= mMargins.top() + mMargins.bottom() +
                             mMainLayout->margins().top() +
                             mMainLayout->margins().bottom();

    QSizeF currentSize = mRect.size();
    currentSize.rwidth() -=
        mMainLayout->margins().left() + mMainLayout->margins().right();
    currentSize.rheight() -=
        mMainLayout->margins().top() + mMainLayout->margins().bottom();

    QSizeF labelSize = defaultSize;

    if (currentSize.height() < defaultSize.height())
        labelSize = currentSize;

    mTextLabel->setMinimumSize(labelSize.width(),
                               LABEL_HEIGHT_RATIO * labelSize.height());
    mTextUnit->setMinimumSize(labelSize.width(),
                              UNIT_HEIGHT_RATIO * labelSize.height());
    mTextMode->setMinimumSize(labelSize.width(),
                              UNIT_HEIGHT_RATIO * labelSize.height());
    mTextValue->setMinimumSize(labelSize.width(),
                               VALUE_HEIGHT_RATIO * labelSize.height());
}

ValueStringFormat NumericalDisplay::valueFormat() const { return mValueFormat; }

void NumericalDisplay::setValueFormat(ValueStringFormat valueFormat)
{
    mValueFormat = valueFormat;
}

NumericalDisplay::DisplayOrientation NumericalDisplay::orientation() const
{
    return mOrientation;
}

void NumericalDisplay::setOrientation(DisplayOrientation orientation)
{
    mOrientation = orientation;
}

NumericalDisplay::StatMode NumericalDisplay::statMode() const
{
    return mStatMode;
}

void NumericalDisplay::setStatMode(StatMode statMode) { mStatMode = statMode; }

} // namespace tl

#include "NumericalDisplay.moc"

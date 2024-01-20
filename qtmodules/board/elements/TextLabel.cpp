#include "TextLabel.h"
#include "editor/ui_TextLabelEditor.h"

#include "Board.h"
#include "BoardElement.h"
#include "BoardLayout.h"
#include "utils/AdaptiveTextElement.h"
#include "utils/LayoutGrid.h"

namespace tl {

///------- TextLabel Editor starts from here
class TextLabelEditor : public QDialog
{
    Q_OBJECT

public:
    explicit TextLabelEditor(TextLabel *display, QWidget *parent = nullptr);
    ~TextLabelEditor() override;

    void accept() override;

private slots:
    void updateElement();

    void on_colorToolButton_clicked();
    void on_colorModeComboBox_currentIndexChanged(int index);

private:
    Ui::TextLabelEditor *ui;
    TextLabel *mElement;
};

TextLabelEditor::TextLabelEditor(TextLabel *display, QWidget *parent)
    : QDialog(parent), ui(new Ui::TextLabelEditor), mElement(display)
{
    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(0);
    ui->listWidget->setCurrentRow(0);
    connect(ui->buttonBox->button(QDialogButtonBox::Apply),
            &QPushButton::clicked, this, &TextLabelEditor::updateElement);

    if (mElement) {
        ui->backgroundComboBox->setCurrentIndex(mElement->backgroundMode());
        ui->textLineEdit->setText(mElement->text());
        ui->colorLineEdit->setColor(mElement->colorSettingsRef().color());
        ui->colorLineEdit->setMode(mElement->colorSettingsRef().mode());
        ui->colorModeComboBox->setCurrentIndex(
            mElement->colorSettingsRef().mode() - 1);
    }
    setMinimumSize(600, 400);
}

TextLabelEditor::~TextLabelEditor() { delete ui; }

void TextLabelEditor::accept()
{
    updateElement();
    QDialog::accept();
}

void TextLabelEditor::updateElement()
{
    if (mElement) {
        mElement->setBackgroundMode(BoardElement::BackgroundMode(
            ui->backgroundComboBox->currentIndex()));
        mElement->setText(ui->textLineEdit->text());
        mElement->colorSettingsRef().setColor(ui->colorLineEdit->color());
        mElement->colorSettingsRef().setMode(ui->colorLineEdit->mode());
        mElement->rebuildElement(true);
    }
}

void TextLabelEditor::on_colorToolButton_clicked()
{
    QColor color = QColorDialog::getColor(ui->colorLineEdit->color(), this,
                                          QString("Color picker"),
                                          QColorDialog::ShowAlphaChannel);
    if (color.isValid()) {
        ui->colorLineEdit->setColor(color);
    }
}

void TextLabelEditor::on_colorModeComboBox_currentIndexChanged(int index)
{
    ui->colorLineEdit->setMode(ColorSettings::ColorMode(index + 1));
}

///------- TextLabel starts from here
TextLabel::TextLabel(Board *dashboard)
    : BoardElement(dashboard), mText("Empty text")
{
    setType(BoardElement::etOther);
    setParametersMaxCount(0);
}

int TextLabel::defaultWidth()
{
    return 4 * mBoard->dashboardLayout()->singleElementColumnCount();
}

int TextLabel::defaultHeight()
{
    return 1 * mBoard->dashboardLayout()->singleElementRowCount();
}

void TextLabel::initialize(Board *dashboard)
{
    if (dashboard) {
        BoardElement::initialize(dashboard);

        auto textColor = QApplication::palette().toolTipText().color();
        mTextElement = new AdaptiveTextElement(dashboard);
        mTextElement->setTextFlags(Qt::AlignCenter);
        mTextElement->setMaxPointSize(16);
        mTextElement->setMinPointSize(1);
        mTextElement->setTextColor(textColor);
        mTextElement->setText(mText);
        mTextElement->setAdjustStrategy(AdaptiveTextElement::asAdjustPointSize);
        mTextElement->setCachedPixmap(true);

        mMainLayout = new LayoutGrid();
        mMainLayout->initializeParentPlot(dashboard);
        mMainLayout->setLayer(QLatin1String("main"));
        mMainLayout->setFillOrder(QCPLayoutGrid::foRowsFirst);
        mMainLayout->setRowSpacing(0);
        mMainLayout->setMargins({});
        mMainLayout->setMinimumMargins({});
    }
}

void TextLabel::resetElement()
{
    BoardElement::resetElement();

    for (int i = 0; i < mMainLayout->elementCount(); i++)
        mMainLayout->takeAt(i);
    mMainLayout->simplify();
}

void TextLabel::buildElement()
{
    mTextElement->setText(mText);
    mTextElement->setBackgroundBrush(mColorSettings.backgroundBrush());
    mTextElement->setTextColor(mColorSettings.foregroundColor());
    mTextElement->needUpdate(true);

    mMainLayout->addElement(mTextElement);
    mMainLayout->needUpdate(true);
}

void TextLabel::clearElement()
{
    delete mTextElement;
    delete mMainLayout;
}

void TextLabel::edit()
{
    TextLabelEditor editor(this);
    editor.exec();
}

void TextLabel::update(QCPLayoutElement::UpdatePhase phase)
{
    BoardElement::update(phase);

    if (phase == upLayout)
        mMainLayout->setOuterRect(rect());

    mMainLayout->update(phase);
}

void TextLabel::saveSettings(QSettings *settings)
{
    settings->beginGroup("TextLabelSettings");
    settings->setValue("Text", mText);
    settings->setValue("Color", mColorSettings.color().name(QColor::HexArgb));
    settings->setValue("ColorMode", mColorSettings.mode());
    settings->setValue("BackgroundMode", mBackgroundMode);
    settings->endGroup();
}

void TextLabel::loadSettings(QSettings *settings)
{
    settings->beginGroup("TextLabelSettings");
    mText = settings->value("Text", "Empty Text").toString();
    mColorSettings.setColor(QColor(settings->value("Color").toString()));
    mColorSettings.setMode(
        ColorSettings::ColorMode(settings->value("ColorMode").toInt()));
    if (settings->contains("BackgroundMode")) {
        auto maxVal = QMetaEnum::fromType<BackgroundMode>().keyCount();
        auto val = settings->value("BackgroundMode").toInt();
        if (val < maxVal)
            setBackgroundMode(BackgroundMode(val));
    }
    settings->endGroup();
}

QString TextLabel::text() const { return mText; }

void TextLabel::setText(const QString &text) { mText = text; }

ColorSettings &TextLabel::colorSettingsRef() { return mColorSettings; }

} // namespace tl

#include "TextLabel.moc"

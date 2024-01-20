#pragma once

#include <QCommonStyle>

#include "TheStyleTypes.h"
#include "TheTheme.h"

class QAbstractItemView;

namespace tl {

class CommandLinkButtonPaintEventFilter;
class LineEditButtonEventFilter;

class QlementineStylePrivate;
class QlementineStyle : public QCommonStyle
{
    Q_OBJECT
    Q_PROPERTY(bool animationsEnabled READ animationsEnabled WRITE
                   setAnimationsEnabled NOTIFY animationsEnabledChanged)

public:
    // static constexpr char Property_DoNotColorizeIcon[] = "DoNotColorizeIcon";

    enum StandardPixmapExt
    {
        SP_Check = SP_CustomBase + 1,
        SP_Calendar,
    };

    enum ControlElementExt
    {
        CE_CommandButtonLabel = CE_CustomBase + 1,
        CE_CommandButton,
    };

    enum ContentsTypeExt
    {
        CT_CommandButton = CT_CustomBase + 1,
    };

    enum PixelMetricExt
    {
        PM_MediumIconSize = PM_CustomBase + 1,
    };

    enum PrimitiveElementExt
    {
        PE_CommandButtonPanel = PE_CustomBase + 1,
        PE_CommandButtonLabel,
    };

public:
    explicit QlementineStyle(QObject* parent = nullptr);
    ~QlementineStyle() override;

    const Theme& theme() const;
    void setTheme(const Theme& theme);
    void setThemeJsonPath(const QString& path);
    Q_SIGNAL void themeChanged();

    bool animationsEnabled() const;
    void setAnimationsEnabled(bool enabled);
    Q_SIGNAL void animationsEnabledChanged();

    bool useMenuForComboBoxPopup() const;
    void setUseMenuForComboBoxPopup(bool useMenu);
    Q_SIGNAL void useMenuForComboBoxPopupChanged();

    void triggerCompleteRepaint();

    void setAutoIconColorEnabled(bool enabled);
    bool isAutoIconColorEnabled() const;

    void setAutoIconColorEnabled(QWidget* widget, bool enabled);
    bool isAutoIconColorEnabled(const QWidget* widget) const;

    static QIcon makeIcon(const QString& svgPath);

public:
    void drawPrimitive(PrimitiveElement pe, const QStyleOption* opt,
                       QPainter* p, const QWidget* w = nullptr) const override;

    void drawControl(ControlElement ce, const QStyleOption* opt, QPainter* p,
                     const QWidget* w = nullptr) const override;

    QRect subElementRect(SubElement se, const QStyleOption* opt,
                         const QWidget* w = nullptr) const override;

    void drawComplexControl(ComplexControl cc, const QStyleOptionComplex* opt,
                            QPainter* p,
                            const QWidget* w = nullptr) const override;

    SubControl hitTestComplexControl(ComplexControl cc,
                                     const QStyleOptionComplex* opt,
                                     const QPoint& pos,
                                     const QWidget* w = nullptr) const override;

    QRect subControlRect(ComplexControl cc, const QStyleOptionComplex* opt,
                         SubControl sc,
                         const QWidget* w = nullptr) const override;

    QSize sizeFromContents(ContentsType ct, const QStyleOption* opt,
                           const QSize& s,
                           const QWidget* w = nullptr) const override;

    int pixelMetric(PixelMetric m, const QStyleOption* opt = nullptr,
                    const QWidget* w = nullptr) const override;

    int styleHint(StyleHint sh, const QStyleOption* opt = nullptr,
                  const QWidget* w = nullptr,
                  QStyleHintReturn* shret = nullptr) const override;

    QPalette standardPalette() const override;

    QIcon standardIcon(StandardPixmap sp, const QStyleOption* opt = nullptr,
                       const QWidget* w = nullptr) const override;

    QPixmap standardPixmap(StandardPixmap sp, const QStyleOption* opt = nullptr,
                           const QWidget* w = nullptr) const override;

    QPixmap generatedIconPixmap(QIcon::Mode im, const QPixmap& pixmap,
                                const QStyleOption* opt) const override;

    int layoutSpacing(QSizePolicy::ControlType c1, QSizePolicy::ControlType c2,
                      Qt::Orientation o, const QStyleOption* opt = nullptr,
                      const QWidget* w = nullptr) const override;

    void polish(QPalette& palette) override;
    void polish(QApplication* app) override;
    void polish(QWidget* w) override;
    void unpolish(QWidget* w) override;
    void unpolish(QApplication* app) override;

    virtual const QColor& color(MouseState mouse, ColorRole role) const;

    virtual const QColor& frameBackgroundColor(MouseState mouse) const;

    virtual const QColor& buttonBackgroundColor(MouseState mouse,
                                                ColorRole role) const;
    virtual const QColor& buttonForegroundColor(MouseState mouse,
                                                ColorRole role) const;

    virtual const QColor& toolButtonBackgroundColor(MouseState mouse,
                                                    ColorRole role) const;
    virtual const QColor& toolButtonForegroundColor(MouseState mouse,
                                                    ColorRole role) const;
    virtual const QColor& toolButtonSeparatorColor(MouseState mouse,
                                                   ColorRole role) const;

    virtual const QColor& commandButtonBackgroundColor(MouseState mouse,
                                                       ColorRole role) const;
    virtual const QColor& commandButtonTextColor(MouseState mouse,
                                                 ColorRole role) const;
    virtual const QColor& commandButtonDescriptionColor(MouseState mouse,
                                                        ColorRole role) const;
    virtual const QColor& commandButtonIconColor(MouseState mouse,
                                                 ColorRole role) const;

    virtual const QColor& checkButtonBackgroundColor(
        MouseState mouse, Qt::CheckState checked) const;
    virtual const QColor& checkButtonForegroundColor(
        MouseState mouse, Qt::CheckState checked) const;
    virtual const QColor& checkButtonBorderColor(MouseState mouse,
                                                 FocusState focus,
                                                 Qt::CheckState checked) const;

    virtual const QColor& radioButtonBackgroundColor(
        MouseState mouse, Qt::CheckState checked) const;
    virtual const QColor& radioButtonForegroundColor(
        MouseState mouse, Qt::CheckState checked) const;
    virtual const QColor& radioButtonBorderColor(MouseState mouse,
                                                 FocusState focus,
                                                 Qt::CheckState checked) const;

    virtual const QColor& comboBoxBackgroundColor(MouseState mouse) const;
    virtual const QColor& comboBoxForegroundColor(MouseState mouse) const;
    virtual const QColor& comboBoxTextColor(MouseState mouse, Status status,
                                            const QWidget* w = nullptr) const;

    virtual const QColor& spinBoxBackgroundColor(MouseState mouse) const;
    virtual const QColor& spinBoxBorderColor(MouseState mouse,
                                             FocusState focus) const;
    virtual const QColor& spinBoxButtonBackgroundColor(MouseState mouse) const;
    virtual const QColor& spinBoxButtonForegroundColor(MouseState mouse) const;

    virtual const QColor& listItemRowBackgroundColor(
        MouseState mouse, AlternateState alternate) const;
    virtual const QColor& listItemBackgroundColor(MouseState mouse,
                                                  SelectionState selected,
                                                  FocusState focus,
                                                  ActiveState active) const;
    virtual const QColor& listItemForegroundColor(MouseState mouse,
                                                  SelectionState selected,
                                                  FocusState focus,
                                                  ActiveState active) const;
    virtual bool listItemIsAutoIconColorEnabled(
        MouseState mouse, SelectionState selected, FocusState focus,
        ActiveState active, const QModelIndex& index,
        const QWidget* widget = nullptr) const;
    virtual const QColor& listItemCaptionForegroundColor(
        MouseState mouse, SelectionState selected, FocusState focus,
        ActiveState active) const;
    virtual const QColor& listItemCheckButtonBackgroundColor(
        MouseState mouse, Qt::CheckState checked, SelectionState selected,
        ActiveState active) const;
    virtual const QColor& listItemCheckButtonBorderColor(
        MouseState mouse, Qt::CheckState checked, SelectionState selected,
        ActiveState active) const;
    virtual const QColor& listItemCheckButtonForegroundColor(
        MouseState mouse, Qt::CheckState checked, SelectionState selected,
        ActiveState active) const;
    virtual const QColor& cellItemFocusBorderColor(FocusState focus,
                                                   SelectionState selected,
                                                   ActiveState active) const;

    virtual const QColor& menuBackgroundColor() const;
    virtual const QColor& menuBorderColor() const;
    virtual const QColor& menuSeparatorColor() const;

    virtual const QColor& menuItemBackgroundColor(MouseState mouse) const;
    virtual const QColor& menuItemForegroundColor(MouseState mouse) const;
    virtual const QColor& menuItemSecondaryForegroundColor(
        MouseState const mouse) const;

    virtual const QColor& menuBarBackgroundColor() const;
    virtual const QColor& menuBarBorderColor() const;
    virtual const QColor& menuBarItemBackgroundColor(
        MouseState mouse, SelectionState selected) const;
    virtual const QColor& menuBarItemForegroundColor(
        MouseState mouse, SelectionState selected) const;

    virtual const QColor& tabBarBackgroundColor() const;
    virtual const QColor& tabBarShadowColor() const;
    virtual const QColor& tabBarBottomShadowColor() const;
    virtual const QColor& tabBackgroundColor(MouseState mouse,
                                             SelectionState selected) const;
    virtual const QColor& tabForegroundColor(MouseState mouse,
                                             SelectionState selected) const;
    virtual const QColor& tabCloseButtonBackgroundColor(
        MouseState mouse, SelectionState selected) const;
    virtual const QColor& tabCloseButtonForegroundColor(
        MouseState mouse, SelectionState selected) const;
    virtual const QColor& tabBarScrollButtonBackgroundColor(
        MouseState mouse) const;

    virtual const QColor& progressBarGrooveColor(MouseState mouse) const;
    virtual const QColor& progressBarValueColor(MouseState mouse) const;

    virtual const QColor& textFieldBackgroundColor(MouseState mouse,
                                                   Status status) const;
    virtual const QColor& textFieldBorderColor(MouseState mouse,
                                               FocusState focus,
                                               Status status) const;
    virtual const QColor& textFieldForegroundColor(MouseState mouse) const;

    virtual const QColor& sliderGrooveColor(MouseState mouse) const;
    virtual const QColor& sliderValueColor(MouseState mouse) const;
    virtual const QColor& sliderHandleColor(MouseState mouse) const;
    virtual const QColor& sliderTickColor(MouseState mouse) const;

    virtual const QColor& dialHandleColor(MouseState mouse) const;
    virtual const QColor& dialGrooveColor(MouseState mouse) const;
    virtual const QColor& dialValueColor(MouseState mouse) const;
    virtual const QColor& dialTickColor(MouseState mouse) const;
    virtual const QColor& dialMarkColor(MouseState mouse) const;
    virtual const QColor& dialBackgroundColor(MouseState mouse) const;

    virtual const QColor& labelForegroundColor(
        MouseState mouse, const QWidget* w = nullptr) const;
    virtual const QColor& labelCaptionForegroundColor(MouseState mouse) const;

    virtual const QColor& toolBarBackgroundColor() const;
    virtual const QColor& toolBarBorderColor() const;
    virtual const QColor& toolBarSeparatorColor() const;

    virtual const QColor& toolTipBackgroundColor() const;
    virtual const QColor& toolTipBorderColor() const;
    virtual const QColor& toolTipForegroundColor() const;

    virtual const QColor& scrollBarGrooveColor(MouseState mouse) const;
    virtual const QColor& scrollBarHandleColor(MouseState mouse) const;
    virtual int getScrollBarThickness(MouseState mouse) const;

    virtual const QColor& groupBoxTitleColor(MouseState mouse,
                                             const QWidget* w = nullptr) const;
    virtual const QColor& groupBoxBorderColor(MouseState mouse) const;
    virtual const QColor& groupBoxBackgroundColor(MouseState mouse) const;

    virtual const QColor& focusBorderColor() const;
    virtual const QColor& frameBorderColor() const;

    virtual const QColor& colorForTextRole(TextRole role,
                                           MouseState mouse) const;
    virtual int pixelSizeForTextRole(TextRole role) const;
    virtual const QFont& fontForTextRole(TextRole role) const;
    virtual QPalette paletteForTextRole(TextRole role) const;

    virtual const QColor& switchGrooveColor(MouseState mouse,
                                            Qt::CheckState checked) const;
    virtual const QColor& switchGrooveBorderColor(MouseState mouse,
                                                  FocusState focus,
                                                  Qt::CheckState checked) const;
    virtual const QColor& switchHandleColor(MouseState mouse,
                                            Qt::CheckState checked) const;

    virtual const QColor& tableHeaderBgColor(MouseState mouse,
                                             Qt::CheckState checked) const;
    virtual const QColor& tableHeaderFgColor(MouseState mouse,
                                             Qt::CheckState checked) const;
    virtual const QColor& tableLineColor() const;

    virtual Status widgetStatus(QWidget const* widget) const;

    friend class CommandLinkButtonPaintEventFilter;
    friend class LineEditButtonEventFilter;

private:
    Q_DISABLE_COPY(QlementineStyle)
    Q_DECLARE_PRIVATE(QlementineStyle)
    const QScopedPointer<QlementineStylePrivate> d_ptr;
};

} // namespace tl

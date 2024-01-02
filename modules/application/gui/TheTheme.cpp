#include "TheTheme.h"

#include <QFile>
#include <QGuiApplication>
#include <QJsonObject>
#include <QScreen>

#include "qcolorutils.h"
#include "qfontutils.h"
#include "qstringutils.h"

namespace thoht {

namespace {

#define TRY_GET_COLOR_ATTRIBUTE(JSON_OBJ, NAME) \
    tryGetColor(JSON_OBJ, QStringLiteral(#NAME), NAME)
#define TRY_GET_INT_ATTRIBUTE(JSON_OBJ, NAME) \
    tryGetInt(JSON_OBJ, QStringLiteral(#NAME), NAME)
#define TRY_GET_DOUBLE_ATTRIBUTE(JSON_OBJ, NAME) \
    tryGetDouble(JSON_OBJ, QStringLiteral(#NAME), NAME)

#define SET_COLOR(JSON_OBJ, NAME) \
    setColor(JSON_OBJ, QStringLiteral(#NAME), NAME)
#define SET_INT(JSON_OBJ, NAME) setInt(JSON_OBJ, QStringLiteral(#NAME), NAME)
#define SET_DOUBLE(JSON_OBJ, NAME) \
    setDouble(JSON_OBJ, QStringLiteral(#NAME), NAME)

std::optional<QColor> tryGetColorRecursive(const QJsonObject& jo,
                                           const QString& key,
                                           int maxRecursiveCalls = 1)
{
    if (jo.contains(key)) {
        const auto variant = jo.value(key).toVariant();

        // Try parse a QColor.
        const auto color = tryGetColorFromVariant(variant);
        if (color.has_value()) {
            return color.value();
        }

        // Check if the value is a reference to another value.
        if (variant.userType() == QMetaType::QString && maxRecursiveCalls > 0) {
            const auto variantString = variant.toString();
            if (variantString != key) {
                return tryGetColorRecursive(jo, variantString,
                                            --maxRecursiveCalls);
            }
        }
    }

    return {};
}

void tryGetColor(const QJsonObject& jo, const QString& key, QColor& target)
{
    if (const auto opt = tryGetColorRecursive(jo, key)) {
        target = opt.value();
    }
}

QString tryGetString(const QJsonObject& jo, const QString& key,
                     const QString& defaultValue)
{
    if (jo.contains(key)) {
        const auto variant = jo.value(key).toVariant();
        if (variant.isValid() && variant.canConvert<QString>()) {
            return variant.toString();
        }
    }
    return defaultValue;
}

std::optional<int> tryGetIntRecursive(const QJsonObject& jo, const QString& key,
                                      int maxRecursiveCalls = 1)
{
    if (jo.contains(key)) {
        const auto variant = jo.value(key).toVariant();

        // Check if the value is a reference to another value.
        if (variant.userType() == QMetaType::QString && maxRecursiveCalls > 0) {
            const auto variantString = variant.toString();
            if (variantString != key) {
                return tryGetIntRecursive(jo, variantString,
                                          --maxRecursiveCalls);
            }
        }

        // Try parse an int.
        if (variant.isValid() && variant.canConvert<int>()) {
            return variant.toInt();
        }
    }
    return {};
}

void tryGetInt(const QJsonObject& jo, const QString& key, int& target)
{
    if (const auto opt = tryGetIntRecursive(jo, key)) {
        target = opt.value();
    }
}

std::optional<double> tryGetDoubleRecursive(const QJsonObject& jo,
                                            const QString& key,
                                            int maxRecursiveCalls = 1)
{
    if (jo.contains(key)) {
        const auto variant = jo.value(key).toVariant();

        // Check if the value is a reference to another value.
        if (variant.userType() == QMetaType::QString && maxRecursiveCalls > 0) {
            const auto variantString = variant.toString();
            if (variantString != key) {
                return tryGetDoubleRecursive(jo, variantString,
                                             --maxRecursiveCalls);
            }
        }

        // Try parse an int.
        if (variant.isValid() && variant.canConvert<double>()) {
            return variant.toDouble();
        }
    }
    return {};
}

void tryGetDouble(const QJsonObject& jo, const QString& key, double& target)
{
    if (const auto opt = tryGetDoubleRecursive(jo, key)) {
        target = opt.value();
    }
}

QJsonDocument readJsonDoc(const QString& path)
{
    QFile jsonFile(path);
    if (jsonFile.open(QIODevice::ReadOnly)) {
        const auto fileContents = jsonFile.readAll();
        QJsonParseError jsonParseError{};
        auto jd = QJsonDocument::fromJson(fileContents, &jsonParseError);
        if (jsonParseError.error == QJsonParseError::ParseError::NoError &&
            !jd.isEmpty()) {
            return jd;
        }
    }

    return {};
}

void setColor(QJsonObject& jo, const QString& key, const QColor& value)
{
    jo.insert(key, toHexRGBA(value));
}

void setInt(QJsonObject& jo, const QString& key, int value)
{
    jo.insert(key, value);
}

void setDouble(QJsonObject& jo, const QString& key, double value)
{
    jo.insert(key, value);
}

} // namespace

Theme::Theme()
{
    initializeFonts();
    initializePalette();
}

Theme::Theme(const QJsonDocument& jd)
{
    initializeFromJson(jd);
    initializePalette();
}

Theme::Theme(const QString& path) : Theme(readJsonDoc(path)) {}

void Theme::initializeFonts()
{
    // Fonts.
    const auto defaultFont = QFont(u"Inter"_s);
    const auto fixedFont = QFont(u"Roboto Mono"_s);
    const auto dpi = QGuiApplication::primaryScreen()->logicalDotsPerInch();
    fontRegular = defaultFont;
    fontRegular.setWeight(QFont::Weight::Normal);
    fontRegular.setPointSizeF(pixelSizeToPointSize(fontSize, dpi));

    fontBold = defaultFont;
    fontBold.setWeight(QFont::Weight::Bold);
    fontBold.setPointSizeF(pixelSizeToPointSize(fontSize, dpi));

    fontH1 = defaultFont;
    fontH1.setWeight(QFont::Weight::Bold);
    fontH1.setPointSizeF(pixelSizeToPointSize(fontSizeH1, dpi));

    fontH2 = defaultFont;
    fontH2.setWeight(QFont::Weight::Bold);
    fontH2.setPointSizeF(pixelSizeToPointSize(fontSizeH2, dpi));

    fontH3 = defaultFont;
    fontH3.setWeight(QFont::Weight::Bold);
    fontH3.setPointSizeF(pixelSizeToPointSize(fontSizeH3, dpi));

    fontH4 = defaultFont;
    fontH4.setWeight(QFont::Weight::Bold);
    fontH4.setPointSizeF(pixelSizeToPointSize(fontSizeH4, dpi));

    fontH5 = defaultFont;
    fontH5.setWeight(QFont::Weight::Bold);
    fontH5.setPointSizeF(pixelSizeToPointSize(fontSizeH5, dpi));

    fontCaption = defaultFont;
    fontCaption.setWeight(QFont::Weight::Normal);
    fontCaption.setPointSizeF(pixelSizeToPointSize(fontSizeS1, dpi));

    fontMonospace = fixedFont;
    fontMonospace.setWeight(QFont::Weight::Normal);
    fontMonospace.setPointSizeF(pixelSizeToPointSize(fontSizeMonospace, dpi));
}

void Theme::initializePalette()
{
    // Shades.
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Window,
                     backgroundColorMain2);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Dark,
                     backgroundColorMain3);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Mid,
                     backgroundColorMain3);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Midlight,
                     backgroundColorMain2);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Light,
                     backgroundColorMain2);

    // ItemViews.
    // Compute color without alpha, to avoid color blending when the QTreeView
    // is animating.
    const auto itemViewBase = backgroundColorMain1;
    const auto itemViewAlternate = getColorSourceOver(
        itemViewBase,
        colorWithAlpha(neutralColorDisabled, neutralColorDisabled.alpha() / 2));
    const auto itemViewDisabled =
        getColorSourceOver(backgroundColorMain1, neutralColorDisabled);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Base,
                     itemViewBase);
    palette.setColor(QPalette::ColorGroup::Disabled, QPalette::ColorRole::Base,
                     itemViewDisabled);
    palette.setColor(QPalette::ColorGroup::All,
                     QPalette::ColorRole::AlternateBase, itemViewAlternate);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::AlternateBase, itemViewDisabled);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::NoRole,
                     backgroundColorMainTransparent);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::NoRole,
                     backgroundColorMainTransparent);

    // Tooltips.
    palette.setColor(QPalette::ColorGroup::All,
                     QPalette::ColorRole::ToolTipBase, secondaryColor);
    palette.setColor(QPalette::ColorGroup::All,
                     QPalette::ColorRole::ToolTipText,
                     secondaryColorForeground);

    // Highlight.
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Highlight,
                     primaryColor);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::Highlight, primaryColorDisabled);
    palette.setColor(QPalette::ColorGroup::All,
                     QPalette::ColorRole::HighlightedText,
                     primaryColorForeground);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::HighlightedText,
                     primaryColorDisabled);

    // Text.
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Text,
                     secondaryColor);
    palette.setColor(QPalette::ColorGroup::Disabled, QPalette::ColorRole::Text,
                     secondaryColorDisabled);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::WindowText,
                     secondaryColor);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::WindowText, secondaryColorDisabled);
    palette.setColor(QPalette::ColorGroup::All,
                     QPalette::ColorRole::PlaceholderText,
                     secondaryColorDisabled);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::PlaceholderText,
                     neutralColorDisabled);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Link,
                     primaryColor);
    palette.setColor(QPalette::ColorGroup::Disabled, QPalette::ColorRole::Link,
                     secondaryColorDisabled);
    palette.setColor(QPalette::ColorGroup::All,
                     QPalette::ColorRole::LinkVisited, primaryColor);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::LinkVisited, secondaryColorDisabled);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::BrightText,
                     secondaryAlternativeColor);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::BrightText,
                     secondaryAlternativeColorDisabled);

    // Buttons.
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::ButtonText,
                     secondaryColorForeground);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::ButtonText,
                     secondaryColorForegroundDisabled);
    palette.setColor(QPalette::ColorGroup::All, QPalette::ColorRole::Button,
                     neutralColor);
    palette.setColor(QPalette::ColorGroup::Normal, QPalette::ColorRole::Button,
                     neutralColor);
    palette.setColor(QPalette::ColorGroup::Current, QPalette::ColorRole::Button,
                     neutralColorHovered);
    palette.setColor(QPalette::ColorGroup::Active, QPalette::ColorRole::Button,
                     neutralColorPressed);
    palette.setColor(QPalette::ColorGroup::Disabled,
                     QPalette::ColorRole::Button, neutralColorDisabled);

    // palette.setColor(QPalette::ColorGroup::, QPalette::ColorRole::Background,
    // backgroundColor);
}

void Theme::initializeFromJson(const QJsonDocument& jd)
{
    if (jd.isObject()) {
        const auto jo = jd.object();
        if (!jo.isEmpty()) {
            // Parse metadata.
            auto const metaObj = jo.value(u"meta"_s).toObject();
            meta = ThemeMeta{
                tryGetString(metaObj, u"name"_s, {}),
                tryGetString(metaObj, u"version"_s, {}),
                tryGetString(metaObj, u"author"_s, {}),
            };

            // Parse all values.
            TRY_GET_COLOR_ATTRIBUTE(jo, backgroundColorMain1);
            TRY_GET_COLOR_ATTRIBUTE(jo, backgroundColorMain2);
            TRY_GET_COLOR_ATTRIBUTE(jo, backgroundColorMain3);
            TRY_GET_COLOR_ATTRIBUTE(jo, backgroundColorMain4);
            backgroundColorMainTransparent =
                colorWithAlpha(backgroundColorMain1, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, neutralColorDisabled);
            TRY_GET_COLOR_ATTRIBUTE(jo, neutralColor);
            TRY_GET_COLOR_ATTRIBUTE(jo, neutralColorHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, neutralColorPressed);
            neutralColorTransparent = colorWithAlpha(neutralColorDisabled, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, focusColor);

            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColor);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColorHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColorPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColorDisabled);
            primaryColorTransparent = colorWithAlpha(primaryColor, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColorForeground);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColorForegroundHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColorForegroundPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryColorForegroundDisabled);
            primaryColorForegroundTransparent =
                colorWithAlpha(primaryColorForeground, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, primaryAlternativeColor);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryAlternativeColorHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryAlternativeColorPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, primaryAlternativeColorDisabled);
            primaryAlternativeColorTransparent =
                colorWithAlpha(primaryAlternativeColor, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColor);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColorHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColorPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColorDisabled);
            secondaryColorTransparent = colorWithAlpha(secondaryColor, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryAlternativeColor);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryAlternativeColorHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryAlternativeColorPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryAlternativeColorDisabled);
            secondaryAlternativeColorTransparent =
                colorWithAlpha(secondaryAlternativeColor, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColorForeground);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColorForegroundHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColorForegroundPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, secondaryColorForegroundDisabled);
            secondaryColorForegroundTransparent =
                colorWithAlpha(secondaryColorForeground, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorSuccess);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorSuccessHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorSuccessPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorSuccessDisabled);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorInfo);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorInfoHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorInfoPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorInfoDisabled);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorWarning);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorWarningHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorWarningPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorWarningDisabled);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorError);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorErrorHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorErrorPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorErrorDisabled);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorForeground);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorForegroundHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorForegroundPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, statusColorForegroundDisabled);

            TRY_GET_COLOR_ATTRIBUTE(jo, borderColor);
            TRY_GET_COLOR_ATTRIBUTE(jo, borderColorHovered);
            TRY_GET_COLOR_ATTRIBUTE(jo, borderColorPressed);
            TRY_GET_COLOR_ATTRIBUTE(jo, borderColorDisabled);
            borderColorTransparent = colorWithAlpha(borderColor, 0);

            TRY_GET_COLOR_ATTRIBUTE(jo, shadowColor1);
            TRY_GET_COLOR_ATTRIBUTE(jo, shadowColor2);
            TRY_GET_COLOR_ATTRIBUTE(jo, shadowColor3);
            shadowColorTransparent = colorWithAlpha(shadowColor1, 0);

            TRY_GET_INT_ATTRIBUTE(jo, fontSize);
            TRY_GET_INT_ATTRIBUTE(jo, fontSizeH1);
            TRY_GET_INT_ATTRIBUTE(jo, fontSizeH2);
            TRY_GET_INT_ATTRIBUTE(jo, fontSizeH3);
            TRY_GET_INT_ATTRIBUTE(jo, fontSizeH4);
            TRY_GET_INT_ATTRIBUTE(jo, fontSizeH5);
            TRY_GET_INT_ATTRIBUTE(jo, fontSizeS1);
            TRY_GET_INT_ATTRIBUTE(jo, animationDuration);
            TRY_GET_INT_ATTRIBUTE(jo, focusAnimationDuration);
            TRY_GET_INT_ATTRIBUTE(jo, sliderAnimationDuration);
            TRY_GET_DOUBLE_ATTRIBUTE(jo, borderRadius);
            TRY_GET_DOUBLE_ATTRIBUTE(jo, checkBoxBorderRadius);
            TRY_GET_DOUBLE_ATTRIBUTE(jo, menuItemBorderRadius);
            TRY_GET_DOUBLE_ATTRIBUTE(jo, menuBarItemBorderRadius);
            TRY_GET_INT_ATTRIBUTE(jo, borderWidth);
            TRY_GET_INT_ATTRIBUTE(jo, controlHeightLarge);
            TRY_GET_INT_ATTRIBUTE(jo, controlHeightMedium);
            TRY_GET_INT_ATTRIBUTE(jo, controlHeightSmall);
            TRY_GET_INT_ATTRIBUTE(jo, controlDefaultWidth);
            TRY_GET_INT_ATTRIBUTE(jo, dialMarkLength);
            TRY_GET_INT_ATTRIBUTE(jo, dialMarkThickness);
            TRY_GET_INT_ATTRIBUTE(jo, dialTickLength);
            TRY_GET_INT_ATTRIBUTE(jo, dialTickSpacing);
            TRY_GET_INT_ATTRIBUTE(jo, dialGrooveThickness);
            TRY_GET_INT_ATTRIBUTE(jo, focusBorderWidth);

            auto iconExtent = 16;
            TRY_GET_INT_ATTRIBUTE(jo, iconExtent);
            iconSize = QSize{iconExtent, iconExtent};
            iconSizeMedium = iconSize * 1.5;
            iconSizeLarge = iconSize * 2;
            iconSizeExtraSmall = iconSize * 0.75;

            TRY_GET_INT_ATTRIBUTE(jo, sliderTickSize);
            TRY_GET_INT_ATTRIBUTE(jo, sliderTickSpacing);
            TRY_GET_INT_ATTRIBUTE(jo, sliderTickThickness);
            TRY_GET_INT_ATTRIBUTE(jo, sliderGrooveHeight);
            TRY_GET_INT_ATTRIBUTE(jo, progressBarGrooveHeight);
            TRY_GET_INT_ATTRIBUTE(jo, spacing);
            TRY_GET_INT_ATTRIBUTE(jo, scrollBarThicknessFull);
            TRY_GET_INT_ATTRIBUTE(jo, scrollBarThicknessSmall);
            TRY_GET_INT_ATTRIBUTE(jo, scrollBarMargin);
            TRY_GET_INT_ATTRIBUTE(jo, tabBarPaddingTop);
            TRY_GET_INT_ATTRIBUTE(jo, tabBarTabMaxWidth);
            TRY_GET_INT_ATTRIBUTE(jo, tabBarTabMinWidth);

            tabBarTabMaxWidth = std::max(0, tabBarTabMaxWidth);
            tabBarTabMinWidth = std::max(0, tabBarTabMinWidth);
            if (tabBarTabMinWidth > tabBarTabMaxWidth) {
                std::swap(tabBarTabMinWidth, tabBarTabMaxWidth);
            }

            // Fonts.
            initializeFonts();
        }
    }
}

QJsonDocument Theme::toJson() const
{
    QJsonObject jo;

    // Metadata.
    QJsonObject joMeta;
    joMeta.insert("name", meta.name);
    joMeta.insert("version", meta.version);
    joMeta.insert("author", meta.author);
    jo.insert("meta", joMeta);

    // Colors.
    SET_COLOR(jo, backgroundColorMain1);
    SET_COLOR(jo, backgroundColorMain2);
    SET_COLOR(jo, backgroundColorMain3);
    SET_COLOR(jo, backgroundColorMain4);

    SET_COLOR(jo, neutralColor);
    SET_COLOR(jo, neutralColorHovered);
    SET_COLOR(jo, neutralColorPressed);
    SET_COLOR(jo, neutralColorDisabled);

    SET_COLOR(jo, focusColor);

    SET_COLOR(jo, primaryColor);
    SET_COLOR(jo, primaryColorHovered);
    SET_COLOR(jo, primaryColorPressed);
    SET_COLOR(jo, primaryColorDisabled);

    SET_COLOR(jo, primaryColorForeground);
    SET_COLOR(jo, primaryColorForegroundHovered);
    SET_COLOR(jo, primaryColorForegroundPressed);
    SET_COLOR(jo, primaryColorForegroundDisabled);

    SET_COLOR(jo, primaryAlternativeColor);
    SET_COLOR(jo, primaryAlternativeColorHovered);
    SET_COLOR(jo, primaryAlternativeColorPressed);
    SET_COLOR(jo, primaryAlternativeColorDisabled);

    SET_COLOR(jo, secondaryColor);
    SET_COLOR(jo, secondaryColorHovered);
    SET_COLOR(jo, secondaryColorPressed);
    SET_COLOR(jo, secondaryColorDisabled);

    SET_COLOR(jo, secondaryColorForeground);
    SET_COLOR(jo, secondaryColorForegroundHovered);
    SET_COLOR(jo, secondaryColorForegroundPressed);
    SET_COLOR(jo, secondaryColorForegroundDisabled);

    SET_COLOR(jo, secondaryAlternativeColor);
    SET_COLOR(jo, secondaryAlternativeColorHovered);
    SET_COLOR(jo, secondaryAlternativeColorPressed);
    SET_COLOR(jo, secondaryAlternativeColorDisabled);

    SET_COLOR(jo, statusColorSuccess);
    SET_COLOR(jo, statusColorSuccessHovered);
    SET_COLOR(jo, statusColorSuccessPressed);
    SET_COLOR(jo, statusColorSuccessDisabled);

    SET_COLOR(jo, statusColorInfo);
    SET_COLOR(jo, statusColorInfoHovered);
    SET_COLOR(jo, statusColorInfoPressed);
    SET_COLOR(jo, statusColorInfoDisabled);

    SET_COLOR(jo, statusColorWarning);
    SET_COLOR(jo, statusColorWarningHovered);
    SET_COLOR(jo, statusColorWarningPressed);
    SET_COLOR(jo, statusColorWarningDisabled);

    SET_COLOR(jo, statusColorError);
    SET_COLOR(jo, statusColorErrorHovered);
    SET_COLOR(jo, statusColorErrorPressed);
    SET_COLOR(jo, statusColorErrorDisabled);

    SET_COLOR(jo, shadowColor1);
    SET_COLOR(jo, shadowColor2);
    SET_COLOR(jo, shadowColor3);

    SET_COLOR(jo, borderColor);
    SET_COLOR(jo, borderColorHovered);
    SET_COLOR(jo, borderColorPressed);
    SET_COLOR(jo, borderColorDisabled);

    SET_COLOR(jo, semiTransparentColor1);
    SET_COLOR(jo, semiTransparentColor2);
    SET_COLOR(jo, semiTransparentColor3);
    SET_COLOR(jo, semiTransparentColor4);

    SET_INT(jo, fontSize);
    SET_INT(jo, fontSizeMonospace);
    SET_INT(jo, fontSizeH1);
    SET_INT(jo, fontSizeH2);
    SET_INT(jo, fontSizeH3);
    SET_INT(jo, fontSizeH4);
    SET_INT(jo, fontSizeH5);
    SET_INT(jo, fontSizeS1);
    SET_INT(jo, animationDuration);
    SET_INT(jo, focusAnimationDuration);
    SET_INT(jo, sliderAnimationDuration);
    SET_DOUBLE(jo, borderRadius);
    SET_DOUBLE(jo, checkBoxBorderRadius);
    SET_DOUBLE(jo, menuBarItemBorderRadius);
    SET_INT(jo, borderWidth);
    SET_INT(jo, controlHeightLarge);
    SET_INT(jo, controlHeightMedium);
    SET_INT(jo, controlHeightSmall);
    SET_INT(jo, controlDefaultWidth);
    SET_INT(jo, dialMarkLength);
    SET_INT(jo, dialMarkThickness);
    SET_INT(jo, dialTickLength);
    SET_INT(jo, dialTickSpacing);
    SET_INT(jo, dialGrooveThickness);
    SET_INT(jo, focusBorderWidth);
    setInt(jo, "iconSize", iconSize.width());
    setInt(jo, "iconSizeMedium", iconSizeMedium.width());
    setInt(jo, "iconSizeLarge", iconSizeLarge.width());
    setInt(jo, "iconSizeExtraSmall", iconSizeExtraSmall.width());
    SET_INT(jo, sliderTickSize);
    SET_INT(jo, sliderTickSpacing);
    SET_INT(jo, sliderTickThickness);
    SET_INT(jo, sliderGrooveHeight);
    SET_INT(jo, progressBarGrooveHeight);
    SET_INT(jo, spacing);
    SET_INT(jo, scrollBarThicknessFull);
    SET_INT(jo, scrollBarThicknessSmall);
    SET_INT(jo, scrollBarMargin);
    SET_INT(jo, tabBarPaddingTop);
    SET_INT(jo, tabBarTabMaxWidth);
    SET_INT(jo, tabBarTabMinWidth);

    QJsonDocument jsonDoc;
    jsonDoc.setObject(jo);
    return jsonDoc;
}

bool Theme::operator==(const Theme& other) const
{
    // All generated values are not used in this equality test, on purpose.
    // TODO: Remove this and instead use spaceship operator when C++20 is
    // available.
    // clang-format off
    return meta == other.meta
           && backgroundColorMain1 == other.backgroundColorMain1
           && backgroundColorMain2 == other.backgroundColorMain2
           && backgroundColorMain3 == other.backgroundColorMain3
           && backgroundColorMain4 == other.backgroundColorMain4

           && neutralColorDisabled == other.neutralColorDisabled
           && neutralColor == other.neutralColor
           && neutralColorHovered == other.neutralColorHovered
           && neutralColorPressed == other.neutralColorPressed

           && focusColor == other.focusColor

           && primaryColor == other.primaryColor
           && primaryColorHovered == other.primaryColorHovered
           && primaryColorPressed == other.primaryColorPressed
           && primaryColorDisabled == other.primaryColorDisabled

           && primaryColorForeground == other.primaryColorForeground
           && primaryColorForegroundHovered == other.primaryColorForegroundHovered
           && primaryColorForegroundPressed == other.primaryColorForegroundPressed
           && primaryColorForegroundDisabled == other.primaryColorForegroundDisabled

           && primaryAlternativeColor == other.primaryAlternativeColor
           && primaryAlternativeColorHovered == other.primaryAlternativeColorHovered
           && primaryAlternativeColorPressed == other.primaryAlternativeColorPressed
           && primaryAlternativeColorDisabled == other.primaryAlternativeColorDisabled

           && secondaryColor == other.secondaryColor
           && secondaryColorHovered == other.secondaryColorHovered
           && secondaryColorPressed == other.secondaryColorPressed
           && secondaryColorDisabled == other.secondaryColorDisabled

           && secondaryAlternativeColor == other.secondaryAlternativeColor
           && secondaryAlternativeColorHovered == other.secondaryAlternativeColorHovered
           && secondaryAlternativeColorPressed == other.secondaryAlternativeColorPressed
           && secondaryAlternativeColorDisabled == other.secondaryAlternativeColorDisabled

           && secondaryColorForeground == other.secondaryColorForeground
           && secondaryColorForegroundHovered == other.secondaryColorForegroundHovered
           && secondaryColorForegroundPressed == other.secondaryColorForegroundPressed
           && secondaryColorForegroundDisabled == other.secondaryColorForegroundDisabled

           && statusColorSuccess == other.statusColorSuccess
           && statusColorSuccessHovered == other.statusColorSuccessHovered
           && statusColorSuccessPressed == other.statusColorSuccessPressed
           && statusColorSuccessDisabled == other.statusColorSuccessDisabled
           && statusColorInfo == other.statusColorInfo
           && statusColorInfoHovered == other.statusColorInfoHovered
           && statusColorInfoPressed == other.statusColorInfoPressed
           && statusColorInfoDisabled == other.statusColorInfoDisabled
           && statusColorWarning == other.statusColorWarning
           && statusColorWarningHovered == other.statusColorWarningHovered
           && statusColorWarningPressed == other.statusColorWarningPressed
           && statusColorWarningDisabled == other.statusColorWarningDisabled
           && statusColorError == other.statusColorError
           && statusColorErrorHovered == other.statusColorErrorHovered
           && statusColorErrorPressed == other.statusColorErrorPressed
           && statusColorErrorDisabled == other.statusColorErrorDisabled
           && statusColorForeground == other.statusColorForeground
           && statusColorForegroundHovered == other.statusColorForegroundHovered
           && statusColorForegroundPressed == other.statusColorForegroundPressed
           && statusColorForegroundDisabled == other.statusColorForegroundDisabled

           && shadowColor1 == other.shadowColor1
           && shadowColor2 == other.shadowColor2
           && shadowColor3 == other.shadowColor3

           && borderColor == other.borderColor
           && borderColorHovered == other.borderColorHovered
           && borderColorPressed == other.borderColorPressed
           && borderColorDisabled == other.borderColorDisabled

           && animationDuration == other.animationDuration
           && focusAnimationDuration == other.focusAnimationDuration
           && sliderAnimationDuration == other.sliderAnimationDuration
           && borderRadius == other.borderRadius
           && checkBoxBorderRadius == other.checkBoxBorderRadius
           && menuItemBorderRadius == other.menuItemBorderRadius
           && menuBarItemBorderRadius == other.menuBarItemBorderRadius
           && borderWidth == other.borderWidth
           && controlHeightLarge == other.controlHeightLarge
           && controlHeightMedium == other.controlHeightMedium
           && controlHeightSmall == other.controlHeightSmall
           && controlDefaultWidth == other.controlDefaultWidth
           && dialMarkLength == other.dialMarkLength
           && dialMarkThickness == other.dialMarkThickness
           && dialTickLength == other.dialTickLength
           && dialTickSpacing == other.dialTickSpacing
           && dialGrooveThickness == other.dialGrooveThickness
           && focusBorderWidth == other.focusBorderWidth
           && iconSize == other.iconSize
           && iconSizeMedium == other.iconSizeMedium
           && iconSizeLarge == other.iconSizeLarge
           && iconSizeExtraSmall == other.iconSizeExtraSmall
           && sliderTickSize == other.sliderTickSize
           && sliderTickThickness == other.sliderTickThickness
           && sliderGrooveHeight == other.sliderGrooveHeight
           && progressBarGrooveHeight == other.progressBarGrooveHeight
           && spacing == other.spacing
           && scrollBarThicknessFull == other.scrollBarThicknessFull
           && scrollBarThicknessSmall == other.scrollBarThicknessSmall
           && scrollBarMargin == other.scrollBarMargin
           && tabBarPaddingTop == other.tabBarPaddingTop;
    // clang-format on
}

bool Theme::operator!=(const Theme& other) const { return !(*this == other); }

} // namespace thoht

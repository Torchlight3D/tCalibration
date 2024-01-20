#include "qcolorutils.h"

#include <QRegularExpression>
#include <QVariant>

#include "qstringutils.h"

namespace tl {

namespace {

constexpr int kHexBase = 16;

std::optional<QColor> tryGetColorFromRGBAString(const QString& str)
{
    // Try parse RGB ("rgb(RRR,GGG,BBB)").
    static const QRegularExpression rgbRegex{
        u"^ *rgb *\\( *(\\d{1,3}) *, *(\\d{1,3}) *, *(\\d{1,3}) *\\) *$"_s,
        QRegularExpression::CaseInsensitiveOption};

    const auto rgbMatch = rgbRegex.match(str);
    if (rgbMatch.hasMatch()) {
        const auto r = rgbMatch.captured(1).toInt();
        const auto g = rgbMatch.captured(2).toInt();
        const auto b = rgbMatch.captured(3).toInt();
        return QColor{r, g, b};
    }

    // Try parse RGBA ("rgba(RRR,GGG,BBB,AAA)").
    static const QRegularExpression rgbaRegex{
        u"^ *rgba *\\( *(\\d{1,3}) *, *(\\d{1,3}) *, *(\\d{1,3}) "
        "*, *(\\d{1,3})*\\) *$"_s,
        QRegularExpression::CaseInsensitiveOption};

    const auto rgbaMatch = rgbaRegex.match(str);
    if (rgbaMatch.hasMatch()) {
        const auto r = rgbaMatch.captured(1).toInt();
        const auto g = rgbaMatch.captured(2).toInt();
        const auto b = rgbaMatch.captured(3).toInt();
        const auto a = rgbaMatch.captured(4).toInt();
        return QColor{r, g, b, a};
    }

    return {};
}

std::optional<QColor> tryGetColorFromHexaString(const QString& str)
{
    constexpr auto kRgbLength = 3 * 2 + 1;
    constexpr auto kRgbaLength = 4 * 2 + 1;

    const auto len = str.length();
    if (str.startsWith('#') && (len == kRgbLength || len == kRgbaLength)) {
        auto success{false};

        const auto r_str = str.mid(1, 2);
        const auto g_str = str.mid(3, 2);
        const auto b_str = str.mid(5, 2);
        const auto a_str = str.mid(7, 2);

        const auto r = r_str.toInt(&success, kHexBase);
        if (success) {
            const auto g = g_str.toInt(&success, kHexBase);
            if (success) {
                const auto b = b_str.toInt(&success, kHexBase);
                if (success) {
                    QColor result{r, g, b};

                    const auto a = a_str.toInt(&success, kHexBase);
                    if (success) {
                        result.setAlpha(a);
                    }

                    return result;
                }
            }
        }
    }

    return {};
}

} // namespace

QColor colorWithAlpha(const QColor& color, int alpha)
{
    alpha = std::min(255, std::max(0, alpha));
    auto result = QColor{color};
    result.setAlpha(alpha);
    return result;
}

QColor getColorSourceOver(const QColor& bg, const QColor& fg)
{
    // Premultiply.
    const auto bgAlpha = bg.alphaF();
    const auto bgRed = bg.redF() * bgAlpha;
    const auto bgGreen = bg.greenF() * bgAlpha;
    const auto bgBlue = bg.blueF() * bgAlpha;

    const auto fgAlpha = fg.alphaF();
    const auto fgRed = fg.redF() * fgAlpha;
    const auto fgGreen = fg.greenF() * fgAlpha;
    const auto fgBlue = fg.blueF() * fgAlpha;
    const auto fgAlphaInv = 1. - fg.alphaF();

    const auto finalAlpha = bgAlpha + fgAlpha - bgAlpha * fgAlpha;
    const auto finalRed = fgRed + bgRed * fgAlphaInv;
    const auto finalGreen = fgGreen + bgGreen * fgAlphaInv;
    const auto finalBlue = fgBlue + bgBlue * fgAlphaInv;

    const auto finalRGBA = qRgba(int(finalRed * 255), int(finalGreen * 255),
                                 int(finalBlue * 255), int(finalAlpha * 255));
    auto finalColor = QColor::fromRgba(finalRGBA);

    return finalColor;
}

std::optional<QColor> tryGetColorFromVariantList(const QVariantList& variants)
{
    const auto size = variants.size();
    if (size != 3 && size != 4) {
        return {};
    }

    int r{0}, g{0}, b{0};
    const auto& var_r = variants[0];
    if (var_r.isValid() && var_r.canConvert<int>()) {
        r = var_r.toInt();
    }
    const auto& var_g = variants[1];
    if (var_g.isValid() && var_g.canConvert<int>()) {
        g = var_g.toInt();
    }
    const auto& var_b = variants[2];
    if (var_b.isValid() && var_b.canConvert<int>()) {
        b = var_b.toInt();
    }

    int a{255};
    if (size == 4) {
        const auto& var_a = variants[3];
        if (var_a.isValid() && var_a.canConvert<int>()) {
            a = var_a.toInt();
        }
    }

    return QColor{r, g, b, a};
}

std::optional<QColor> tryGetColorFromVariant(const QVariant& variant)
{
    const auto type = variant.typeId();

    // Channel list ([RRR, GGG, BBB, AAA]).
    if (type == QMetaType::Type::QVariantList) {
        const auto color = tryGetColorFromVariantList(variant.toList());
        if (color.has_value()) {
            return color.value();
        }
    }
    // Various ways to write color as a string.
    else if (type == QMetaType::Type::QString) {
        const auto variantString = variant.toString();

        // Check if color is written as hexadecimal.
        const auto hexaColor = tryGetColorFromHexaString(variantString);
        if (hexaColor.has_value()) {
            return hexaColor.value();
        }

        // Check if color is written as RGB(A).
        const auto rgbaColor = tryGetColorFromRGBAString(variantString);
        if (rgbaColor.has_value()) {
            return rgbaColor.value();
        }
    }

    return {};
}

QString toHexRGB(const QColor& color)
{
    return u"#%1%2%3"_s.arg(QString::number(color.red(), kHexBase), 2, '0')
        .arg(QString::number(color.green(), kHexBase), 2, '0')
        .arg(QString::number(color.blue(), kHexBase), 2, '0');
}

QString toHexRGBA(const QColor& color)
{
    return u"#%1%2%3%4"_s.arg(QString::number(color.red(), kHexBase), 2, '0')
        .arg(QString::number(color.green(), kHexBase), 2, '0')
        .arg(QString::number(color.blue(), kHexBase), 2, '0')
        .arg(QString::number(color.alpha(), kHexBase), 2, '0');
}

} // namespace tl

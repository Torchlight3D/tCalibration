#include "TextLabel.h"

#include <QApplication>

#include "TheStyle.h"

namespace thoht {

TextLabel::TextLabel(const QString& text, TextRole role, QWidget* parent)
    : QLabel(text, parent), m_role(role)
{
    updatePaletteFromTheme();
    qApp->installEventFilter(this);
}

TextLabel::~TextLabel() = default;

TextRole TextLabel::role() const { return m_role; }

void TextLabel::setRole(TextRole role)
{
    if (role != m_role) {
        m_role = role;
        // Change text font, size and color.
        updatePaletteFromTheme();
        emit roleChanged();
    }
}

bool TextLabel::event(QEvent* e)
{
    // Ensure the label repaints when the theme changes.
    if (e->type() == QEvent::Type::PaletteChange && !m_changingPaletteFlag) {
        m_changingPaletteFlag = true;
        updatePaletteFromTheme();
        m_changingPaletteFlag = false;
    }
    return QLabel::event(e);
}

bool TextLabel::eventFilter(QObject* obj, QEvent* e)
{
    if (e->type() == QEvent::Type::PaletteChange) {
        updatePaletteFromTheme();
    }
    return QLabel::eventFilter(obj, e);
}

void TextLabel::updatePaletteFromTheme()
{
    if (const auto* theStyle = qobject_cast<QlementineStyle*>(style())) {
        const auto& font = theStyle->fontForTextRole(m_role);
        const auto palette = theStyle->paletteForTextRole(m_role);
        setFont(font);
        setPalette(palette);
        updateGeometry();
        update(contentsRect());
    }
}

} // namespace thoht

#include "moc_TextLabel.cpp"

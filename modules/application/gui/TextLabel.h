#pragma once

#include <QLabel>
#include "TheStyleTypes.h"

namespace thoht {

/// A QLabel that handles automatic styling for different text roles (titles,
/// normal text, ect.) like h1,h2,p in HTML.
class TextLabel : public QLabel
{
    Q_OBJECT
    Q_PROPERTY(thoht::TextRole role READ role WRITE setRole NOTIFY roleChanged)

public:
    explicit TextLabel(const QString& text, TextRole role = TextRole::Default,
                   QWidget* parent = nullptr);
    ~TextLabel();

    TextRole role() const;
    Q_SLOT void setRole(TextRole role);
    Q_SIGNAL void roleChanged();

protected:
    bool event(QEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void updatePaletteFromTheme();

private:
    TextRole m_role{TextRole::Default};
    bool m_changingPaletteFlag{false};
};

} // namespace thoht

#pragma once

#include <QScrollArea>

#include "qtcoreutils.h"

namespace tl {

class QtListWidgetItem : public QWidget
{
    Q_OBJECT

public:
    explicit QtListWidgetItem(QWidget* parent = nullptr);
    ~QtListWidgetItem();

    bool isSelected() const;
    void select(bool on);

protected:
    void enterEvent(QEnterEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    void highlightBackground(bool on);

private:
    bool m_selected{false};
};

class QtListWidgetPrivate;
class QtListWidget : public QScrollArea
{
    Q_OBJECT

    using Base = QScrollArea;

public:
    explicit QtListWidget(bool stretch = true, QWidget* parent = nullptr);
    ~QtListWidget();

    /// Properties
    QString placeholderText() const;
    void setPlaceholderText(const QString& text);

    bool selectable() const;
    void setSelectable(bool enable);

    /// Data
    // Take ownership of the item
    QtListWidgetItem* addItem(QtListWidgetItem* item, int index = -1);
    void removeItem(QtListWidgetItem* item);
    QList<QtListWidgetItem*> allItems(bool selectedOnly = false) const;
    inline auto selectedItems() const { return allItems(true); }
    inline auto itemCount() const { return allItems().size(); }
    void clear();

signals:
    // TODO: Add more infos in parameters
    void itemChanged();

protected:
    // Handle selection
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void childEvent(QChildEvent* event) override;

private:
    Q_DECLARE_PIMPL(QtListWidget);
};

} // namespace tl

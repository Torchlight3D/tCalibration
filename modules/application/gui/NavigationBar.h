#pragma once

#include <QWidget>

class NavigationBar : public QWidget
{
    Q_OBJECT

public:
    explicit NavigationBar(QWidget* parent = nullptr);
    ~NavigationBar();

    /// Properties
    void setBackgroundColor(const QColor& color);
    void setSelectedColor(const QColor& color);
    void setRowHeight(int height);
    void setWidth(int width);

    /// Data
    int addItem(const QString& text, const QString& toolTips = {});
    int insertItem(int index, const QString& text,
                   const QString& toolTips = {});
    void deleteItem(int index);

    void setItemText(int index, const QString& text);
    void setItemToolTip(int index, const QString& toolTip);

    int currentIndex() const;
    int count() const;

signals:
    void currentTabChanged(int index);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    QStringList m_tabNames;
    QStringList m_tabToolTips;
    QColor m_bgColor;
    QColor m_selectedColor;
    int m_rowHeight;
    int m_currentIndex;
};

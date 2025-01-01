#pragma once

#include <QTabWidget>

namespace tl {

class QtTabWidget : public QTabWidget
{
    Q_OBJECT

public:
    explicit QtTabWidget(QWidget *parent = nullptr);
};

} // namespace tl

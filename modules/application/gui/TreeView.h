#pragma once

#include <QTreeView>

namespace tl {

/// Brief:
/// QTreeView with some custom initializations
///
/// Explanation:
///
class TreeView : public QTreeView
{
    Q_OBJECT
public:
    explicit TreeView(QWidget* parent = nullptr);
};

} // namespace tl

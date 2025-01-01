#pragma once

#include <QAbstractItemModel>

namespace tl {

class QtJsonTreeItem;

class QtJsonTreeModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    explicit QtJsonTreeModel(QObject *parent = nullptr);
    ~QtJsonTreeModel();

    bool loadFromFile(const QString &filename);
    bool loadFromJson(const QByteArray &json);

    enum Section
    {
        Key,
        Value,

        Count,
    };

    QModelIndex index(int row, int column,
                      const QModelIndex &parent = {}) const override;
    QModelIndex parent(const QModelIndex &child) const override;

    int rowCount(const QModelIndex &parent = {}) const override;
    int columnCount(const QModelIndex &parent = {}) const override;

    QVariant data(const QModelIndex &index, int role) const override;
    bool setData(const QModelIndex &index, const QVariant &value,
                 int role = Qt::EditRole) override;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    void clear();

    QByteArray toJson(bool compact = false);

private:
    QScopedPointer<QtJsonTreeItem> m_root;
};

} // namespace tl

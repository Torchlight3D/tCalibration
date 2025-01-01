#include "qtjsontreemodel.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace tl {

using namespace Qt::Literals::StringLiterals;

namespace {

} // namespace

///------- QtJsonTreeItem starts from here
class QtJsonTreeItem
{
public:
    explicit QtJsonTreeItem(QtJsonTreeItem *parent = nullptr);
    ~QtJsonTreeItem();

    void appendChild(QtJsonTreeItem *item);
    const QtJsonTreeItem *child(int row) const;
    const QtJsonTreeItem *parent() const;
    int childCount() const;

    int row() const;

    QString key() const;
    void setKey(const QString &key);

    QVariant value() const;
    void setValue(const QVariant &value);

    QJsonValue::Type type() const;
    void setType(const QJsonValue::Type &type);

    QJsonValue toJson() const;

    static QtJsonTreeItem *load(const QJsonValue &jv,
                                QtJsonTreeItem *parent = nullptr);

private:
    QString m_key;
    QVariant m_val;
    QJsonValue::Type m_type;
    QList<QtJsonTreeItem *> m_children;
    QtJsonTreeItem *m_parent{nullptr};
};

QtJsonTreeItem::QtJsonTreeItem(QtJsonTreeItem *parent) : m_parent(parent) {}

QtJsonTreeItem::~QtJsonTreeItem() { qDeleteAll(m_children); }

void QtJsonTreeItem::appendChild(QtJsonTreeItem *item)
{
    m_children.append(item);
}

const QtJsonTreeItem *QtJsonTreeItem::child(int row) const
{
    return m_children.value(row);
}

const QtJsonTreeItem *QtJsonTreeItem::parent() const { return m_parent; }

int QtJsonTreeItem::childCount() const { return m_children.count(); }

int QtJsonTreeItem::row() const
{
    if (!m_parent) {
        return 0;
    }

    return m_parent->m_children.indexOf(this);
}

QString QtJsonTreeItem::key() const { return m_key; }

void QtJsonTreeItem::setKey(const QString &key) { m_key = key; }

QVariant QtJsonTreeItem::value() const { return m_val; }

void QtJsonTreeItem::setValue(const QVariant &value) { m_val = value; }

QJsonValue::Type QtJsonTreeItem::type() const { return m_type; }

void QtJsonTreeItem::setType(const QJsonValue::Type &type) { m_type = type; }

QJsonValue QtJsonTreeItem::toJson() const
{
    switch (m_type) {
        case QJsonValue::Object: {
            QJsonObject jo;
            for (auto i{0}; i < childCount(); ++i) {
                auto child = this->child(i);
                jo.insert(child->key(), child->toJson());
            }
            return jo;
        }
        case QJsonValue::Array: {
            QJsonArray ja;
            for (auto i{0}; i < childCount(); ++i) {
                ja.append(child(i)->toJson());
            }
            return ja;
        }
        default: {
            return QJsonValue::fromVariant(m_val);
        }
    }
}

QtJsonTreeItem *QtJsonTreeItem::load(const QJsonValue &jv,
                                     QtJsonTreeItem *parent)
{
    auto *root = new QtJsonTreeItem(parent);
    root->setKey("root");
    root->setType(jv.type());

    if (jv.isObject()) {
        const auto jo = jv.toObject();
        const auto keys = jo.keys();
        for (const auto &key : keys) {
            const auto _jv = jo.value(key);
            auto child = load(_jv, root);
            child->setKey(key);
            child->setType(_jv.type());
            root->appendChild(child);
        }
    }
    else if (jv.isArray()) {
        const auto ja = jv.toArray();

        auto index{0};
        for (const auto &_jv : ja) {
            auto child = load(_jv, root);
            child->setKey(QString::number(index));
            child->setType(_jv.type());
            root->appendChild(child);
            ++index;
        }
    }
    else {
        root->setValue(jv.toVariant());
    }

    return root;
}

///------- QtJsonTreeModel starts from here
QtJsonTreeModel::QtJsonTreeModel(QObject *parent)
    : QAbstractItemModel(parent), m_root{new QtJsonTreeItem}
{
}

QtJsonTreeModel::~QtJsonTreeModel() = default;

bool QtJsonTreeModel::loadFromFile(const QString &filename)
{
    QFile file{filename};
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }

    return loadFromJson(file.readAll());
}

bool QtJsonTreeModel::loadFromJson(const QByteArray &json)
{
    QJsonParseError err;
    const auto jd = QJsonDocument::fromJson(json, &err);

    if (jd.isNull()) {
        qWarning() << u"Failed to load JSON into QtJsonTreeModel: "_s
                   << err.errorString();
        return false;
    }

    beginResetModel();
    if (jd.isArray()) {
        m_root.reset(QtJsonTreeItem::load(jd.array()));
    }
    else {
        m_root.reset(QtJsonTreeItem::load(jd.object()));
    }
    endResetModel();

    return true;
}

QModelIndex QtJsonTreeModel::index(int row, int column,
                                   const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent)) {
        return {};
    }

    auto parentItem =
        parent.isValid()
            ? static_cast<const QtJsonTreeItem *>(parent.internalPointer())
            : m_root.get();

    if (auto childItem = parentItem->child(row)) {
        return createIndex(row, column, childItem);
    }

    return {};
}

QModelIndex QtJsonTreeModel::parent(const QModelIndex &child) const
{
    if (!child.isValid()) {
        return {};
    }

    auto childItem =
        static_cast<const QtJsonTreeItem *>(child.internalPointer());
    auto parentItem = childItem->parent();

    if (parentItem == m_root.get()) {
        return {};
    }

    return createIndex(parentItem->row(), 0, parentItem);
}

int QtJsonTreeModel::rowCount(const QModelIndex &parent) const
{
    if (parent.column() > 0) {
        return 0;
    }

    auto parentItem =
        parent.isValid()
            ? static_cast<const QtJsonTreeItem *>(parent.internalPointer())
            : m_root.get();

    return parentItem->childCount();
}

int QtJsonTreeModel::columnCount(const QModelIndex & /*parent*/) const
{
    return Section::Count;
}

QVariant QtJsonTreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid()) {
        return {};
    }

    auto item = static_cast<const QtJsonTreeItem *>(index.internalPointer());

    switch (role) {
        case Qt::DisplayRole: {
            switch (index.column()) {
                case Section::Key:
                    return item->key();
                case Section::Value:
                    return item->value();
                default:
                    break;
            }
        } break;
        case Qt::EditRole: {
            switch (index.column()) {
                case Section::Value:
                    return item->value();
                default:
                    break;
            }
        } break;
        default:
            break;
    }

    return {};
}

bool QtJsonTreeModel::setData(const QModelIndex &index, const QVariant &value,
                              int role)
{
    if (role != Qt::EditRole) {
        return false;
    }

    if (index.column() == Section::Value) {
        auto item = static_cast<QtJsonTreeItem *>(index.internalPointer());
        item->setValue(value);

        emit dataChanged(index, index, {Qt::EditRole});
        return true;
    }

    return false;
}

QVariant QtJsonTreeModel::headerData(int section, Qt::Orientation orientation,
                                     int role) const
{
    if (role != Qt::DisplayRole) {
        return {};
    }

    static const std::array kHeaders{tr("Key"), tr("Value")};
    if (orientation == Qt::Horizontal) {
        return kHeaders.at(section);
    }

    return {};
}

Qt::ItemFlags QtJsonTreeModel::flags(const QModelIndex &index) const
{
    const auto flags = QAbstractItemModel::flags(index);

    auto item = static_cast<const QtJsonTreeItem *>(index.internalPointer());

    auto isArray = QJsonValue::Array == item->type();
    auto isObject = QJsonValue::Object == item->type();

    if ((index.column() == Section::Value) && !(isArray || isObject)) {
        return Qt::ItemIsEditable | flags;
    }

    return flags;
}

void QtJsonTreeModel::clear()
{
    //
}

QByteArray QtJsonTreeModel::toJson(bool compact)
{
    const auto jv = m_root->toJson();
    const auto format =
        compact ? QJsonDocument::Compact : QJsonDocument::Indented;
    switch (jv.type()) {
        case QJsonValue::Array:
            return QJsonDocument{jv.toArray()}.toJson(format);
        case QJsonValue::Object:
            return QJsonDocument{jv.toObject()}.toJson(format);
        case QJsonValue::Null:
            return {};
        case QJsonValue::Bool:
        case QJsonValue::Double:
        case QJsonValue::String:
            return QJsonDocument{QJsonObject{{m_root->key(), jv}}}.toJson(
                format);
        case QJsonValue::Undefined:
        default:
            break;
    }

    return {};
}

} // namespace tl

#include "moc_qtjsontreemodel.cpp"

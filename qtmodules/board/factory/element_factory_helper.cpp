#include "element_factory_helper.h"

#include "../../ui/qpluginfactory.h"
#include "ElementInterface.h"

namespace thoht {

Q_GLOBAL_PLUGIN_FACTORY(ElementInterface, "element", _loader)

BoardElement *ElementFactory::createElement(const QString &name)
{
    BoardElement *instance = nullptr;

    if (valid(name)) {
        auto plugin = _loader->plugin(name);
        if (plugin) {
            instance = plugin->createElement();
            instance->setName(name);
            instance->setType(elementType(name));
            instance->setTimeSeriesSize(timeseriesSize(name));
            instance->setTimeSeriesType(timeseriesType(name));
        }
    }

    return instance;
}

QString ElementFactory::description(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto descr = meta[u"Description"_qs].toString();
    return descr;
}

QString ElementFactory::tooltip(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto tooltip = meta[u"ToolTip"_qs].toString();
    return tooltip;
}

QString ElementFactory::category(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto tooltip = meta[u"Category"_qs].toString();
    return tooltip;
}

QPixmap ElementFactory::iconPixmap(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto iconPath = meta[u"IconPath"_qs].toString();

    QPixmap pix;
    auto plugin = _loader->plugin(name);
    if (plugin)
        pix = plugin->iconPixmap(iconPath).scaled(25, 25, Qt::KeepAspectRatio);
    return pix;
}

QString ElementFactory::iconPath(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto iconPath = meta[u"IconPath"_qs].toString();

    return iconPath;
}

BoardElement::ElementType ElementFactory::elementType(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto typeInt = meta[u"Type"_qs].toInt();
    return BoardElement::ElementType(typeInt);
}

BoardElement::TimeSeriesType ElementFactory::timeseriesType(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto typeInt = meta[u"TimeSeriesType"_qs].toInt();
    return BoardElement::TimeSeriesType(typeInt);
}

BoardElement::TimeSeriesSize ElementFactory::timeseriesSize(const QString &name)
{
    auto meta = _loader->metaData(name);
    auto typeInt = meta[u"TimeSeriesSize"_qs].toInt();
    return BoardElement::TimeSeriesSize(typeInt);
}

bool ElementFactory::valid(const QString &name)
{
    auto meta = _loader->metaData(name);
    if (meta.contains(u"Validity"_qs))
        return meta[u"Validity"_qs].toBool();
    return true;
}

QStringList ElementFactory::pluginslist() const { return _loader->allKeys(); }

} // namespace thoht

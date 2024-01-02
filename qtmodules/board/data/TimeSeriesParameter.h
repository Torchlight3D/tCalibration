#pragma once

#include <QString>

namespace thoht {

class TimeSeriesParameter
{
public:
    TimeSeriesParameter();

    void setParameterId(quint32 id);
    quint32 parameterId() const;

    void setName(const QString &name);
    QString name() const;

    void setSourceName(const QString &sourceName);
    QString sourceName() const;

    void setUnit(const QString &unit);
    QString unit() const;

    void setDepth(int depth);
    int depth() const;

    void setSize(int size);
    int size() const;
    int capacity() const;

    void setRangeMax(double max);
    double rangeMax() const;
    void setRangeMin(double min);
    double rangeMin() const;
    bool isRangeValid();

    void setIndexStart(double start);
    double indexStart() const;
    void setIndexOffset(double offset);
    double indexOffset() const;

protected:
    quint32 mParameterId;
    QString mName;
    QString mUnit;
    QString mSourceName;

    int mDepth;
    int mSize;

    double mRangeMax;
    double mRangeMin;

    double mIndexStart;
    double mIndexOffset;
};

} // namespace thoht

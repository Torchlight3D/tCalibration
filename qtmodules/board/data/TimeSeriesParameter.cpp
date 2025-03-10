﻿#include "TimeSeriesParameter.h"

namespace tl {

TimeSeriesParameter::TimeSeriesParameter()
    : mParameterId(0),
      mDepth(0),
      mSize(0),
      mRangeMax(qQNaN()),
      mRangeMin(qQNaN()),
      mIndexStart(0),
      mIndexOffset(1)
{
}

void TimeSeriesParameter::setParameterId(quint32 id) { mParameterId = id; }

quint32 TimeSeriesParameter::parameterId() const { return mParameterId; }

QString TimeSeriesParameter::name() const { return mName; }

void TimeSeriesParameter::setName(const QString &name) { mName = name; }

QString TimeSeriesParameter::unit() const { return mUnit; }

void TimeSeriesParameter::setUnit(const QString &unit) { mUnit = unit; }

QString TimeSeriesParameter::sourceName() const { return mSourceName; }

void TimeSeriesParameter::setSourceName(const QString &sourceName)
{
    mSourceName = sourceName;
}

int TimeSeriesParameter::depth() const { return mDepth; }

void TimeSeriesParameter::setDepth(int depth) { mDepth = depth; }

int TimeSeriesParameter::size() const { return mSize; }

int TimeSeriesParameter::capacity() const { return mDepth * mSize; }

bool TimeSeriesParameter::isRangeValid()
{
    return (!qIsNaN(mRangeMax) && !qIsNaN(mRangeMin) && !qIsInf(mRangeMax) &&
            !qIsInf(mRangeMin));
}

void TimeSeriesParameter::setSize(int size) { mSize = size; }

double TimeSeriesParameter::rangeMax() const { return mRangeMax; }

void TimeSeriesParameter::setRangeMax(double rangeMax) { mRangeMax = rangeMax; }

double TimeSeriesParameter::rangeMin() const { return mRangeMin; }

void TimeSeriesParameter::setRangeMin(double rangeMin) { mRangeMin = rangeMin; }

double TimeSeriesParameter::indexStart() const { return mIndexStart; }

void TimeSeriesParameter::setIndexStart(double indexStart)
{
    mIndexStart = indexStart;
}

double TimeSeriesParameter::indexOffset() const { return mIndexOffset; }

void TimeSeriesParameter::setIndexOffset(double indexOffset)
{
    mIndexOffset = indexOffset;
}

} // namespace tl

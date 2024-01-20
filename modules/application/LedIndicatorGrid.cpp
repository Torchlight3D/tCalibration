#include "LedIndicatorGrid.h"

#include <QBoxLayout>

namespace tl {

class LedIndicatorGridPrivate
{
    Q_DEFINE_PIMPL(LedIndicatorGrid)

public:
    explicit LedIndicatorGridPrivate(LedIndicatorGrid* q);

    void init();

public:
    QList<LedIndicator*> m_leds;
    QGridLayout* m_layout;
};

LedIndicatorGridPrivate::LedIndicatorGridPrivate(LedIndicatorGrid* q)
    : q_ptr(q), m_layout(new QGridLayout(q))
{
}

void LedIndicatorGridPrivate::init()
{
    Q_Q(LedIndicatorGrid);
    m_layout->setContentsMargins({});
    m_layout->setSpacing(3);
}

LedIndicatorGrid::LedIndicatorGrid(QWidget* parent)
    : QWidget(parent), d_ptr(new LedIndicatorGridPrivate(this))
{
    d_ptr->init();
}

LedIndicatorGrid::~LedIndicatorGrid() = default;

int LedIndicatorGrid::ledCount() const { return 0; }

void LedIndicatorGrid::setLedCount(int count)
{
    Q_D(LedIndicatorGrid);
    if (count == d->m_leds.size()) {
        return;
    }

    clearLayout(d->m_layout);
    d->m_leds.clear();

    // FIXME: Avoid hardcoded
    constexpr int kRows = 2;
    constexpr int kCols = 2;
    for (int i{0}; i < count; ++i) {
        auto led = new LedIndicator(this);

        QSizePolicy sizePolicy{QSizePolicy::Fixed, QSizePolicy::Fixed};
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(led->sizePolicy().hasHeightForWidth());
        led->setSizePolicy(sizePolicy);

        constexpr int kLedEdge{25};
        constexpr QSize kSize{kLedEdge, kLedEdge};
        led->setMinimumSize(kSize);
        led->setMaximumSize(kSize);

        led->setShape(LedIndicator::RoundRect);

        d->m_layout->addWidget(led, i / kCols, i % kCols);
        d->m_leds.append(led);
    }
}

void LedIndicatorGrid::setLedStatus(int index, LedIndicator::Status status)
{
    Q_D(LedIndicatorGrid);
    if (index >= d->m_leds.size()) {
        return;
    }

    d->m_leds[index]->setStatus(status);
}

void LedIndicatorGrid::setLedColor(int index, const QColor& color)
{
    Q_D(LedIndicatorGrid);
    if (index >= d->m_leds.size()) {
        return;
    }

    d->m_leds[index]->setOnColor(color);
}

void LedIndicatorGrid::setLed(int index, LedIndicator::Status status,
                              const QColor& color)
{
    Q_D(LedIndicatorGrid);
    if (index >= d->m_leds.size()) {
        return;
    }

    d->m_leds[index]->setStatus(status);
    d->m_leds[index]->setOnColor(color);
}

void LedIndicatorGrid::setLedStatuses(
    const std::vector<LedIndicator::Status>& statuses)
{
    Q_D(LedIndicatorGrid);
    if (statuses.size() != d->m_leds.size()) {
        return;
    }

    for (size_t i{0}; i < statuses.size(); ++i) {
        d->m_leds[i]->setStatus(statuses[i]);
    }
}

void LedIndicatorGrid::setLedColors(const std::vector<QColor>& colors)
{
    Q_D(LedIndicatorGrid);
    if (colors.size() != d->m_leds.size()) {
        return;
    }

    for (size_t i{0}; i < colors.size(); ++i) {
        d->m_leds[i]->setOnColor(colors[i]);
    }
}

QColor LedIndicatorGrid::ledOnColor() const
{
    Q_D(const LedIndicatorGrid);
    if (d->m_leds.empty()) {
        return LedIndicator::defaultOnColor();
    }

    return d->m_leds.front()->onColor();
}

} // namespace tl

#include "moc_LedIndicatorGrid.cpp"

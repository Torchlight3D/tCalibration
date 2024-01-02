#pragma once

#include <QWidget>

namespace thoht {

class Scene;

class TestStereoModuleWidgetPrivate;
class TestStereoModuleWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TestStereoModuleWidget(QWidget* parent = nullptr);
    ~TestStereoModuleWidget();

signals:
    void requestShowScene(Scene* scene);

private:
    Q_DISABLE_COPY(TestStereoModuleWidget)
    Q_DECLARE_PRIVATE(TestStereoModuleWidget)
    const QScopedPointer<TestStereoModuleWidgetPrivate> d_ptr;
};

} // namespace thoht

#include "GlobalConfigs.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "ProductInfo.h"

namespace tl {

namespace key {
constexpr char kVersion[]{"version"};

// App
constexpr char kAppConfigs[]{"app"};
constexpr char kHomePage[]{"home_page"};
constexpr char kDevMode[]{"dev_mode"};
constexpr char kSNPattern[]{"sn_pattern"};

/// V1 Configs
// Product
constexpr char kProductName[]{"product_name"};
constexpr char kProductConfigGroup[]{"product_config_group"};
constexpr char kLuba2[]{"Luba2"};
constexpr char kYuka[]{"Yuka"};
// Calibration
constexpr char kCalibTask[]{"calib_task"};
constexpr char kCalib[]{"calibration"};
constexpr char kCalibReference[]{"calib_reference"};
// Verification
constexpr char kVerify[]{"verification"};

/// V2 Configs
constexpr char kProducts[]{"products"};
constexpr char kMonoCalibration[]{"mono_camera_calibration"};
constexpr char kMonoVerification[]{"mono_camera_verification"};
constexpr char kStereoCalibrationV2[]{"stereo_camera_calibration"};
constexpr char kStereoVerificationV2[]{"stereo_camera_verification"};
} // namespace key

namespace {
constexpr char kUnknown[]{"unknown"};

bool loadQJsonObjectToString(const QJsonObject& jo, std::string& str)
{
    if (auto _str =
            QJsonDocument{jo}.toJson(QJsonDocument::Compact).toStdString();
        _str.empty()) {
        return false;
    }
    else {
        str.swap(_str);
        return true;
    }
}

} // namespace

///------- AppConfigs::Impl starts from here
class GlobalConfigs::Impl
{
public:
    Impl();

    bool setFromJson(const QJsonObject& jo);
    bool setMainWindowConfigsFromJson(const QJsonObject& jo);

public:
    std::string _version{kUnknown};
    MainWindow::Configs _mainWindow{};

    // V1
    std::string _prodConfigs{};
    std::string _calibTaskConfigs{};
    std::string _calibConfigs{};
    std::string _calibReference{};
    std::string _verifyConfigs{};

    // V2
    std::string _monoCalibration{};
    std::string _monoVerification{};
    std::string _stereoCalibration{};
    std::string _stereoVerification{};
};

GlobalConfigs::Impl::Impl() {}

bool GlobalConfigs::Impl::setFromJson(const QJsonObject& j)
{
    _version = j.value(key::kVersion).toString(kUnknown).toStdString();

    bool success{true};
    if (const auto jo = j.value(key::kAppConfigs).toObject(); jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find app configs.";
        return false;
    }
    else {
        success &= setMainWindowConfigsFromJson(jo);
    }

    // Load supported products V1.
    // Load all supported products first, then select the one to use.
    if (const auto joProducts = j.value(key::kProductConfigGroup).toObject();
        joProducts.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find product configs.";
        return false;
    }
    else if (const auto jo =
                 joProducts
                     .value(j.value(key::kProductName).toString(key::kLuba2))
                     .toObject();
             jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find supported product configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _prodConfigs)) {
        return false;
    }

    // Load supported products V2.
    if (const auto ja = j.value(key::kProducts).toArray(); ja.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find supported product infos.";
        return false;
    }
    else if (!ProductInfo::loadProductsFromJson(
                 QJsonDocument{ja}
                     .toJson(QJsonDocument::Compact)
                     .toStdString())) {
        qWarning() << "Invalid configs: "
                      "Incompatiable product infos.";
        return false;
    }

    if (const auto jo = j.value(key::kCalibTask).toObject(); jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find calibration task configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _calibTaskConfigs)) {
        return false;
    }

    if (const auto jo = j.value(key::kCalib).toObject(); jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find calibration configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _calibConfigs)) {
        return false;
    }

    if (const auto jo = j.value(key::kCalibReference).toObject(); jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find calibration reference.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _calibReference)) {
        return false;
    }

    if (const auto jo = j.value(key::kVerify).toObject(); jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find verification configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _verifyConfigs)) {
        return false;
    }

    if (const auto jo = j.value(key::kMonoCalibration).toObject(); jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find mono camera calibration configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _monoCalibration)) {
        return false;
    }

    if (const auto jo = j.value(key::kMonoVerification).toObject();
        jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find mono camera verification configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _monoVerification)) {
        return false;
    }

    if (const auto jo = j.value(key::kStereoCalibrationV2).toObject();
        jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find stereo camera calibration v2 configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _stereoCalibration)) {
        return false;
    }

    if (const auto jo = j.value(key::kStereoVerificationV2).toObject();
        jo.empty()) {
        qWarning() << "Invalid configs: "
                      "Failed to find stereo camera verification v2 configs.";
        return false;
    }
    else if (!loadQJsonObjectToString(jo, _stereoVerification)) {
        return false;
    }

    return success;
}

bool GlobalConfigs::Impl::setMainWindowConfigsFromJson(const QJsonObject& jo)
{
    _mainWindow.snPattern = jo.value(key::kSNPattern).toString().toStdString();
    _mainWindow.homePage = MainWindow::Page(jo.value(key::kHomePage).toInt(0));
    _mainWindow.devMode = jo.value(key::kDevMode).toBool(false);

    return true;
}

///------- GlobalConfigs starts from here
GlobalConfigs::GlobalConfigs(const std::string& filename)
    : d(std::make_unique<Impl>())
{
    if (!filename.empty()) {
        setFromFile(filename);
    }
}

GlobalConfigs::~GlobalConfigs() = default;

GlobalConfigs& GlobalConfigs::instance()
{
    static GlobalConfigs configs;
    return configs;
}

bool GlobalConfigs::setFromFile(const std::string& filename)
{
    QFile file{QString::fromStdString(filename)};
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Failed to open global configs file. " << filename;
        return false;
    }

    if (const auto jo = QJsonDocument::fromJson(file.readAll()).object();
        jo.empty()) {
        qWarning()
            << "Invalid configs:"
               "Empty global config. The input JSON may be incompatible.";
        return false;
    }
    else {
        return d->setFromJson(jo);
    }
}

const std::string& GlobalConfigs::version() const { return d->_version; }

const MainWindow::Configs& GlobalConfigs::mainWindow() const
{
    return d->_mainWindow;
}

const std::string& GlobalConfigs::productConfigs() const
{
    return d->_prodConfigs;
}

const std::string& GlobalConfigs::calibTaskConfigs() const
{
    return d->_calibTaskConfigs;
}

const std::string& GlobalConfigs::calibConfigs() const
{
    return d->_calibConfigs;
}

const std::string& GlobalConfigs::calibReference() const
{
    return d->_calibReference;
}

const std::string& GlobalConfigs::stereoVerificationV1() const
{
    return d->_verifyConfigs;
}

const std::string& GlobalConfigs::monoCalibration() const
{
    return d->_monoCalibration;
}

const std::string& GlobalConfigs::monoVerification() const
{
    return d->_monoVerification;
}

const std::string& GlobalConfigs::stereoCalibration() const
{
    return d->_stereoCalibration;
}

const std::string& GlobalConfigs::stereoVerification() const
{
    return d->_stereoVerification;
}

} // namespace tl

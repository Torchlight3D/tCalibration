#pragma once

#include "MainWindow.h"

namespace tl {

// TODO:
// 1. Use nlohmann::json instead of Qt Json
class GlobalConfigs
{
public:
    static GlobalConfigs& instance();
    ~GlobalConfigs();

    bool setFromFile(const std::string& filename);

    const std::string& version() const;
    const MainWindow::Configs& mainWindow() const;

    /// V1 interfaces
    const std::string& productConfigs() const;
    const std::string& calibTaskConfigs() const;
    const std::string& calibConfigs() const;
    const std::string& calibReference() const;
    const std::string& stereoVerificationV1() const;

    /// V2 interfaces
    // TODO:
    // 1. We can auto register View's configs with type enum (or string), but
    // there could be some configs not related to any views.
    const std::string& monoCalibration() const;
    const std::string& monoVerification() const;
    const std::string& stereoCalibration() const;
    const std::string& stereoVerification() const;

private:
    explicit GlobalConfigs(const std::string& filename = {});

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

} // namespace tl

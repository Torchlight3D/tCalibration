#pragma once

#include <mutex>
#include <unordered_map>

#include <AxCore/axglobal.h>
#include <AxCore/HashUtils>

#include "featuredatabase.h"

namespace thoht {

// A simple implementation for storing features and feature matches in memory.
class InMemoryFeaturesAndMatchesDatabase : public FeaturesAndMatchesDatabase
{
public:
    InMemoryFeaturesAndMatchesDatabase() = default;
    ~InMemoryFeaturesAndMatchesDatabase() = default;

    /// Data access
    void putCameraMetaData(const std::string& name,
                           const CameraMetaData& meta) override;
    CameraMetaData cameraMetaData(const std::string& name) const override;
    bool hasCameraMetaData(const std::string& name) const override;
    std::vector<std::string> imageNamesOfCameraMetaData() const override;
    size_t cameraMetaDataCount() const override;

    void putFeatures(const std::string& name,
                     const KeypointsAndDescriptors& features) override;
    KeypointsAndDescriptors features(const std::string& name) const override;
    bool hasFeatures(const std::string& name) const override;
    std::vector<std::string> imageNamesOfFeatures() const override;
    size_t imageCount() const override;

    void putImagePairMatch(const std::string& name1, const std::string& name2,
                           const ImagePairMatch& match) override;
    ImagePairMatch imagePairMatch(const std::string& name1,
                                  const std::string& name2) const override;
    std::vector<std::pair<std::string, std::string>> imageNamesOfMatches()
        const override;
    size_t matchCount() const override;

    void removeAllMatches() override;

    // IO, WARNING: Not ready
    bool ReadFromFile(const std::string& filepath);
    bool WriteToFile(const std::string& filepath) const;

private:
    DISABLE_COPY(InMemoryFeaturesAndMatchesDatabase)

    std::mutex m_mutex;
    std::unordered_map<std::string, CameraMetaData> m_nameToMeta;
    std::unordered_map<std::string, KeypointsAndDescriptors> m_nameToFeature;
    std::unordered_map<std::pair<std::string, std::string>, ImagePairMatch>
        m_nameToMatch;
};

} // namespace thoht

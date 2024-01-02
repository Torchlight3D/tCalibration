#include "memoryfeaturedatabase.h"

#include <fstream>

#include <AxCore/ContainerUtils>

namespace thoht {

void InMemoryFeaturesAndMatchesDatabase::putCameraMetaData(
    const std::string& name, const CameraMetaData& meta)
{
    m_nameToMeta[name] = meta;
}

CameraMetaData InMemoryFeaturesAndMatchesDatabase::cameraMetaData(
    const std::string& name) const
{
    return utils::FindOrDie(m_nameToMeta, name);
}

bool InMemoryFeaturesAndMatchesDatabase::hasCameraMetaData(
    const std::string& name) const
{
    return utils::ContainsKey(m_nameToMeta, name);
}

std::vector<std::string>
InMemoryFeaturesAndMatchesDatabase::imageNamesOfCameraMetaData() const
{
    std::vector<std::string> names;
    names.reserve(m_nameToMeta.size());
    for (const auto& [name, _] : m_nameToMeta) {
        names.push_back(name);
    }
    return names;
}

size_t InMemoryFeaturesAndMatchesDatabase::cameraMetaDataCount() const
{
    return m_nameToMeta.size();
}

void InMemoryFeaturesAndMatchesDatabase::putFeatures(
    const std::string& image_name, const KeypointsAndDescriptors& features)
{
    m_nameToFeature[image_name] = features;
}

KeypointsAndDescriptors InMemoryFeaturesAndMatchesDatabase::features(
    const std::string& name) const
{
    return utils::FindOrDie(m_nameToFeature, name);
}

bool InMemoryFeaturesAndMatchesDatabase::hasFeatures(
    const std::string& name) const
{
    return utils::ContainsKey(m_nameToFeature, name);
}

std::vector<std::string>
InMemoryFeaturesAndMatchesDatabase::imageNamesOfFeatures() const
{
    std::vector<std::string> names;
    names.reserve(m_nameToFeature.size());
    for (const auto& [name, _] : m_nameToFeature) {
        names.push_back(name);
    }
    return names;
}

size_t InMemoryFeaturesAndMatchesDatabase::imageCount() const
{
    return m_nameToFeature.size();
}

void InMemoryFeaturesAndMatchesDatabase::putImagePairMatch(
    const std::string& name1, const std::string& name2,
    const ImagePairMatch& matches)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_nameToMatch[std::make_pair(name1, name2)] = matches;
}

ImagePairMatch InMemoryFeaturesAndMatchesDatabase::imagePairMatch(
    const std::string& name1, const std::string& name2) const
{
    return utils::FindOrDieNoPrint(m_nameToMatch, std::make_pair(name1, name2));
}

std::vector<std::pair<std::string, std::string>>
InMemoryFeaturesAndMatchesDatabase::imageNamesOfMatches() const
{
    std::vector<std::pair<std::string, std::string>> pairs;
    pairs.reserve(m_nameToMatch.size());
    for (const auto& [pair, _] : m_nameToMatch) {
        pairs.push_back(pair);
    }
    return pairs;
}

size_t InMemoryFeaturesAndMatchesDatabase::matchCount() const
{
    return m_nameToMatch.size();
}

void InMemoryFeaturesAndMatchesDatabase::removeAllMatches()
{
    m_nameToMatch.clear();
}

bool InMemoryFeaturesAndMatchesDatabase::ReadFromFile(
    const std::string& filepath)
{
    // TODO:
    return false;
}

bool InMemoryFeaturesAndMatchesDatabase::WriteToFile(
    const std::string& filepath) const
{
    // TODO:
    return true;
}

} // namespace thoht

#pragma once

#include <string>
#include <vector>

#include <AxCamera/CameraMetaData>
#include <AxImgproc/Keypoint>

#include "imagepairmatch.h"

namespace thoht {

// An interface for retreiving feature and match related data. This data is
// typically memory intensive so caches or database systems may be used to
// access the data more efficiently. This class is guaranteed to be thread safe.
class FeaturesAndMatchesDatabase
{
public:
    virtual ~FeaturesAndMatchesDatabase() {}

    /// Data access
    virtual void putCameraMetaData(const std::string& name,
                                   const CameraMetaData& meta) = 0;
    virtual CameraMetaData cameraMetaData(const std::string& name) const = 0;
    virtual bool hasCameraMetaData(const std::string& name) const = 0;
    virtual std::vector<std::string> imageNamesOfCameraMetaData() const = 0;
    virtual size_t cameraMetaDataCount() const = 0;

    virtual void putFeatures(const std::string& name,
                             const KeypointsAndDescriptors& features) = 0;
    // TODO: Plural?
    virtual KeypointsAndDescriptors features(const std::string& name) const = 0;
    virtual bool hasFeatures(const std::string& name) const = 0;
    virtual std::vector<std::string> imageNamesOfFeatures() const = 0;
    virtual size_t imageCount() const = 0;

    virtual void putImagePairMatch(const std::string& name1,
                                   const std::string& name2,
                                   const ImagePairMatch& match) = 0;
    virtual ImagePairMatch imagePairMatch(const std::string& name1,
                                          const std::string& name2) const = 0;
    virtual std::vector<std::pair<std::string, std::string>>
    imageNamesOfMatches() const = 0;
    virtual size_t matchCount() const = 0;

    virtual void removeAllMatches() = 0;
};

} // namespace thoht

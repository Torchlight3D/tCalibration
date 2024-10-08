#pragma once

#include <map>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>

namespace tl {

struct Feature
{
    int x;
    int y;
    int label;
    float theta;

    void read(const cv::FileNode &fn);
    void write(cv::FileStorage &fs) const;

    Feature() : x(0), y(0), label(0) {}
    Feature(int x, int y, int label);
};

inline Feature::Feature(int _x, int _y, int _label)
    : x(_x), y(_y), label(_label)
{
}

struct Template
{
    int width;
    int height;
    int tl_x;
    int tl_y;
    int pyramid_level;
    std::vector<Feature> features;

    void read(const cv::FileNode &fn);
    void write(cv::FileStorage &fs) const;
};

class ColorGradientPyramid
{
public:
    ColorGradientPyramid(const cv::Mat &src, const cv::Mat &mask,
                         float weak_threshold, size_t num_features,
                         float strong_threshold);

    void quantize(cv::Mat &dst) const;

    bool extractTemplate(Template &templ) const;

    void pyrDown();

public:
    void update();
    /// Candidate feature with a score
    struct Candidate
    {
        Candidate(int x, int y, int label, float score);

        /// Sort candidates with high score to the front
        bool operator<(const Candidate &rhs) const { return score > rhs.score; }

        Feature f;
        float score;
    };

    cv::Mat src;
    cv::Mat mask;

    int pyramid_level;
    cv::Mat angle;
    cv::Mat magnitude;
    cv::Mat angle_ori;

    float weak_threshold;
    size_t num_features;
    float strong_threshold;
    static bool selectScatteredFeatures(
        const std::vector<Candidate> &candidates,
        std::vector<Feature> &features, size_t num_features, float distance);
};

inline ColorGradientPyramid::Candidate::Candidate(int x, int y, int label,
                                                  float _score)
    : f(x, y, label), score(_score)
{
}

class ColorGradient
{
public:
    ColorGradient();
    ColorGradient(float weak_threshold, size_t num_features,
                  float strong_threshold);

    std::string name() const;

    float weak_threshold;
    size_t num_features;
    float strong_threshold;
    void read(const cv::FileNode &fn);
    void write(cv::FileStorage &fs) const;

    cv::Ptr<ColorGradientPyramid> process(const cv::Mat src,
                                          const cv::Mat &mask = cv::Mat()) const
    {
        return cv::makePtr<ColorGradientPyramid>(
            src, mask, weak_threshold, num_features, strong_threshold);
    }
};

struct Match
{
    Match() {}

    Match(int x, int y, float similarity, const std::string &class_id,
          int template_id);

    /// Sort matches with high similarity to the front
    bool operator<(const Match &rhs) const
    {
        // Secondarily sort on template_id for the sake of duplicate removal
        if (similarity != rhs.similarity)
            return similarity > rhs.similarity;
        return template_id < rhs.template_id;
    }

    bool operator==(const Match &rhs) const
    {
        return x == rhs.x && y == rhs.y && similarity == rhs.similarity &&
               class_id == rhs.class_id;
    }

    int x;
    int y;
    float similarity;
    std::string class_id;
    int template_id;
};

inline Match::Match(int _x, int _y, float _similarity,
                    const std::string &_class_id, int _template_id)
    : x(_x),
      y(_y),
      similarity(_similarity),
      class_id(_class_id),
      template_id(_template_id)
{
}

class Detector
{
public:
    Detector();
    Detector(std::vector<int> T);
    Detector(int num_features, std::vector<int> T, float weak_thresh = 30.0f,
             float strong_thresh = 60.0f);

    std::vector<Match> match(cv::Mat sources, float threshold,
                             const std::vector<std::string> &class_ids = {},
                             const cv::Mat masks = {}) const;

    int addTemplate(const cv::Mat sources, const std::string &class_id,
                    const cv::Mat &object_mask, int num_features = 0);

    int addTemplate_rotate(const std::string &class_id, int zero_id,
                           float theta, cv::Point2f center);

    const cv::Ptr<ColorGradient> &getModalities() const { return modality; }

    int getT(int pyramid_level) const { return T_at_level[pyramid_level]; }

    int pyramidLevels() const { return pyramid_levels; }

    const std::vector<Template> &getTemplates(const std::string &class_id,
                                              int template_id) const;

    int numTemplates() const;
    int numTemplates(const std::string &class_id) const;
    int numClasses() const { return static_cast<int>(class_templates.size()); }

    std::vector<std::string> classIds() const;

    void read(const cv::FileNode &fn);
    void write(cv::FileStorage &fs) const;

    std::string readClass(const cv::FileNode &fn,
                          const std::string &class_id_override = "");
    void writeClass(const std::string &class_id, cv::FileStorage &fs) const;

    void readClasses(const std::vector<std::string> &class_ids,
                     const std::string &format = "templates_%s.yml.gz");
    void writeClasses(const std::string &format = "templates_%s.yml.gz") const;

protected:
    cv::Ptr<ColorGradient> modality;
    int pyramid_levels;
    std::vector<int> T_at_level;

    using TemplatePyramid = std::vector<Template>;
    using TemplatesMap = std::map<std::string, std::vector<TemplatePyramid>>;
    TemplatesMap class_templates;

    using LinearMemories = std::vector<cv::Mat>;
    // Indexed as [pyramid level][ColorGradient][quantized label]
    using LinearMemoryPyramid = std::vector<std::vector<LinearMemories>>;

    void matchClass(
        const LinearMemoryPyramid &lm_pyramid,
        const std::vector<cv::Size> &sizes, float threshold,
        std::vector<Match> &matches, const std::string &class_id,
        const std::vector<TemplatePyramid> &template_pyramids) const;
};

class shapeInfo_producer
{
public:
    cv::Mat src;
    cv::Mat mask;

    std::vector<float> angle_range;
    std::vector<float> scale_range;

    float angle_step = 15;
    float scale_step = 0.5;
    float eps = 0.00001f;

    struct Info
    {
        float angle;
        float scale;
    };
    std::vector<Info> infos;

    explicit shapeInfo_producer(cv::Mat src, cv::Mat mask = cv::Mat());

    static cv::Mat transform(cv::InputArray src, float angle, float scale);

    // TODO: These two are JSON, use nlohmann::json later
    static void save_infos(const std::vector<shapeInfo_producer::Info> &infos,
                           const std::string &path = "infos.yaml");
    static std::vector<Info> load_infos(const std::string &path = "info.yaml");

    void produce_infos();

    cv::Mat src_of(const Info &info);

    cv::Mat mask_of(const Info &info);
};

} // namespace tl

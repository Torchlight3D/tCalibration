#pragma once

#include <QObject>

namespace tl {

// TODO:
// 1. Make Player abstraction, and clean up the player logic.
// 2. Add PlayMode [Normal;RepeatOne;RepeatAll;Shuffle]. Current play mode is
// equivalent to Normal.
// 3. Add PlayDirection[Forward;Backward;Rewind].
// Current play direction is forward. Rewind is a little bit tricky.

// WARNING: Not finished!!!
class CalibTaskInfo
{
public:
    inline static constexpr char kLeftVisualFilename[9]{"cam0.avi"};
    inline static constexpr char kRightVisualFilename[9]{"cam1.avi"};
    inline static constexpr char kLeftVisualMetaFilename[9]{"cam0.csv"};
    inline static constexpr char kRightVisualMetaFilename[9]{"cam1.csv"};
    inline static constexpr char kMotionFilename[11]{"motion.csv"};

public:
    enum Type
    {
        None,
        Calibration,
        Verification,
        Mix,
        Playback,
    };

    explicit CalibTaskInfo(const std::string &path);

    static bool checkPath(const std::string &path, Type &type);
    // Lazy initialization
    bool validate();
    bool isValid() const;

    /// Cameras
    std::string camerasDir(const std::string &sub = {}) const;
    inline auto leftVisualFilename() const
    {
        return camerasDir(kLeftVisualFilename);
    }
    inline auto rightVisualFilename() const
    {
        return camerasDir(kRightVisualFilename);
    }
    inline auto leftVisualMetaFilename() const
    {
        return camerasDir(kLeftVisualMetaFilename);
    }
    inline auto rightVisualMetaFilename() const
    {
        return camerasDir(kRightVisualMetaFilename);
    }
    inline auto motionFilename() const { return camerasDir(kMotionFilename); }

    /// Verify samples
    std::string verifySamplesDir(const std::string &sub = {}) const;
    /// Reports
    std::string calibParameterFilename() const;

    /// Playback
    std::string playbackDir(const std::string &sub = {}) const;

private:
    std::string _path{};

    Type _type{None};
};

class CalibTaskPlayer
{
public:
    explicit CalibTaskPlayer(const std::string &path = {});

    bool isValid() const;

    struct Infos
    {
        std::string params;
        std::string left, right;
        std::string leftMeta, rightMeta;
        std::string motion;

        bool isValid() const
        {
            return !params.empty() && !left.empty() && !right.empty() &&
                   !leftMeta.empty() && !rightMeta.empty() && !motion.empty();
        }
    };
    const Infos &infos() const;

private:
    bool parsePath(const std::string &path);

private:
    Infos m_infos;
};

struct DeviceInfo;
struct ImuData;
struct StereoImages;

// TODO:
// 1. Similar to GroupVerifyTaskPlayer. How to abstract?
// 2. Get rid of QObject, callback could be an option
class GroupCalibTaskPlayer : public QObject
{
    Q_OBJECT

public:
    explicit GroupCalibTaskPlayer(const std::string &path = {},
                                  QObject *parent = nullptr);

    // Lazy initialization
    bool loadTasks();
    bool isValid() const;
    const std::string &root() const;

    int count() const;
    int currentIndex() const;
    void playCurrent();
    void toPrevTask();
    void toNextTask();
    void toTask(int index);
    bool finished() const;

signals:
    void loadingProgress(int current, int total);
    void taskLoaded(const DeviceInfo &info);
    void stereoImageDataReceived(const StereoImages &stereo);
    void imuDataReceived(const ImuData &imu);
    void streamDataFinished();

private:
    bool parsePath(const std::string &path);
    void dockCurrent();
    void streamCurrent();

private:
    const std::string m_path;
    std::vector<CalibTaskPlayer> m_players;
    int m_index{-1};
};

inline constexpr int kInvalidIndex{-1};
inline constexpr char kVerifyReportPrefixV2[]{"verify_report"};
inline constexpr char kVerifyReportPrefixV1[]{"verification_report"};

class VerifyTaskPlayer
{
public:
    explicit VerifyTaskPlayer(const std::string &taskDir = {});

    bool isValid() const;

    size_t count() const;
    std::string name() const;

    struct Infos
    {
        std::string parameters;
        std::string left, right;
        std::string report;
    };
    Infos currentInfos() const;
    int currentIndex() const;
    void playCurrent();
    void playPrev();
    void playNext();
    void toIndex(size_t index);
    inline void toBegin() { toIndex(0); }
    bool finished() const;

private:
    bool parsePath(const std::string &path);

private:
    std::vector<std::string> m_suffixes;
    std::string m_root{};
    size_t m_index{0};
};

// TODO:
// 1. Get rid of QObject, callback could be an option
class GroupVerifyTaskPlayer : public QObject
{
    Q_OBJECT

public:
    explicit GroupVerifyTaskPlayer(const std::string &savedDataDir = {},
                                   QObject *parent = nullptr);

    bool isValid() const;

    /// In the context of player group
    void toPrevTask();
    void toNextTask();
    void toTask(size_t index);
    size_t currentTaskIndex() const;
    size_t taskCount() const;

    /// In the context of current player
    void playCurrent();
    void playPrev();
    void playNext();
    void playIndex(size_t index);
    size_t frameCount() const;

    bool finished() const;

signals:
    void taskLoaded(const std::string &name, size_t size);
    void stereoDataReceived(const std::string &params,
                            const StereoImages &stereo);

private:
    bool parsePath(const std::string &path);
    void loadCurrentFrame();

private:
    std::vector<VerifyTaskPlayer> m_players;
    size_t m_index{0};
    bool m_forward{true};
};

} // namespace tl

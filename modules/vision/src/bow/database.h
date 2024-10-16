#pragma once

#include <vector>
#include <string>
#include <list>

#include "vocabulary.h"
#include "queryresults.h"
#include "bowvector.h"

namespace tl {

// For query functions
inline constexpr int MIN_COMMON_WORDS = 5;

class Database
{
public:
    /**
     * Creates an empty database without vocabulary
     * @param use_di a direct index is used to store feature indexes
     * @param di_levels levels to go up the vocabulary tree to select the
     *   node id to store in the direct index when adding images
     */
    explicit Database(bool useDirectIndex = true, int directIndexLevel = 0);

    /**
     * Creates a database with the given vocabulary
     * @param T class inherited from Vocabulary
     * @param voc vocabulary
     * @param use_di a direct index is used to store feature indexes
     * @param di_levels levels to go up the vocabulary tree to select the
     *   node id to store in the direct index when adding images
     */

    explicit Database(const Vocabulary &voc, bool useDirectIndex = true,
                      int directIndexLevel = 0);
    Database(const std::string &filename);

    Database(const Database &db);
    Database &operator=(const Database &db);
    virtual ~Database();

    /**
     * Sets the vocabulary to use and clears the content of the database.
     * @param T class inherited from Vocabulary
     * @param voc vocabulary to copy
     */
    void setVocabulary(const Vocabulary &voc);

    /**
     * Sets the vocabulary to use and the direct index parameters, and clears
     * the content of the database
     * @param T class inherited from Vocabulary
     * @param voc vocabulary to copy
     * @param use_di a direct index is used to store feature indexes
     * @param di_levels levels to go up the vocabulary tree to select the
     *   node id to store in the direct index when adding images
     */

    void setVocabulary(const Vocabulary &voc, bool use_di, int di_levels = 0);

    /**
     * Returns a pointer to the vocabulary used
     * @return vocabulary
     */
    const Vocabulary *getVocabulary() const;

    /**
     * Allocates some memory for the direct and inverted indexes
     * @param nd number of expected image entries in the database
     * @param ni number of expected words per image
     * @note Use 0 to ignore a parameter
     */
    void allocate(int nd = 0, int ni = 0);

    /**
     * Adds an entry to the database and returns its index
     * @param features features of the new entry
     * @param bowvec if given, the bow vector of these features is returned
     * @param fvec if given, the vector of nodes and feature indexes is returned
     * @return id of new entry
     */
    EntryId add(const std::vector<cv::Mat> &features, BowVector *bowvec = NULL,
                FeatureVector *fvec = NULL);
    /**
     * Adds an entry to the database and returns its index
     * @param features features of the new entry, one per row
     * @param bowvec if given, the bow vector of these features is returned
     * @param fvec if given, the vector of nodes and feature indexes is returned
     * @return id of new entry
     */
    EntryId add(const cv::Mat &features, BowVector *bowvec = NULL,
                FeatureVector *fvec = NULL);

    /**
     * Adss an entry to the database and returns its index
     * @param vec bow vector
     * @param fec feature vector to add the entry. Only necessary if using the
     *   direct index
     * @return id of new entry
     */
    EntryId add(const BowVector &vec,
                const FeatureVector &fec = FeatureVector());

    /**
     * Empties the database
     */
    void clear();

    /**
     * Returns the number of entries in the database
     * @return number of entries in the database
     */
    unsigned int size() const { return m_nentries; }

    /**
     * Checks if the direct index is being used
     * @return true iff using direct index
     */
    bool usingDirectIndex() const { return m_use_di; }

    /**
     * Returns the di levels when using direct index
     * @return di levels
     */
    int getDirectIndexLevels() const { return m_dilevels; }

    /**
     * Queries the database with some features
     * @param features query features
     * @param ret (out) query results
     * @param max_results number of results to return. <= 0 means all
     * @param max_id only entries with id <= max_id are returned in ret.
     *   < 0 means all
     */
    void query(const std::vector<cv::Mat> &features, QueryResults &ret,
               int max_results = 1, int max_id = -1) const;
    /**
     * Queries the database with some features
     * @param features query features,one per row
     * @param ret (out) query results
     * @param max_results number of results to return. <= 0 means all
     * @param max_id only entries with id <= max_id are returned in ret.
     *   < 0 means all
     */
    void query(const cv::Mat &features, QueryResults &ret, int max_results = 1,
               int max_id = -1) const;

    /**
     * Queries the database with a vector
     * @param vec bow vector already normalized
     * @param ret results
     * @param max_results number of results to return. <= 0 means all
     * @param max_id only entries with id <= max_id are returned in ret.
     *   < 0 means all
     */
    void query(const BowVector &vec, QueryResults &ret, int max_results = 1,
               int max_id = -1) const;

    /**
     * Returns the a feature vector associated with a database entry
     * @param id entry id (must be < size())
     * @return const reference to map of nodes and their associated features in
     *   the given entry
     */
    const FeatureVector &retrieveFeatures(EntryId id) const;

    // OpenCV YAML
    void save(const std::string &filename) const;
    void load(const std::string &filename);

    /**
     * Stores the database in the given file storage structure
     * @param fs
     * @param name node name
     */
    virtual void save(cv::FileStorage &fs,
                      const std::string &name = "database") const;

    /**
     * Loads the database from the given file storage structure
     * @param fs
     * @param name node name
     */
    virtual void load(const cv::FileStorage &fs,
                      const std::string &name = "database");

    // --------------------------------------------------------------------------

    /**
     * Writes printable information of the database
     * @param os stream to write to
     * @param db
     */
    friend std::ostream &operator<<(std::ostream &os, const Database &db);

protected:
    /// Query with L1 scoring
    void queryL1(const BowVector &vec, QueryResults &ret, int max_results,
                 int max_id) const;

    /// Query with L2 scoring
    void queryL2(const BowVector &vec, QueryResults &ret, int max_results,
                 int max_id) const;

    /// Query with Chi square scoring
    void queryChiSquare(const BowVector &vec, QueryResults &ret,
                        int max_results, int max_id) const;

    /// Query with Bhattacharyya scoring
    void queryBhattacharyya(const BowVector &vec, QueryResults &ret,
                            int max_results, int max_id) const;

    /// Query with KL divergence scoring
    void queryKL(const BowVector &vec, QueryResults &ret, int max_results,
                 int max_id) const;

    /// Query with dot product scoring
    void queryDotProduct(const BowVector &vec, QueryResults &ret,
                         int max_results, int max_id) const;

protected:
    /* Inverted file declaration */

    /// Item of IFRow
    struct IFPair
    {
        EntryId entry_id;
        WordValue word_weight;

        IFPair() {}
        IFPair(EntryId eid, WordValue wv) : entry_id(eid), word_weight(wv) {}

        inline bool operator==(EntryId eid) const { return entry_id == eid; }
    };

    /// Row of InvertedFile
    using IFRow = std::list<IFPair>;
    // IFRows are sorted in ascending entry_id order

    /// Inverted index
    using InvertedFile = std::vector<IFRow>;
    // InvertedFile[word_id] --> inverted file of that word

    /* Direct file declaration */

    /// Direct index
    using DirectFile = std::vector<FeatureVector>;
    // DirectFile[entry_id] --> [ directentry, ... ]

protected:
    /// Associated vocabulary
    std::unique_ptr<Vocabulary> m_voc;

    /// Flag to use direct index
    bool m_use_di;

    /// Levels to go up the vocabulary tree to select nodes to store
    /// in the direct index
    int m_dilevels;

    /// Inverted file (must have size() == |words|)
    InvertedFile m_ifile;

    /// Direct file (resized for allocation)
    DirectFile m_dfile;

    /// Number of valid entries in m_dfile
    int m_nentries;
};

} // namespace tl

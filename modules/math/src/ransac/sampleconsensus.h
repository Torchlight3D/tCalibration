#pragma once

#include <algorithm>

#include <glog/logging.h>

#include "qualitymeasurement.h"
#include "sampler.h"

namespace tl {

struct SacParameters
{
    std::shared_ptr<RandomNumberGenerator> rng;

    // Error threshold to determine inliers for RANSAC (e.g. squared
    // reprojection error). This is what will be used by the estimator to
    // determine inliers.
    double error_thresh = -1.;

    // The failure probability of RANSAC. Set to 0.01 means that RANSAC has a 1%
    // chance of missing the correct pose.
    double failure_probability = 0.01;

    // The minimal assumed inlier ratio, i.e., it is assumed that the given set
    // of correspondences has an inlier ratio of at least min_inlier_ratio.
    // This is required to limit the number of RANSAC iteratios.
    double min_inlier_ratio = 0;

    // The minimum number of iterations required before exiting.
    int min_iterations = 100;

    // Another way to specify the maximal number of RANSAC iterations. In
    // effect, the maximal number of iterations is set to
    // min(max_ransac_iterations, T), where T is the number of iterations
    // corresponding to min_inlier_ratio. This variable is useful if RANSAC is
    // to be applied iteratively, i.e., first applying RANSAC with an
    // min_inlier_ratio of x, then with one of x-y and so on, and we want to
    // avoid repeating RANSAC iterations. However, the preferable way to limit
    // the number of RANSAC iterations is to set min_inlier_ratio and leave
    // max_ransac_iterations to its default value. Per default, this variable is
    // set to std::numeric_limits<int>::max().
    int max_iterations = std::numeric_limits<int>::max();

    // Instead of the standard inlier count, use the Maximum Likelihood Estimate
    // (MLE) to determine the best solution. Inliers are weighted by their error
    // and outliers count as a constant penalty.
    bool use_mle = false;

    // If local optimization should be used (LO-RANSAC). This only works if the
    // corresponding estimator has the RefineModel() model function implemented.
    // Otherwise no local optimization is performed
    bool use_lo = false;

    // Local optimizaton should not start directly at the beginning but
    // rather after ransac has performed some sifting already
    int lo_start_iterations = 50;

    // Whether to use the T_{d,d}, with d=1, test proposed in
    // Chum, O. and Matas, J.: Randomized RANSAC and T(d,d) test, BMVC 2002.
    // After computing the pose, RANSAC selects one match at random and
    // evaluates all poses. If the point is an outlier to one pose, the
    // corresponding pose is rejected. Notice that if the pose solver returns
    // multiple poses, then at most one pose is correct. If the selected match
    // is correct, then only the correct pose will pass the test. Per default,
    // the test is disabled.
    //
    // NOTE: Not implemented!
    bool use_Tdd_test = false;
};

struct SacSummary
{
    // Indices of all inliers.
    std::vector<size_t> inliers;

    // The confidence in the solution.
    double confidence;

    // Number of input data
    int num_input_data_points;

    // The number of iterations performed before stopping RANSAC.
    int num_iterations;

    // Number of local optimization iterations
    int num_lo_iterations = 0;
};

template <class ModelEstimator>
class SampleConsensus
{
public:
    using Data = typename ModelEstimator::Data;
    using Model = typename ModelEstimator::Model;

    SampleConsensus(const SacParameters& params,
                    const ModelEstimator& estimator);
    virtual ~SampleConsensus() = default;

    virtual bool Initialize() { return true; }

    // Computes the best-fitting model using RANSAC.
    //
    // Inputs:
    //   data: sample set
    // Outputs:
    //   model: best fit model estimated from RANSAC
    //   summary: Ransac summary
    // Return:
    //   bool: if Ransac calculation is successful
    virtual bool Estimate(const std::vector<Data>& data, Model* model,
                          SacSummary* summary);

protected:
    // This method is called from derived classes to set up the sampling scheme
    // and the method for computing inliers. It must be called by derived
    // classes unless they override the Estimate(...) method. The method for
    // computing inliers (standar inlier support or MLE) is determined by the
    // ransac params.
    //
    // sampler: The class that instantiates the sampling strategy for this
    //   particular type of sampling consensus.
    bool SetUpSampler(Sampler* sampler);

    // Computes the maximum number of iterations required to ensure the inlier
    // ratio is the best with a probability corresponding to log_failure_prob.
    int ComputeMaxIterations(double min_sample_size, double inlier_ratio,
                             double log_failure_prob) const;

    // For LO
    void GetInliers(const std::vector<Data>& data,
                    const std::vector<size_t>& indices,
                    std::vector<Data>& inliers) const;

protected:
    std::unique_ptr<Sampler> sampler_;
    std::unique_ptr<QualityMeasurement> quality_measurement_;
    const SacParameters& _params;
    const ModelEstimator& _estimator;
};

/// -------------------------- Implementation ------------------------------//
///
template <class ModelEstimator>
SampleConsensus<ModelEstimator>::SampleConsensus(
    const SacParameters& params, const ModelEstimator& estimator)
    : _params(params), _estimator(estimator)
{
    CHECK_GT(params.error_thresh, 0)
        << "Error threshold must be set to greater than zero";
    CHECK_LE(params.min_inlier_ratio, 1.0);
    CHECK_GE(params.min_inlier_ratio, 0.0);
    CHECK_LT(params.failure_probability, 1.0);
    CHECK_GT(params.failure_probability, 0.0);
    CHECK_GE(params.max_iterations, params.min_iterations);
}

template <class ModelEstimator>
bool SampleConsensus<ModelEstimator>::SetUpSampler(Sampler* sampler)
{
    if (!sampler) {
        return false;
    }

    sampler_.reset(sampler);

    if (_params.use_mle) {
        quality_measurement_.reset(
            new MLEQualityMeasurement(_params.error_thresh));
    }
    else {
        quality_measurement_.reset(new InlierSupport(_params.error_thresh));
    }

    return quality_measurement_->Initialize();
}

template <class ModelEstimator>
int SampleConsensus<ModelEstimator>::ComputeMaxIterations(
    double minSampleSize, double inlierRatio, double log_failure_prob) const
{
    CHECK_GT(inlierRatio, 0.);
    if (inlierRatio == 1.) {
        return _params.min_iterations;
    }

    // If we use the T_{1,1} test, we have to adapt the number of samples
    // that needs to be generated accordingly since we use another
    // match for verification and a correct match is selected with probability
    // inlier_ratio.
    const double num_samples =
        _params.use_Tdd_test ? minSampleSize + 1 : minSampleSize;

    const double log_prob = std::log(1. - std::pow(inlierRatio, num_samples)) -
                            std::numeric_limits<double>::epsilon();

    // NOTE: For very low inlier ratios the number of iterations can actually
    // exceed the maximum value for an int. We need to keep this variable as a
    // double until we do the check below against the minimum and maximum number
    // of iterations in the parameter settings.
    const double num_iterations = log_failure_prob / log_prob;

    return std::clamp(num_iterations,
                      static_cast<double>(_params.min_iterations),
                      static_cast<double>(_params.max_iterations));
}

template <class ModelEstimator>
bool SampleConsensus<ModelEstimator>::Estimate(const std::vector<Data>& data,
                                               Model* bestModel,
                                               SacSummary* summary)
{
    CHECK(!data.empty())
        << "Cannot perform estimation with 0 data measurements!";
    CHECK_NOTNULL(sampler_.get());
    CHECK_NOTNULL(quality_measurement_.get());
    CHECK_NOTNULL(summary);
    summary->inliers.clear();
    CHECK_NOTNULL(bestModel);

    // Initialize the sampler with the size of the data input.
    if (!sampler_->Initialize(data.size())) {
        return false;
    }

    summary->num_input_data_points = data.size();

    const double log_failure_prob = std::log(_params.failure_probability);

    int max_iterations = _params.max_iterations;
    // Set the max iterations if the inlier ratio is set.
    if (_params.min_inlier_ratio > 0) {
        max_iterations = std::min(
            ComputeMaxIterations(_estimator.SampleSize(),
                                 _params.min_inlier_ratio, log_failure_prob),
            _params.max_iterations);
    }

    auto minCost = std::numeric_limits<double>::max();
    for (summary->num_iterations = 0; summary->num_iterations < max_iterations;
         summary->num_iterations++) {
        // Random sample data.
        std::vector<size_t> sampleIndices;
        if (!sampler_->Sample(&sampleIndices)) {
            continue;
        }

        // Get the corresponding data from indices.
        std::vector<Data> sampledData(sampleIndices.size());
        for (size_t i{0}; i < sampleIndices.size(); ++i) {
            sampledData[i] = data[sampleIndices[i]];
        }

        // Estimate model from sampled data. Skip to this iteration if fail.
        std::vector<Model> estimatedModels;
        if (!_estimator.EstimateModel(sampledData, &estimatedModels)) {
            continue;
        }

        // Calculate residuals from estimated model.
        for (const auto& model : estimatedModels) {
            const auto residuals = _estimator.Residuals(data, model);

            // Determine cost of the generated model.
            std::vector<size_t> inlierIndices;
            const double cost =
                quality_measurement_->ComputeCost(residuals, &inlierIndices);
            const double inlierRatio =
                static_cast<double>(inlierIndices.size()) /
                static_cast<double>(data.size());

            if (std::isgreaterequal(cost, minCost)) {
                continue;
            }

            // Update best model
            *bestModel = model;
            minCost = cost;

            if (inlierRatio <
                _estimator.SampleSize() / static_cast<double>(data.size())) {
                continue;
            }

            if (_params.use_lo &&
                summary->num_iterations >= _params.lo_start_iterations) {
                std::vector<Data> inliers;
                GetInliers(data, inlierIndices, inliers);
                if (!_estimator.RefineModel(inliers, _params.error_thresh,
                                            bestModel)) {
                    continue;
                }

                ++summary->num_lo_iterations;
            }

            // A better cost does not guarantee a higher inlier ratio (i.e,
            // the MLE case) so we only update the max iterations if the
            // number decreases.
            max_iterations =
                std::min(ComputeMaxIterations(_estimator.SampleSize(),
                                              inlierRatio, log_failure_prob),
                         max_iterations);

            VLOG(3) << "Inlier ratio = " << inlierRatio
                    << " and max number of iterations = " << max_iterations;
        }
    }

    // Compute the final inliers for the best model.
    const auto bestResiduals = _estimator.Residuals(data, *bestModel);
    quality_measurement_->ComputeCost(bestResiduals, &summary->inliers);

    if (_params.use_lo) {
        std::vector<Data> inliers;
        GetInliers(data, summary->inliers, inliers);
        _estimator.RefineModel(inliers, _params.error_thresh, bestModel);
        ++summary->num_lo_iterations;
    }

    const double inlier_ratio =
        static_cast<double>(summary->inliers.size()) / data.size();

    summary->confidence =
        1. - std::pow(1. - std::pow(inlier_ratio, _estimator.SampleSize()),
                      summary->num_iterations);

    return true;
}

template <class ModelEstimator>
void SampleConsensus<ModelEstimator>::GetInliers(
    const std::vector<Data>& data, const std::vector<size_t>& indices,
    std::vector<Data>& inliers) const
{
    inliers.resize(indices.size());
    for (size_t i{0}; i < indices.size(); i++) {
        inliers[i] = data[indices[i]];
    }
}

} // namespace tl

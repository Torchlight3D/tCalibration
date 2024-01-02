#pragma once

#include <glog/logging.h>

#include "inlier_support.h"
#include "mle_quality_measurement.h"
#include "sampler.h"

namespace thoht {

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

// A struct to hold useful outputs of Ransac-like methods.
struct SacSummary
{
    // Indices of all inliers.
    std::vector<int> inliers;

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
class SampleConsensusEstimator
{
public:
    using Datum = typename ModelEstimator::Datum;
    using Model = typename ModelEstimator::Model;

    SampleConsensusEstimator(const SacParameters& params,
                             const ModelEstimator& estimator);
    virtual ~SampleConsensusEstimator() = default;

    virtual bool Initialize() { return true; }

    // Computes the best-fitting model using RANSAC. Returns false if RANSAC
    // calculation fails and true (with the best_model output) if successful.
    //
    // Params:
    //   data: the set from which to sample
    //   estimator: The estimator used to estimate the model based on the Datum
    //     and Model type
    //   best_model: The output parameter that will be filled with the best
    //   model
    //     estimated from RANSAC
    virtual bool Estimate(const std::vector<Datum>& data, Model* best_model,
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
    bool Initialize(Sampler* sampler);

    // Computes the maximum number of iterations required to ensure the inlier
    // ratio is the best with a probability corresponding to log_failure_prob.
    int ComputeMaxIterations(double min_sample_size, double inlier_ratio,
                             double log_failure_prob) const;

    // For LO
    void GetInlierDatum(const std::vector<Datum>& data,
                        const std::vector<int>& inlier_indices,
                        std::vector<Datum>& inlier_datum) const;

protected:
    // The sampling strategy.
    std::unique_ptr<Sampler> sampler_;

    // The quality metric for the estimated model and data.
    std::unique_ptr<QualityMeasurement> quality_measurement_;

    // Ransac parameters
    const SacParameters& sac_params_;

    // Estimator to use for generating models.
    const ModelEstimator& estimator_;
};

/// -------------------------- Implementation ------------------------------//
///
template <class ModelEstimator>
SampleConsensusEstimator<ModelEstimator>::SampleConsensusEstimator(
    const SacParameters& params, const ModelEstimator& estimator)
    : sac_params_(params), estimator_(estimator)
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
bool SampleConsensusEstimator<ModelEstimator>::Initialize(Sampler* sampler)
{
    CHECK_NOTNULL(sampler);
    sampler_.reset(sampler);

    if (sac_params_.use_mle) {
        quality_measurement_.reset(
            new MLEQualityMeasurement(sac_params_.error_thresh));
    }
    else {
        quality_measurement_.reset(new InlierSupport(sac_params_.error_thresh));
    }
    return quality_measurement_->Initialize();
}

template <class ModelEstimator>
int SampleConsensusEstimator<ModelEstimator>::ComputeMaxIterations(
    double min_sample_size, double inlier_ratio, double log_failure_prob) const
{
    CHECK_GT(inlier_ratio, 0.);
    if (inlier_ratio == 1.) {
        return sac_params_.min_iterations;
    }

    // If we use the T_{1,1} test, we have to adapt the number of samples
    // that needs to be generated accordingly since we use another
    // match for verification and a correct match is selected with probability
    // inlier_ratio.
    const double num_samples =
        sac_params_.use_Tdd_test ? min_sample_size + 1 : min_sample_size;

    const double log_prob = log(1. - pow(inlier_ratio, num_samples)) -
                            std::numeric_limits<double>::epsilon();

    // NOTE: For very low inlier ratios the number of iterations can actually
    // exceed the maximum value for an int. We need to keep this variable as a
    // double until we do the check below against the minimum and maximum number
    // of iterations in the parameter settings.
    const double num_iterations = log_failure_prob / log_prob;

    return std::clamp(num_iterations,
                      static_cast<double>(sac_params_.min_iterations),
                      static_cast<double>(sac_params_.max_iterations));
}

template <class ModelEstimator>
bool SampleConsensusEstimator<ModelEstimator>::Estimate(
    const std::vector<Datum>& data, Model* best_model, SacSummary* summary)
{
    CHECK_GT(data.size(), 0)
        << "Cannot perform estimation with 0 data measurements!";
    CHECK_NOTNULL(sampler_.get());
    CHECK_NOTNULL(quality_measurement_.get());
    CHECK_NOTNULL(summary);
    summary->inliers.clear();
    CHECK_NOTNULL(best_model);

    // Initialize the sampler with the size of the data input.
    if (!sampler_->Initialize(data.size())) {
        return false;
    }

    summary->num_input_data_points = data.size();

    const double log_failure_prob = log(sac_params_.failure_probability);
    double best_cost = std::numeric_limits<double>::max();
    int max_iterations = sac_params_.max_iterations;

    // Set the max iterations if the inlier ratio is set.
    if (sac_params_.min_inlier_ratio > 0) {
        max_iterations =
            std::min(ComputeMaxIterations(estimator_.SampleSize(),
                                          sac_params_.min_inlier_ratio,
                                          log_failure_prob),
                     sac_params_.max_iterations);
    }

    for (summary->num_iterations = 0; summary->num_iterations < max_iterations;
         summary->num_iterations++) {
        // Sample subset. Proceed if successfully sampled.
        std::vector<int> data_subset_indices;
        if (!sampler_->Sample(&data_subset_indices)) {
            continue;
        }

        // Get the corresponding data elements for the subset.
        std::vector<Datum> data_subset(data_subset_indices.size());
        for (int i = 0; i < data_subset_indices.size(); i++) {
            data_subset[i] = data[data_subset_indices[i]];
        }

        // Estimate model from subset. Skip to next iteration if the model fails
        // to estimate.
        std::vector<Model> temp_models;
        if (!estimator_.EstimateModel(data_subset, &temp_models)) {
            continue;
        }

        // Calculate residuals from estimated model.
        for (const auto& model : temp_models) {
            const std::vector<double> residuals =
                estimator_.Residuals(data, model);

            // Determine cost of the generated model.
            std::vector<int> inlier_indices;
            const double sample_cost =
                quality_measurement_->ComputeCost(residuals, &inlier_indices);
            const double inlier_ratio =
                static_cast<double>(inlier_indices.size()) /
                static_cast<double>(data.size());

            // Update best model if error is the best we have seen.
            if (sample_cost < best_cost) {
                *best_model = model;
                best_cost = sample_cost;

                if (inlier_ratio < estimator_.SampleSize() /
                                       static_cast<double>(data.size())) {
                    continue;
                }

                if (summary->num_iterations >=
                        sac_params_.lo_start_iterations &&
                    sac_params_.use_lo) {
                    std::vector<Datum> inliers;
                    GetInlierDatum(data, inlier_indices, inliers);
                    if (!estimator_.RefineModel(inliers, best_model)) {
                        continue;
                    }
                    ++summary->num_lo_iterations;
                }

                // A better cost does not guarantee a higher inlier ratio (i.e,
                // the MLE case) so we only update the max iterations if the
                // number decreases.
                max_iterations = std::min(
                    ComputeMaxIterations(estimator_.SampleSize(), inlier_ratio,
                                         log_failure_prob),
                    max_iterations);

                VLOG(3) << "Inlier ratio = " << inlier_ratio
                        << " and max number of iterations = " << max_iterations;
            }
        }
    }

    // Compute the final inliers for the best model.
    const std::vector<double> best_residuals =
        estimator_.Residuals(data, *best_model);
    quality_measurement_->ComputeCost(best_residuals, &summary->inliers);

    if (sac_params_.use_lo) {
        std::vector<Datum> inliers;
        GetInlierDatum(data, summary->inliers, inliers);
        estimator_.RefineModel(inliers, best_model);
        ++summary->num_lo_iterations;
    }

    const double inlier_ratio =
        static_cast<double>(summary->inliers.size()) / data.size();
    summary->confidence =
        1.0 - pow(1.0 - pow(inlier_ratio, estimator_.SampleSize()),
                  summary->num_iterations);

    return true;
}

template <class ModelEstimator>
void SampleConsensusEstimator<ModelEstimator>::GetInlierDatum(
    const std::vector<Datum>& data, const std::vector<int>& inlier_indices,
    std::vector<Datum>& inlier_datum) const
{
    inlier_datum.resize(inlier_indices.size());
    for (size_t i{0}; i < inlier_indices.size(); i++) {
        inlier_datum[i] = data[inlier_indices[i]];
    }
}

} // namespace thoht

﻿#pragma once

#include <vector>
#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace tl {

// Templated class for estimating a model for RANSAC. This class is purely a
// virtual class and should be implemented for the specific task that RANSAC is
// being used for. Two methods must be implemented: EstimateModel and Error. All
// other methods are optional, but will likely enhance the quality of the RANSAC
// output.
//
// NOTE: RANSAC, ARRSAC, and other solvers work best if Data and Model are
// lightweight classes or structs.
template <typename Data_t, typename Model_t>
class RansacModelEstimator
{
public:
    using Data = Data_t;
    using Model = Model_t;

    RansacModelEstimator() = default;
    virtual ~RansacModelEstimator() = default;

    // Get the minimum number of samples needed to generate a model.
    virtual size_t SampleSize() const = 0;

    // Given a set of data points, estimate the model. Users should implement
    // this function appropriately for the task being solved. Returns true for
    // successful model estimation (and outputs model), false for failed
    // estimation. Typically, this is a minimal set, but it is not required to
    // be.
    virtual bool EstimateModel(const std::vector<Data>& data,
                               std::vector<Model>* model) const = 0;

    // Estimate a model from a non-minimal sampling of the data. E.g. for a
    // line, use SVD on a set of points instead of constructing a line from two
    // points. By default, this simply implements the minimal case.
    virtual bool EstimateModelNonminimal(const std::vector<Data>& data,
                                         std::vector<Model>* model) const
    {
        return EstimateModel(data, model);
    }

    // Refine the model based on an updated subset of data, and a pre-computed
    // model. Can be optionally implemented.
    virtual bool RefineModel(const std::vector<Data>& data, double threshold,
                             Model* model) const
    {
        return true;
    }

    // Given a model and a data point, calculate the error. Users should
    // implement this function appropriately for the task being solved.
    virtual double Error(const Data& data, const Model& model) const = 0;

    // Compute the residuals of many data points. By default this is just a loop
    // that calls Error() on each data point, but this function can be useful if
    // the errors of multiple points may be estimated simultanesously (e.g.,
    // matrix multiplication to compute the reprojection error of many points at
    // once).
    virtual std::vector<double> Residuals(const std::vector<Data>& data,
                                          const Model& model) const
    {
        std::vector<double> residuals(data.size());
#pragma omp parallel for
        for (size_t i{0}; i < data.size(); i++) {
            residuals[i] = Error(data[i], model);
        }
        return residuals;
    }

    // Returns the set inliers of the data set based on the error threshold
    // provided.
    std::vector<size_t> GetInliers(const std::vector<Data>& data,
                                   const Model& model, double threshold) const
    {
        std::vector<size_t> inliers;
        inliers.reserve(data.size());
        for (size_t i{0}; i < data.size(); i++) {
            if (Error(data[i], model) < threshold) {
                inliers.emplace_back(i);
            }
        }
        return inliers;
    }

    // Enable a quick check to see if the model is valid. This can be a
    // geometric check or some other verification of the model structure.
    virtual bool ValidModel(const Model& model) const { return true; }
};

} // namespace tl

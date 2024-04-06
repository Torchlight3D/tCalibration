#pragma once

class Snr
{
public:
    Snr(double dark_mean = 0., double dark_sd = 1., double bright_mean = 1.,
        double bright_sd = 1.);

    double mean_cnr() const;
    double dark_cnr() const;
    double bright_cnr() const;
    double dark_snr() const;
    double bright_snr() const;

    double contrast() const;

    double oversampling() const;
    void set_oversampling(double val);

private:
    double dark_mean;
    double dark_sd;
    double bright_mean;
    double bright_sd;
    double contrast_val;
    double oversampling_val;
};

#pragma once

#include <cmath>
#include <vector>

template <int n>
class AFFT
{
public:
    AFFT()
    {
        int j = 1;
        // digit (bit) reverse
        for (int i = 1; i < n; i++) {
            if (i < j) {
                bitrev.push_back(std::make_pair(j, i));
            }
            int k = n / 2;
            while (k >= 1 && k < j) {
                j -= k;
                k /= 2;
            }
            j += k;
        }

        power = 0;
        unsigned int t = n / 2;
        while (t > 0) {
            power++;
            t >>= 1;
        }

        // choose fixed upper limit to help compiler
        cstab = std::vector<std::vector<double>>(power + 1,
                                                 std::vector<double>(n / 2, 0));
        int n2 = 1;
        for (unsigned int k = 2; k <= power; k++) {
            int n4 = n2;
            n2 = 2 * n4;
            int n1 = 2 * n2;
            double e = 2 * M_PI / double(n1);
            int idx = 0;
            for (int i = 1; i <= n; i += n1) {
                double a = e;
                for (int j = 1; j <= (n4 - 1); j++) {
                    cstab[k][idx++] = cos(a);
                    cstab[k][idx++] = sin(a);
                    a += e;
                }
            }
        }
    }

    // NB: Input is n real samples
    //     Output is [Re(0), Re(1), ..., Re(N/2), Im(N/2-1), ..., Im(1)]
    //     So DC is x[0], and complex frequency k is (x[k], x[N-k])
    void realfft(double* x)
    {
        x--; // simulate 1-based arrays

        // TODO: can we combine the first pass with the bit reversal?

        for (const auto& item : bitrev) {
            std::swap(x[item.first], x[item.second]);
        }

        // length 2 butterflies have special twiddle factors, do them first
        double* xp = x + 1;
        double* xp_sent = x + n;
        for (; xp <= xp_sent; xp += 2) {
            double xt = *xp;
            *(xp) = xt + *(xp + 1);
            *(xp + 1) = xt - *(xp + 1);
        }

        // other stages
        int n2 = 1;
        for (unsigned int k = 2; k <= power; k++) {
            int n4 = n2;
            n2 = n4 << 1;
            int n1 = n2 << 1;
            double* cs_ptr = cstab[k].data();

            double* xp = x + 1;       // start at x[1]
            double* xp_sent = xp + n; // stage sentinel

            for (; xp < xp_sent;) {
                double xt = *xp;
                *(xp) = xt + *(xp + n2);
                *(xp + n2) = xt - *(xp + n2);
                *(xp + n4 + n2) = -*(xp + n4 + n2);

                for (int j = 1; j <= (n4 - 1); j++) {
                    double* i1 = xp + j;
                    double* i2 = xp - j + n2;
                    double* i3 = i1 + n2;
                    double* i4 = i2 + n2;

                    double t1 =
                        *(i1 + n2) * (*(cs_ptr)) + *(i2 + n2) * (*(cs_ptr + 1));
                    double t2 =
                        *(i1 + n2) * (*(cs_ptr + 1)) - *(i2 + n2) * (*(cs_ptr));
                    *i4 = *i2 - t2;
                    *i3 = -*i2 - t2;

                    *i2 = *i1 - t1;
                    *i1 += t1;
                    cs_ptr += 2;
                }
                xp += n1;
            }
        }
    }

    std::vector<std::pair<int, int>> bitrev;
    std::vector<std::vector<double>> cstab;
    unsigned int power;
};

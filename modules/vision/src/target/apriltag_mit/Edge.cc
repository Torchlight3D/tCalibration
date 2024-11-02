#include "Edge.h"

#include "MathUtil.h"
#include "UnionFindSimple.h"

namespace AprilTags {

int Edge::edgeCost(float theta0, float theta1, float mag1)
{
    // mag0 was checked by the main routine so no need to
    // recheck here
    if (mag1 < minMag) {
        return -1;
    }

    const float thetaErr = std::abs(MathUtil::mod2pi(theta1 - theta0));
    if (thetaErr > maxEdgeCost) {
        return -1;
    }

    const float normErr = thetaErr / maxEdgeCost;
    return (int)(normErr * WEIGHT_SCALE);
}

void Edge::calcEdges(float theta0, int x, int y, const cv::Mat &theta,
                     const cv::Mat &mag, std::vector<Edge> &edges)
{
    int width = theta.cols;
    int thisPixel = y * width + x;

    // horizontal edge
    int cost1 =
        edgeCost(theta0, theta.at<float>(x + 1, y), mag.at<float>(x + 1, y));
    if (cost1 >= 0) {
        edges.emplace_back(thisPixel, y * width + x + 1, cost1);
    }

    // vertical edge
    int cost2 =
        edgeCost(theta0, theta.at<float>(x, y + 1), mag.at<float>(x, y + 1));
    if (cost2 >= 0) {
        edges.emplace_back(thisPixel, (y + 1) * width + x, cost2);
    }

    // downward diagonal edge
    int cost3 = edgeCost(theta0, theta.at<float>(x + 1, y + 1),
                         mag.at<float>(x + 1, y + 1));
    if (cost3 >= 0) {
        edges.emplace_back(thisPixel, (y + 1) * width + x + 1, cost3);
    }

    // updward diagonal edge
    int cost4 = (x == 0) ? -1
                         : edgeCost(theta0, theta.at<float>(x - 1, y + 1),
                                    mag.at<float>(x - 1, y + 1));
    if (cost4 >= 0) {
        edges.emplace_back(thisPixel, (y + 1) * width + x - 1, cost4);
    }
}

void Edge::mergeEdges(const std::vector<Edge> &edges, UnionFindSimple &uf,
                      float tmin[], float tmax[], float mmin[], float mmax[])
{
    for (size_t i = 0; i < edges.size(); i++) {
        int ida = edges[i].pixelIdxA;
        int idb = edges[i].pixelIdxB;

        ida = uf.getRepresentative(ida);
        idb = uf.getRepresentative(idb);

        if (ida == idb) {
            continue;
        }

        int sza = uf.getSetSize(ida);
        int szb = uf.getSetSize(idb);

        float tmina = tmin[ida], tmaxa = tmax[ida];
        float tminb = tmin[idb], tmaxb = tmax[idb];

        float costa = (tmaxa - tmina);
        float costb = (tmaxb - tminb);

        // bshift will be a multiple of 2pi that aligns the spans of 'b' with
        // 'a' so that we can properly take the union of them.
        float bshift =
            MathUtil::mod2pi((tmina + tmaxa) / 2, (tminb + tmaxb) / 2) -
            (tminb + tmaxb) / 2;

        float tminab = std::min(tmina, tminb + bshift);
        float tmaxab = std::max(tmaxa, tmaxb + bshift);

        // corner case that's probably not too useful to
        // handle correctly, oh well.
        tmaxab = std::min(tmaxab, tminab + 2 * (float)M_PI);

        float mminab = std::min(mmin[ida], mmin[idb]);
        float mmaxab = std::max(mmax[ida], mmax[idb]);

        // merge these two clusters?
        float costab = (tmaxab - tminab);
        if (costab <=
                (std::min(costa, costb) + Edge::thetaThresh / (sza + szb)) &&
            (mmaxab - mminab) <=
                std::min(mmax[ida] - mmin[ida], mmax[idb] - mmin[idb]) +
                    Edge::magThresh / (sza + szb)) {
            int idab = uf.connectNodes(ida, idb);

            tmin[idab] = tminab;
            tmax[idab] = tmaxab;

            mmin[idab] = mminab;
            mmax[idab] = mmaxab;
        }
    }
}

} // namespace AprilTags

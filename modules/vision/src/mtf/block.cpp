#include "block.h"

#include "common_types.h"
#include "edgeinfo.h"

Block::Block()
    : rect(Mrectangle()),
      mtf50(4, 0.0),
      quality(4, 0.0),
      sfr(4),
      esf(4),
      centroid(0, 0),
      area(0.0),
      valid(true),
      line_deviation(4, cv::Point3d(0, 0, 1)),
      snr(4),
      scansets(4),
      chromatic_aberration(4,
                           cv::Point2d(Edge_info::nodata, Edge_info::nodata)),
      edge_model(4),
      valid_edge(4, false),
      edge_length(4, 0.0)
{
}

Block::Block(const Mrectangle &in_rect)
    : rect(in_rect),
      mtf50(4, 0.0),
      quality(4, 0.0),
      sfr(4),
      esf(4),
      centroid(0, 0),
      area(0.0),
      valid(true),
      line_deviation(in_rect.line_deviation),
      snr(4),
      scansets(4),
      chromatic_aberration(4,
                           cv::Point2d(Edge_info::nodata, Edge_info::nodata)),
      edge_model(4),
      valid_edge(4, false),
      edge_length(4, 0.0)
{
    size_t top_edge_idx = 0;
    size_t bot_edge_idx = 0;
    size_t left_edge_idx = 0;
    size_t right_edge_idx = 0;

    if (rect.centroids.size() != 4) {
        valid = false;
        return;
    }

    for (size_t i = 0; i < 4; i++) {
        centroid.x += get_edge_centroid(i).x;
        centroid.y += get_edge_centroid(i).y;

        if (get_edge_centroid(i).y < get_edge_centroid(top_edge_idx).y) {
            top_edge_idx = i;
        }

        if (get_edge_centroid(i).y > get_edge_centroid(bot_edge_idx).y) {
            bot_edge_idx = i;
        }

        if (get_edge_centroid(i).x < get_edge_centroid(left_edge_idx).x) {
            left_edge_idx = i;
        }

        if (get_edge_centroid(i).x > get_edge_centroid(right_edge_idx).x) {
            right_edge_idx = i;
        }
    }
    centroid.x /= 4;
    centroid.y /= 4;

    edge_lut[TOP] = top_edge_idx;
    edge_lut[BOTTOM] = bot_edge_idx;
    edge_lut[LEFT] = left_edge_idx;
    edge_lut[RIGHT] = right_edge_idx;

    cv::Point2d e1 =
        get_edge_centroid(edge_lut[BOTTOM]) - get_edge_centroid(edge_lut[TOP]);

    cv::Point2d e2 =
        get_edge_centroid(edge_lut[RIGHT]) - get_edge_centroid(edge_lut[LEFT]);

    area = sqrt(SQR(e1.x) + SQR(e1.y)) * sqrt(SQR(e2.x) + SQR(e2.y));
}

void Block::set_snr(size_t edge_number, const Snr &in_snr)
{
    snr[edge_number] = in_snr;
}

const Snr &Block::get_snr(size_t edge_number) const { return snr[edge_number]; }

void Block::set_sfr(size_t edge_number, const std::vector<double> &in_sfr)
{
    sfr[edge_number] =
        std::shared_ptr<std::vector<double>>(new std::vector<double>(in_sfr));
}

const std::vector<double> &Block::get_sfr(size_t edge_number) const
{
    return *sfr[edge_number];
}

void Block::set_esf(size_t edge_number, const std::vector<double> &in_esf)
{
    esf[edge_number] =
        std::shared_ptr<std::vector<double>>(new std::vector<double>(in_esf));
}

const std::vector<double> &Block::get_esf(size_t edge_number) const
{
    return *esf[edge_number];
}

const std::vector<cv::Point2d> &Block::get_ridge(size_t edge_number) const
{
    return edge_model[edge_number]->ridge;
}

void Block::set_normal(size_t edge_number, const cv::Point2d &rgrad)
{
    rect.normals[edge_number] = rgrad;
}

cv::Point2d Block::get_normal(size_t edge_number) const
{
    return rect.normals[edge_number];
}

cv::Point2d Block::get_edge(size_t edge_number) const
{
    assert(edge_number < 4);
    return rect.edges[edge_number];
}

cv::Point2d Block::get_corner(size_t edge_number) const
{
    assert(edge_number < 4);
    return rect.corners[edge_number];
}

double Block::get_edge_angle(size_t edge_number) const
{
    assert(edge_number < 4);
    return rect.thetas[edge_number];
}

cv::Point2d Block::get_edge_centroid(size_t edge_number) const
{
    assert(edge_number < 4);
    return rect.centroids[edge_number];
}

cv::Point2d Block::get_edge_centroid(edge_position ep) const
{
    auto it = edge_lut.find(ep);
    return rect.centroids[it->second];
}

void Block::set_mtf50_value(size_t edge_number, double mtf50_value,
                            double quality_value)
{
    assert(edge_number < 4);
    mtf50[edge_number] = mtf50_value;
    quality[edge_number] = quality_value;
}

double Block::get_mtf50_value(size_t edge_number) const
{
    assert(edge_number < 4);
    return mtf50[edge_number];
}

double Block::get_mtf50_value(edge_position ep) const
{
    auto it = edge_lut.find(ep);
    return get_mtf50_value(it->second);
}

cv::Point2d Block::get_centroid() const { return centroid; }

double Block::get_area() const { return area; }

double Block::get_quality(size_t edge_number) const
{
    return quality[edge_number];
}

double Block::get_quality(edge_position ep) const
{
    auto it = edge_lut.find(ep);
    return quality[it->second];
}

int Block::get_edge_index(edge_position ep) const
{
    auto it = edge_lut.find(ep);
    return it->second;
}

cv::Point3d Block::get_line_deviation(size_t edge_number) const
{
    assert(edge_number < 4);
    return line_deviation[edge_number];
}

void Block::set_line_deviation(size_t edge_number, const cv::Point3d &deviation)
{
    assert(edge_number < 4);
    line_deviation[edge_number] = deviation;
}

void Block::set_scanset(size_t edge_number,
                        const std::map<int, scanline> &scanset)
{
    assert(edge_number < 4);
    scansets[edge_number] = std::shared_ptr<std::map<int, scanline>>(
        new std::map<int, scanline>(scanset));
}

const std::map<int, scanline> &Block::get_scanset(size_t edge_number) const
{
    assert(edge_number < 4);
    return *scansets[edge_number];
}

void Block::set_ca(size_t edge_number, cv::Point2d ca)
{
    assert(edge_number < 4);
    chromatic_aberration[edge_number] = ca;
}

const cv::Point2d &Block::get_ca(size_t edge_number) const
{
    assert(edge_number < 4);
    return chromatic_aberration[edge_number];
}

void Block::set_edge_model(size_t edge_number,
                           const std::shared_ptr<Edge_model> &em)
{
    assert(edge_number < 4);
    edge_model[edge_number] = em;
}

Edge_model &Block::get_edge_model(size_t edge_number)
{
    assert(edge_number < 4);
    return *edge_model[edge_number];
}

bool Block::get_edge_valid(size_t edge_number) const
{
    assert(edge_number < 4);
    return valid_edge[edge_number];
}

void Block::set_edge_valid(size_t edge_number)
{
    assert(edge_number < 4);
    valid_edge[edge_number] = true;
}

double Block::get_edge_length(size_t edge_number) const
{
    assert(edge_number < 4);
    return edge_length[edge_number];
}

void Block::set_edge_length(size_t edge_number, double length)
{
    assert(edge_number < 4);
    edge_length[edge_number] = length;
}

bool Block::serialize(FILE *fout) const
{
    std::vector<cv::Point2d> edge_centroids(4);
    std::vector<double> edge_angle(4);
    std::vector<cv::Point2d> edge_snr(4);
    std::vector<double> geometric_length(4);

    for (size_t k = 0; k < 4; k++) {
        edge_centroids[k] = rect.get_centroid(k);
        edge_angle[k] = atan2(-get_normal(k).x, get_normal(k).y);
        edge_snr[k] = cv::Point2d(snr[k].mean_cnr(), snr[k].oversampling());
        std::vector<std::pair<double, size_t>> corner_dist(4);
        for (size_t j = 0; j < 4; j++) {
            corner_dist[j] = std::pair<double, size_t>(
                cv::norm(edge_centroids[k] - get_corner(j)), j);
        }
        std::sort(corner_dist.begin(), corner_dist.end());
        geometric_length[k] = cv::norm(get_corner(corner_dist[0].second) -
                                       get_corner(corner_dist[1].second));
    }

    return Edge_info::serialize(
        fout, edge_centroids, edge_angle, mtf50, quality, sfr, esf, edge_snr,
        chromatic_aberration, valid_edge, edge_length, geometric_length);
}

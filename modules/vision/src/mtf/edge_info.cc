#include "edge_info.h"
#include "logger.h"

bool Edge_info::serialize_header(FILE* fout, size_t valid_count,
                                 const Job_metadata& metadata)
{
    std::vector<char> buffer(5 * sizeof(uint32_t) + 2 * sizeof(double));

    char* obuf = buffer.data();
    write_uint32(&obuf, valid_count);
    write_double(&obuf, metadata.pixel_pitch);
    write_double(&obuf, metadata.mtf_contrast);
    write_uint32(&obuf, uint32_t(metadata.bayer));
    write_uint32(&obuf, uint32_t(metadata.channels));
    write_uint32(&obuf, uint32_t(metadata.image_width));
    write_uint32(&obuf, uint32_t(metadata.image_height));

    size_t nwritten = fwrite(buffer.data(), 1, buffer.size(), fout);

    if (nwritten != buffer.size()) {
        logger.error("Could not write header to edge info serialization "
                     "file, tried %ld bytes, only wrote %ld\n",
                     buffer.size(), nwritten);
        return false;
    }

    return true;
}

bool Edge_info::deserialize_header(FILE* fin, size_t& valid_count,
                                   Job_metadata& metadata)
{
    std::vector<char> buffer(5 * sizeof(uint32_t) + 2 * sizeof(double));

    size_t nread = fread(buffer.data(), 1, buffer.size(), fin);

    if (nread != buffer.size()) {
        logger.error("Could not read header from edge info serialization "
                     "file, tried %ld bytes, only wrote %ld\n",
                     buffer.size(), nread);
        return false;
    }

    char* ibuf = buffer.data();
    valid_count = read_uint32(&ibuf);
    metadata.pixel_pitch = read_double(&ibuf);
    metadata.mtf_contrast = read_double(&ibuf);
    metadata.bayer = Bayer::bayer_t(read_uint32(&ibuf));
    metadata.channels = read_uint32(&ibuf);
    metadata.image_width = read_uint32(&ibuf);
    metadata.image_height = read_uint32(&ibuf);

    return true;
}

bool Edge_info::serialize(
    FILE* fout, const std::vector<cv::Point2d>& in_centroid,
    const std::vector<double>& in_angle, const std::vector<double>& in_mtf50,
    const std::vector<double>& /*in_quality*/,
    const std::vector<std::shared_ptr<std::vector<double>>>& in_sfr,
    const std::vector<std::shared_ptr<std::vector<double>>>& in_esf,
    const std::vector<cv::Point2d> in_snr,
    const std::vector<cv::Point2d> in_chromatic_aberration,
    const std::vector<bool> valid_edge,
    const std::vector<double>& in_edge_length,
    const std::vector<double>& in_geometric_length)
{
    // there is no need to keep data grouped by block at this point, so each
    // record is independent
    std::vector<char> buffer(20 * 1024);
    for (size_t k = 0; k < in_centroid.size(); k++) {
        if (!valid_edge[k])
            continue;

        char* buf = buffer.data();

        write_double(&buf, in_centroid[k].x);
        write_double(&buf, in_centroid[k].y);
        write_double(&buf, in_angle[k]); // whole angle, not relative
        write_double(&buf, in_mtf50[k]); // MTF-XX, in c/p
        write_double(&buf, in_snr[k].x); // mean CNR
        write_double(&buf, in_snr[k].y); // oversampling factor
        write_double(&buf, in_chromatic_aberration[k].x); // CA red-green
        write_double(&buf, in_chromatic_aberration[k].y); // CA blue-green
        write_double(
            &buf,
            in_edge_length[k]); // edge length (effective ROI), in pixels
        write_double(
            &buf,
            in_geometric_length[k]); // geometric edge length, in pixels

        write_uint32(&buf, in_sfr[k]->size());
        for (size_t j = 0; j < in_sfr[k]->size(); j++) {
            write_float(&buf, (float)(*in_sfr[k])[j]);
        }

        write_uint32(&buf, in_esf[k]->size());
        for (size_t j = 0; j < in_esf[k]->size(); j++) {
            write_float(&buf, (float)(*in_esf[k])[j]);
        }

        size_t size_in_bytes = buf - buffer.data();
        size_t nwritten = fwrite(buffer.data(), 1, size_in_bytes, fout);
        if (nwritten != size_in_bytes) {
            logger.error("Could not write to edge info serialization file, "
                         "tried %ld bytes, only wrote %ld\n",
                         buffer.size(), size_in_bytes);
            return false;
        }
    }
    return true;
}

Edge_info Edge_info::deserialize(FILE* fin, bool& valid)
{
    Edge_info b;
    valid = false;

    std::vector<char> buffer(20 * 1024);

    char* ibuf = buffer.data();

    size_t nread = fread(ibuf, 1, 10 * sizeof(double), fin);
    if (nread != 10 * sizeof(double)) {
        logger.error("Could not read from edge info serialization file, "
                     "tried %ld bytes, only read %ld\n",
                     9 * sizeof(double), nread);
        return b;
    }

    b.centroid.x = read_double(&ibuf);
    b.centroid.y = read_double(&ibuf);
    b.angle = read_double(&ibuf);
    b.mtf50 = read_double(&ibuf);
    b.snr.x = read_double(&ibuf);
    b.snr.y = read_double(&ibuf);
    b.chromatic_aberration.x = read_double(&ibuf);
    b.chromatic_aberration.y = read_double(&ibuf);
    b.edge_length = read_double(&ibuf);
    b.geometric_edge_length = read_double(&ibuf);

    nread = fread(ibuf, 1, sizeof(uint32_t), fin);
    if (nread != sizeof(uint32_t)) {
        logger.error("%s\n", "Could not read from edge info serialization "
                             "file, SFR size read failed");
        return b;
    }

    size_t nsfr = read_uint32(&ibuf);
    if (nsfr > 128) {
        logger.error("Unexpected SFR length (%ld) in edge deserialization, "
                     "aborting\n",
                     nsfr);
        return b;
    }

    nread = fread(ibuf, 1, sizeof(float) * nsfr, fin);
    if (nread != sizeof(float) * nsfr) {
        logger.error("%s\n", "Could not read from edge info serialization "
                             "file, SFR read failed");
        return b;
    }

    b.sfr = std::shared_ptr<std::vector<double>>(
        new std::vector<double>(nsfr, 0.0));
    for (size_t j = 0; j < nsfr; j++) {
        (*b.sfr)[j] = read_float(&ibuf);
    }

    nread = fread(ibuf, 1, sizeof(uint32_t), fin);
    if (nread != sizeof(uint32_t)) {
        logger.error("%s\n", "Could not read from edge info serialization "
                             "file, ESF size read failed");
        return b;
    }

    size_t nesf = read_uint32(&ibuf);
    if (nsfr > 256) {
        logger.error("%s\n",
                     "Unexpected ESF length in edge deserialization, aborting");
        return b;
    }

    nread = fread(ibuf, 1, sizeof(float) * nesf, fin);
    if (nread != sizeof(float) * nesf) {
        logger.error("%s\n", "Could not read from edge info serialization "
                             "file, ESF read failed");
        return b;
    }

    b.esf = std::shared_ptr<std::vector<double>>(
        new std::vector<double>(nesf, 0.0));
    for (size_t j = 0; j < nesf; j++) {
        (*b.esf)[j] = read_float(&ibuf);
    }

    valid = true;
    return b;
}

void Edge_info::set_metadata(const Job_metadata& md) { metadata = md; }

uint32_t Edge_info::read_uint32(char** buffer)
{
    uint32_t val = 0;
    memcpy((char*)&val, *buffer, sizeof(uint32_t));
    *buffer += sizeof(uint32_t);
    return val;
}

void Edge_info::write_uint32(char** buffer, uint32_t val)
{
    memcpy(*buffer, (char*)&val, sizeof(uint32_t));
    *buffer += sizeof(uint32_t);
}

float Edge_info::read_float(char** buffer)
{
    float val = 0;
    memcpy((char*)&val, *buffer, sizeof(float));
    *buffer += sizeof(float);
    return val;
}

void Edge_info::write_float(char** buffer, float val)
{
    memcpy(*buffer, (char*)&val, sizeof(float));
    *buffer += sizeof(float);
}

double Edge_info::read_double(char** buffer)
{
    double val = 0;
    memcpy((char*)&val, *buffer, sizeof(double));
    *buffer += sizeof(double);
    return val;
}

void Edge_info::write_double(char** buffer, double val)
{
    memcpy(*buffer, (char*)&val, sizeof(double));
    *buffer += sizeof(double);
}

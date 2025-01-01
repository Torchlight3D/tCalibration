#pragma once

#include <cstdint>

#if defined(_WIN32) || defined(_WIN64) // Windows
#include <io.h>
#include <winsock.h>
#if defined(_WIN64)
struct mytimeval
{
    long long tv_sec;
    long long tv_usec;
};
#else
using mytimeval = struct timeval;
#endif
#else // Unix-like
#include <sys/time.h>
using mytimeval = struct timeval;
#endif

namespace tl {

// NOTE: structs in this namespace are copied from soc code. The best practice
// should be Embeded team offers a header for all the supported data type.
namespace soc {

#define HB_VIO_BUFFER_MAX_PLANES 3

typedef enum VIO_DATA_TYPE_S
{
    HB_VIO_IPU_DS0_DATA = 0,
    HB_VIO_IPU_DS1_DATA,
    HB_VIO_IPU_DS2_DATA,
    HB_VIO_IPU_DS3_DATA,
    HB_VIO_IPU_DS4_DATA,
    HB_VIO_IPU_US_DATA,           // 5
    HB_VIO_PYM_FEEDBACK_SRC_DATA, // for debug
    HB_VIO_PYM_DATA,
    HB_VIO_SIF_FEEDBACK_SRC_DATA,
    HB_VIO_SIF_RAW_DATA, // 9
    HB_VIO_SIF_YUV_DATA,
    HB_VIO_ISP_YUV_DATA, // 11 for debug, a process result for raw feedback
    HB_VIO_GDC_DATA,
    HB_VIO_GDC1_DATA,
    HB_VIO_IARWB_DATA,
    HB_VIO_GDC_FEEDBACK_SRC_DATA,
    HB_VIO_GDC1_FEEDBACK_SRC_DATA,
    HB_VIO_PYM_LAYER_DATA,
    HB_VIO_MD_DATA,
    HB_VIO_ISP_RAW_DATA,
    HB_VIO_PYM_COMMON_DATA,
    HB_VIO_PYM_DATA_V2,
    HB_VIO_CIM_RAW_DATA,
    HB_VIO_CIM_YUV_DATA,
    HB_VIO_EMBED_DATA,
    HB_VIO_IPU_ALL_CHN_DATA,
    HB_VIO_DATA_TYPE_MAX
} VIO_DATA_TYPE_E;

typedef enum buffer_state
{
    BUFFER_AVAILABLE,
    BUFFER_PROCESS,
    BUFFER_DONE,
    BUFFER_REPROCESS,
    BUFFER_USER,
    BUFFER_INVALID
} buffer_state_e;

typedef struct address_info_s
{
    uint16_t width;  // 2
    uint16_t height; //
    uint16_t stride_size;
    char* addr[HB_VIO_BUFFER_MAX_PLANES];
    uint64_t paddr[HB_VIO_BUFFER_MAX_PLANES];
} address_info_t;

typedef struct image_info_s
{
    uint16_t sensor_id;
    uint32_t pipeline_id;
    uint32_t frame_id;
    double time_stamp;   // HW time stamp
    struct mytimeval tv; // system time of hal get buf
    int buf_index;
    int img_format;
    int fd[HB_VIO_BUFFER_MAX_PLANES]; // ion buf fd
    uint32_t size[HB_VIO_BUFFER_MAX_PLANES];
    uint32_t planeCount;
    uint32_t dynamic_flag;
    uint32_t water_mark_line;
    VIO_DATA_TYPE_E data_type;
    buffer_state_e state;
} image_info_t;

typedef struct hb_vio_buffer_s
{
    image_info_t img_info;
    address_info_t img_addr;
} hb_vio_buffer_t;

struct imu_data_t
{
    double temp;
    double ax;
    double ay;
    double az;
    double gx;
    double gy;
    double gz;
    double stamp;
    unsigned int id;
};

inline constexpr auto kMinMonoDataSize = sizeof(hb_vio_buffer_t);
inline constexpr auto kMinStereoDataSize = 2 * sizeof(hb_vio_buffer_t);
inline constexpr auto kMinMotionDataSize = sizeof(imu_data_t);

} // namespace soc
} // namespace tl

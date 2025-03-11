#include "ISP.h"

#include <rkaiq/uAPI/rk_aiq_user_api_imgproc.h>
#include <rkaiq/uAPI/rk_aiq_user_api_sysctl.h>
#include <rkaiq/uAPI/rk_aiq_user_api_af.h>

#define MAX_AIQ_CTX 3

namespace vision {

ISP::ISP(uint8_t cam_id, rk_aiq_working_mode_t wdr_mode, std::string iq_files_path):
    Logger("ISP"),
    iq_files_path(iq_files_path),
    cam_id(cam_id),
    wdr_mode(wdr_mode)
{
    aiq_ctx = nullptr;
    aiq_static_info = {0};
}

ISP::~ISP()
{
    rk_aiq_uapi_sysctl_stop(aiq_ctx, false);
    rk_aiq_uapi_sysctl_deinit(aiq_ctx);
}

rc_t ISP::Start() 
{
    if (cam_id > MAX_AIQ_CTX) {
        printf("%s : CamId is over 3\n", __FUNCTION__);
        return RC_ERROR_INVALID_ARG;
    }
    LogDebug("Starting ISP for cam_id %d", cam_id);

    if (rk_aiq_uapi_sysctl_enumStaticMetas(cam_id, &aiq_static_info) != 0) {
        LogError("Failed to get static info for cam_id %d", cam_id);
    }
    LogInfo("Sensor name: %s", aiq_static_info.sensor_info.sensor_name);

    std::string hdr = std::to_string(static_cast<int>(wdr_mode));
    setenv("HDR_MODE", hdr.c_str(), 1);

    // TODO: Add callbacks for err handling
    aiq_ctx = rk_aiq_uapi_sysctl_init(aiq_static_info.sensor_info.sensor_name, iq_files_path.c_str(), nullptr, nullptr);
    if (!aiq_ctx) {
        LogError("Failed to initialize AIQ context for cam_id %d", cam_id);
        return RC_ERROR;
    }
    rk_aiq_af_attrib_t af_attrib;
    rk_aiq_user_api_af_GetAttrib(aiq_ctx, &af_attrib);
    af_attrib.AfMode = RKAIQ_AF_MODE_NOT_SET;
    rk_aiq_user_api_af_SetAttrib(aiq_ctx, &af_attrib);

    if (rk_aiq_uapi_sysctl_prepare(aiq_ctx, 0, 0, wdr_mode)) {
        LogError("rkaiq engine prepare failed !\n");
        return RC_ERROR;
    }

    if (rk_aiq_uapi_sysctl_start(aiq_ctx)) {
        LogError("rk_aiq_uapi_sysctl_start  failed\n");
        return RC_ERROR;
    }

    hw_camera_found = true;
    return RC_OK;
}

}

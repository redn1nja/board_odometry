#pragma once

#include <string>

#include "ret_codes.h"
#include "Logger.h"

#include <rkaiq/common/rk_aiq_comm.h>
#include <rkaiq/common/rk_aiq_types.h>
#include <rkaiq/uAPI/rk_aiq_user_api_sysctl.h>

#define DEFAULT_PATH_TO_IQ_FILE "/etc/iqfiles"

namespace vision {

class ISP: public Logger {
    public:
        ISP(uint8_t cam_id = 0, rk_aiq_working_mode_t wdr_mode = RK_AIQ_WORKING_MODE_NORMAL, std::string iq_files_path = DEFAULT_PATH_TO_IQ_FILE);
        ~ISP();
        rc_t Start();
        rc_t Stop();
    private:
        std::string iq_files_path;
        uint8_t cam_id;
        rk_aiq_working_mode_t wdr_mode;
        rk_aiq_sys_ctx_t* aiq_ctx;
        rk_aiq_static_info_t aiq_static_info;
        bool hw_camera_found;
};

}
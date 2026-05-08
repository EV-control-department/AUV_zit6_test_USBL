#include "AppMain.hpp"
#include "MicroRosTask.hpp"
#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include "SoftWatchdog.hpp"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <zit6_interfaces/msg/zit_setpoint.h>
#include <zit6_interfaces/msg/zit_status.h>
#include <zit6_interfaces/msg/zit_pid.h>
#include <zit6_interfaces/msg/zit_pid_status.h>
#include <zit6_interfaces/srv/update_params.h>
#include <zit6_interfaces/srv/get_params.h>
#include <rosidl_runtime_c/string_functions.h>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cstdio>
#include "cJSON.h"

extern "C" {
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *ptr, void *state);
void *microros_reallocate(void *ptr, size_t new_size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_t_of_element, void *state);
}

static void *cjson_malloc(size_t size) { return microros_allocate(size, NULL); }
static void cjson_free(void *ptr) { microros_deallocate(ptr, NULL); }

// 实例指针定义
MicroRosTask *MicroRosTask::instance_ = nullptr;

// (on-target debug UART removed — no debug pins available)

// --- 成员回调实现 ---
void MicroRosTask::onZitPid(const void *msgin) {
    const auto *msg = (const zit6_interfaces__msg__ZitPid *)msgin;
    if (!std::isfinite(msg->kp) || !std::isfinite(msg->ki) || !std::isfinite(msg->kd)) return;
    auv::control::chassis.configurePID(msg->axis, msg->is_pos_ring, msg->kp, msg->ki, msg->kd, msg->i_limit, msg->out_limit);
    if (msg->is_pos_ring) auv::control::chassis.configureProfile(msg->axis, msg->max_v, msg->max_a);
}

void MicroRosTask::onZitSetpoint(const void *msgin) {
    const auto *msg = (const zit6_interfaces__msg__ZitSetpoint *)msgin;
    last_received_seq = msg->seq;
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->z) || !std::isfinite(msg->yaw)) return;
    if (!is_system_armed) return;

    uint32_t level = msg->control_key & 0x03;
    bool is_body = (msg->control_key & 0x10) != 0;
    bool is_inc = (msg->control_key & 0x20) != 0;
    uint32_t mask = msg->type_mask;
    float val[4] = {msg->x, msg->y, msg->z, msg->yaw};

    auto nav = auv::shared::snapshotNavState();
    if ((level == 0 || level == 1) && !auv::shared::isNavigationValid(nav)) return;

    if (level == 2) { // FORCE
        float fx = val[0], fy = val[1];
        if (!is_body) auv::control::CoordinateManager::worldToBody(nav.yaw, val[0], val[1], fx, fy);
        float forces[4] = {fx, fy, val[2], val[3]};
        taskENTER_CRITICAL();
        auv::control::chassis.setActuatorForces(forces);
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        auv::control::chassis.setControlLevel(auv::common::ControlLevel::ACTUATOR, actual_p, actual_v);
        taskEXIT_CRITICAL();
    } else if (level == 1) { // VEL
        taskENTER_CRITICAL();
        for (int i = 0; i < 4; i++) target_p[i] = val[i];
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        auv::control::chassis.setControlLevel(auv::common::ControlLevel::VELOCITY, actual_p, actual_v);
        taskEXIT_CRITICAL();
    } else if (level == 0) { // POS
        taskENTER_CRITICAL();
        if (is_body && is_inc) {
            float wx, wy;
            auv::control::CoordinateManager::bodyToWorld(nav.yaw, val[0], val[1], wx, wy);
            if (mask & 0x01) target_p[0] = nav.x + wx;
            if (mask & 0x02) target_p[1] = nav.y + wy;
            if (mask & 0x04) target_p[2] = nav.z + val[2];
            if (mask & 0x08) target_p[3] = nav.yaw + val[3];
        } else {
            for (int i = 0; i < 4; i++) if (mask & (1 << i)) target_p[i] = val[i];
        }
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        auv::control::chassis.setControlLevel(auv::common::ControlLevel::POSITION, actual_p, actual_v);
        taskEXIT_CRITICAL();
    }
}

void MicroRosTask::onArmHeartbeat(const void *msgin) {
    const auto *msg = (const std_msgs__msg__UInt32 *)msgin;
    taskENTER_CRITICAL();
    last_arm_heartbeat_ms = HAL_GetTick();
    last_arm_heartbeat_data = msg->data;
    if (!is_system_armed) {
        if (arm_heartbeat_count == 0) arm_start_ms = last_arm_heartbeat_ms;
        arm_heartbeat_count++;
    }
    taskEXIT_CRITICAL();
}

void MicroRosTask::onInsCommand(const void *msgin) {
    const auto *message = static_cast<const std_msgs__msg__UInt8 *>(msgin);
    if (message == nullptr) return;
    switch (message->data) {
        case 1: auv::device::ins_driver.setDvlPower(true); break;
        case 2: auv::device::ins_driver.setDvlPower(false); break;
        case 3: auv::device::ins_driver.restart(); break;
        case 4: auv::device::ins_driver.resetPosition(); break;
        case 5: auv::device::ins_driver.setInitialPosition(45.7749, 126.6765); break;
    }
}

void MicroRosTask::onServoCmd(const void *msgin) {
    const auto *msg = (const std_msgs__msg__Float32 *)msgin;
    auv::device::motor_driver.setServoAngle(msg->data);
}

void MicroRosTask::onLedCmd(const void *msgin) {
    const auto *msg = (const std_msgs__msg__UInt8 *)msgin;
    auv::device::motor_driver.setLightState(msg->data);
}

// Helper: find matching closing brace for a block starting at the first '{' pointer
static const char * find_matching_brace(const char *open) {
    if (!open) return nullptr;
    const char *p = open;
    int depth = 0;
    while (*p) {
        if (*p == '{') depth++;
        else if (*p == '}') {
            depth--;
            if (depth == 0) return p + 1;
        }
        p++;
    }
    return nullptr;
}

// Helper: search for a double value for `key` inside region [start, end)
static bool find_double_in_region(const char *start, const char *end, const char *key, double *out) {
    if (!start || !end || !key) return false;
    const char *p = strstr(start, key);
    while (p && p < end) {
        const char *colon = strchr(p, ':');
        if (!colon || colon >= end) return false;
        char *endptr = nullptr;
        double v = strtod(colon + 1, &endptr);
        if (endptr != colon + 1) { *out = v; return true; }
        p = strstr(p + 1, key);
    }
    return false;
}

// Helper: append a C-string into buffer at pos, ensuring null-termination
static size_t append_str(char *buf, size_t bufsize, size_t pos, const char *s) {
    if (!buf || bufsize == 0 || pos >= bufsize - 1) return pos;
    if (!s) return pos;
    size_t avail = bufsize - pos;
    size_t len = strlen(s);
    size_t cp = (len < avail - 1) ? len : (avail - 1);
    memcpy(buf + pos, s, cp);
    pos += cp;
    buf[pos] = '\0';
    return pos;
}

// Helper: append a float to buffer with fixed decimal precision without using %f
static size_t append_float_fixed(char *buf, size_t bufsize, size_t pos, float v, int prec) {
    if (!buf || bufsize == 0 || pos >= bufsize - 1) return pos;
    if (isnan(v)) {
        return append_str(buf, bufsize, pos, "nan");
    }
    if (isinf(v)) {
        if (v < 0) return append_str(buf, bufsize, pos, "-inf");
        return append_str(buf, bufsize, pos, "inf");
    }
    bool neg = false;
    if (v < 0.0f) { neg = true; v = -v; }
    unsigned long scale = 1UL;
    for (int i = 0; i < prec; ++i) scale *= 10UL;
    unsigned long scaled = (unsigned long)(v * (float)scale + 0.5f);
    unsigned long intpart = scaled / scale;
    unsigned long frac = scaled % scale;

    char tmp[48];
    int n = snprintf(tmp, sizeof(tmp), "%s%lu", neg ? "-" : "", intpart);
    if (n > 0) {
        size_t avail = bufsize - pos;
        size_t cp = ((size_t)n < avail - 1) ? (size_t)n : (avail - 1);
        memcpy(buf + pos, tmp, cp);
        pos += cp;
        buf[pos] = '\0';
    }
    if (prec > 0 && pos < bufsize - 1) {
        buf[pos++] = '.';
        // write fractional part with leading zeros
        // compute digits of frac
        char digits[32];
        int d = 0;
        if (frac == 0) {
            digits[d++] = '0';
        } else {
            unsigned long t = frac;
            while (t > 0 && d < (int)sizeof(digits)) { digits[d++] = (char)('0' + (t % 10)); t /= 10; }
        }
        int pad = prec - d;
        while (pad-- > 0 && pos < bufsize - 1) buf[pos++] = '0';
        for (int i = d - 1; i >= 0 && pos < bufsize - 1; --i) buf[pos++] = digits[i];
        buf[pos] = '\0';
    }
    return pos;
}

void MicroRosTask::onUpdateParams(const void *reqin, rmw_request_id_t *req_id, void *resin) {
    auto *res = static_cast<zit6_interfaces__srv__UpdateParams_Response *>(resin);
    if (!res) return;
    res->success = false;
    rosidl_runtime_c__String__assign(&res->message, "");
    if (!reqin) {
        rosidl_runtime_c__String__assign(&res->message, "null request");
        return;
    }
    const auto *req = static_cast<const zit6_interfaces__srv__UpdateParams_Request *>(reqin);

    // avoid blocking UART in service callback: do not perform blocking dbg serial transmit here

    // start from current chassis config (read axis 0 as representative)
    auv::config::ChassisConfig cfg;
    float v = 0.0f, a = 0.0f;
    auv::control::chassis.getProfileLimits(0, v, a);
    cfg.profile.default_max_v = v;
    cfg.profile.default_max_a = a;
    auto p_cfg = auv::control::chassis.getPIDConfig(0, true);
    cfg.pos_pid.kp = p_cfg.kp; cfg.pos_pid.ki = p_cfg.ki; cfg.pos_pid.kd = p_cfg.kd; cfg.pos_pid.i_limit = p_cfg.i_limit; cfg.pos_pid.output_limit = p_cfg.output_limit; cfg.pos_pid.dt = p_cfg.dt;
    auto vel_cfg = auv::control::chassis.getPIDConfig(0, false);
    cfg.vel_pid.kp = vel_cfg.kp; cfg.vel_pid.ki = vel_cfg.ki; cfg.vel_pid.kd = vel_cfg.kd; cfg.vel_pid.i_limit = vel_cfg.i_limit; cfg.vel_pid.output_limit = vel_cfg.output_limit; cfg.vel_pid.dt = vel_cfg.dt;

    // If JSON provided, prefer parsing it (use embedded cjson_min)
    if (req->json.data && req->json.data[0] != '\0') {
        const char *json = req->json.data;
        cJSON *root = cJSON_Parse(json);
        if (!root) {
            rosidl_runtime_c__String__assign(&res->message, "json parse error");
            res->success = false;
            return;
        }
        cJSON *chassis = cJSON_GetObjectItemCaseSensitive(root, "chassis");
        cJSON *ctx = chassis ? chassis : root;
        bool updated = false;

        cJSON *profile = cJSON_GetObjectItemCaseSensitive(ctx, "profile");
        if (profile) {
            cJSON *j = cJSON_GetObjectItemCaseSensitive(profile, "default_max_v");
            if (j && cJSON_IsNumber(j)) { cfg.profile.default_max_v = (float)cJSON_GetNumberValue(j); updated = true; }
            j = cJSON_GetObjectItemCaseSensitive(profile, "default_max_a");
            if (j && cJSON_IsNumber(j)) { cfg.profile.default_max_a = (float)cJSON_GetNumberValue(j); updated = true; }
        }

        cJSON *pid = cJSON_GetObjectItemCaseSensitive(ctx, "pid");
        if (pid) {
            cJSON *pos = cJSON_GetObjectItemCaseSensitive(pid, "pos");
            if (pos) {
                cJSON *j = cJSON_GetObjectItemCaseSensitive(pos, "kp"); if (j && cJSON_IsNumber(j)) { cfg.pos_pid.kp = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(pos, "ki"); if (j && cJSON_IsNumber(j)) { cfg.pos_pid.ki = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(pos, "kd"); if (j && cJSON_IsNumber(j)) { cfg.pos_pid.kd = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(pos, "i_limit"); if (j && cJSON_IsNumber(j)) { cfg.pos_pid.i_limit = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(pos, "output_limit"); if (j && cJSON_IsNumber(j)) { cfg.pos_pid.output_limit = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(pos, "dt"); if (j && cJSON_IsNumber(j)) { cfg.pos_pid.dt = (float)cJSON_GetNumberValue(j); updated = true; }
            }
            cJSON *vel = cJSON_GetObjectItemCaseSensitive(pid, "vel");
            if (vel) {
                cJSON *j = cJSON_GetObjectItemCaseSensitive(vel, "kp"); if (j && cJSON_IsNumber(j)) { cfg.vel_pid.kp = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(vel, "ki"); if (j && cJSON_IsNumber(j)) { cfg.vel_pid.ki = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(vel, "kd"); if (j && cJSON_IsNumber(j)) { cfg.vel_pid.kd = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(vel, "i_limit"); if (j && cJSON_IsNumber(j)) { cfg.vel_pid.i_limit = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(vel, "output_limit"); if (j && cJSON_IsNumber(j)) { cfg.vel_pid.output_limit = (float)cJSON_GetNumberValue(j); updated = true; }
                j = cJSON_GetObjectItemCaseSensitive(vel, "dt"); if (j && cJSON_IsNumber(j)) { cfg.vel_pid.dt = (float)cJSON_GetNumberValue(j); updated = true; }
            }
        }

        if (updated) {
            taskENTER_CRITICAL();
            auv::control::chassis.applyConfig(cfg);
            taskEXIT_CRITICAL();
            planner_replan_flag = true;
            res->success = true;
            rosidl_runtime_c__String__assign(&res->message, "ok");
            cJSON_Delete(root);
            return;
        }
        cJSON_Delete(root);
    }

    // Otherwise, try paths/values sequence update (simple mapping)
    if (req->paths.data && req->values.data && req->paths.size > 0 && req->values.size == req->paths.size) {
        for (size_t i = 0; i < req->paths.size; ++i) {
            const char *path = req->paths.data[i].data;
            const char *value = req->values.data[i].data;
            if (!path || !value) continue;
            // example supported paths (exact match)
            if (strcmp(path, "chassis.profile.default_max_v") == 0) {
                float fv = strtof(value, nullptr);
                cfg.profile.default_max_v = fv;
            } else if (strcmp(path, "chassis.profile.default_max_a") == 0) {
                float fa = strtof(value, nullptr);
                cfg.profile.default_max_a = fa;
            } else if (strcmp(path, "chassis.pid.pos.kp") == 0) cfg.pos_pid.kp = strtof(value, nullptr);
            else if (strcmp(path, "chassis.pid.pos.ki") == 0) cfg.pos_pid.ki = strtof(value, nullptr);
            else if (strcmp(path, "chassis.pid.pos.kd") == 0) cfg.pos_pid.kd = strtof(value, nullptr);
            else if (strcmp(path, "chassis.pid.vel.kp") == 0) cfg.vel_pid.kp = strtof(value, nullptr);
            else if (strcmp(path, "chassis.pid.vel.ki") == 0) cfg.vel_pid.ki = strtof(value, nullptr);
            else if (strcmp(path, "chassis.pid.vel.kd") == 0) cfg.vel_pid.kd = strtof(value, nullptr);
        }
        taskENTER_CRITICAL();
        auv::control::chassis.applyConfig(cfg);
        taskEXIT_CRITICAL();
        planner_replan_flag = true;
        res->success = true;
        rosidl_runtime_c__String__assign(&res->message, "ok");
        return;
    }

    // Provide richer diagnostic info when no-op: return json size/cap/pointer and first bytes
    {
        size_t json_strlen = 0;
        size_t json_size = 0;
        size_t json_cap = 0;
        uintptr_t json_ptr = 0;
        char head[129] = "";
        if (req->json.data) {
            json_ptr = (uintptr_t)req->json.data;
            json_size = req->json.size;
            json_cap = req->json.capacity;
            // try to copy up to 128 bytes (not assuming NUL termination)
            size_t cp = (json_size < (sizeof(head)-1)) ? json_size : (sizeof(head)-1);
            if (cp > 0) {
                memcpy(head, req->json.data, cp);
                head[cp] = '\0';
            }
            // compute strlen of copied head (it's NUL-terminated at head[cp])
            if (cp > 0) json_strlen = strlen(head);
        }
        size_t paths_cnt = req->paths.size;
        size_t vals_cnt = req->values.size;
        char diag[256];
        int n = snprintf(diag, sizeof(diag), "json_len=%lu json_size=%lu cap=%lu ptr=0x%08lx paths=%lu values=%lu head=%s",
            (unsigned long)json_strlen, (unsigned long)json_size, (unsigned long)json_cap, (unsigned long)json_ptr, (unsigned long)paths_cnt, (unsigned long)vals_cnt, head);
        // avoid blocking UART in callback; copy diagnostic into response message
        if (n > 0) rosidl_runtime_c__String__assign(&res->message, diag);
        else rosidl_runtime_c__String__assign(&res->message, "no-op");
        res->success = false;
    }
}

void MicroRosTask::onGetParams(const void *reqin, rmw_request_id_t *req_id, void *resin) {
    (void)req_id;
    auto *res = static_cast<zit6_interfaces__srv__GetParams_Response *>(resin);
    if (!res) return;

    // current representative values (axis 0)
    float v = 0.0f, a = 0.0f;
    auv::control::chassis.getProfileLimits(0, v, a);
    auto p_cfg = auv::control::chassis.getPIDConfig(0, true);
    auto vel_cfg = auv::control::chassis.getPIDConfig(0, false);

    const auto *req = static_cast<const zit6_interfaces__srv__GetParams_Request *>(reqin);
    char buf[512];
    size_t pos = 0;

    // If no paths requested, return full minimal JSON (backwards compatible)
    if (!req || req->paths.size == 0 || req->paths.data == NULL) {
        pos = append_str(buf, sizeof(buf), pos, "{\"chassis\":{\"profile\":{\"default_max_v\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, v, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"default_max_a\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, a, 6);
        pos = append_str(buf, sizeof(buf), pos, "},\"pid\":{\"pos\":{\"kp\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.kp, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"ki\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.ki, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"kd\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.kd, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"i_limit\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.i_limit, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"output_limit\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.output_limit, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"dt\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.dt, 6);
        pos = append_str(buf, sizeof(buf), pos, "},\"vel\":{\"kp\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.kp, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"ki\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.ki, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"kd\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.kd, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"i_limit\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.i_limit, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"output_limit\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.output_limit, 6);
        pos = append_str(buf, sizeof(buf), pos, ",\"dt\":");
        pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.dt, 6);
        pos = append_str(buf, sizeof(buf), pos, "}}}");

        if (pos > 0 && pos < sizeof(buf)) {
            if (res->config_json.data && res->config_json.capacity >= pos + 1) {
                memcpy(res->config_json.data, buf, pos);
                res->config_json.data[pos] = '\0';
                res->config_json.size = pos;
            } else {
                rosidl_runtime_c__String__assign(&res->config_json, buf);
            }
            res->success = true;
        } else {
            if (res->config_json.data && res->config_json.capacity >= 3) {
                memcpy(res->config_json.data, "{}", 2);
                res->config_json.data[2] = '\0';
                res->config_json.size = 2;
            } else {
                rosidl_runtime_c__String__assign(&res->config_json, "{}");
            }
            res->success = false;
        }
        return;
    }

    // Parse requested paths and build minimal JSON containing only requested fields
    bool any = false;
    bool want_profile_v = false, want_profile_a = false;
    bool want_pos_kp = false, want_pos_ki = false, want_pos_kd = false, want_pos_i_limit = false, want_pos_output = false, want_pos_dt = false;
    bool want_vel_kp = false, want_vel_ki = false, want_vel_kd = false, want_vel_i_limit = false, want_vel_output = false, want_vel_dt = false;

    for (size_t i = 0; i < req->paths.size; ++i) {
        if (!req->paths.data) break;
        const rosidl_runtime_c__String *s = &req->paths.data[i];
        const char *path = (s && s->data) ? s->data : NULL;
        if (!path) continue;
        if (strcmp(path, "chassis.profile.default_max_v") == 0) { want_profile_v = true; any = true; }
        else if (strcmp(path, "chassis.profile.default_max_a") == 0) { want_profile_a = true; any = true; }
        else if (strcmp(path, "chassis.pid.pos.kp") == 0) { want_pos_kp = true; any = true; }
        else if (strcmp(path, "chassis.pid.pos.ki") == 0) { want_pos_ki = true; any = true; }
        else if (strcmp(path, "chassis.pid.pos.kd") == 0) { want_pos_kd = true; any = true; }
        else if (strcmp(path, "chassis.pid.pos.i_limit") == 0) { want_pos_i_limit = true; any = true; }
        else if (strcmp(path, "chassis.pid.pos.output_limit") == 0) { want_pos_output = true; any = true; }
        else if (strcmp(path, "chassis.pid.pos.dt") == 0) { want_pos_dt = true; any = true; }
        else if (strcmp(path, "chassis.pid.vel.kp") == 0) { want_vel_kp = true; any = true; }
        else if (strcmp(path, "chassis.pid.vel.ki") == 0) { want_vel_ki = true; any = true; }
        else if (strcmp(path, "chassis.pid.vel.kd") == 0) { want_vel_kd = true; any = true; }
        else if (strcmp(path, "chassis.pid.vel.i_limit") == 0) { want_vel_i_limit = true; any = true; }
        else if (strcmp(path, "chassis.pid.vel.output_limit") == 0) { want_vel_output = true; any = true; }
        else if (strcmp(path, "chassis.pid.vel.dt") == 0) { want_vel_dt = true; any = true; }
    }

    if (!any) {
        // no recognized paths requested
        if (res->config_json.data && res->config_json.capacity >= 3) {
            memcpy(res->config_json.data, "{}", 2);
            res->config_json.data[2] = '\0';
            res->config_json.size = 2;
        } else {
            rosidl_runtime_c__String__assign(&res->config_json, "{}");
        }
        res->success = false;
        return;
    }

    // Build minimal JSON
    pos = append_str(buf, sizeof(buf), pos, "{");
    pos = append_str(buf, sizeof(buf), pos, "\"chassis\":{");
    bool first_chassis = true;

    // profile
    if (want_profile_v || want_profile_a) {
        pos = append_str(buf, sizeof(buf), pos, "\"profile\":{");
        bool first = true;
        if (want_profile_v) {
            pos = append_str(buf, sizeof(buf), pos, "\"default_max_v\":");
            pos = append_float_fixed(buf, sizeof(buf), pos, v, 6);
            first = false;
        }
        if (want_profile_a) {
            if (!first) pos = append_str(buf, sizeof(buf), pos, ",");
            pos = append_str(buf, sizeof(buf), pos, "\"default_max_a\":");
            pos = append_float_fixed(buf, sizeof(buf), pos, a, 6);
        }
        pos = append_str(buf, sizeof(buf), pos, "}");
        first_chassis = false;
    }

    // pid
    bool any_pos = want_pos_kp || want_pos_ki || want_pos_kd || want_pos_i_limit || want_pos_output || want_pos_dt;
    bool any_vel = want_vel_kp || want_vel_ki || want_vel_kd || want_vel_i_limit || want_vel_output || want_vel_dt;
    if (any_pos || any_vel) {
        if (!first_chassis) pos = append_str(buf, sizeof(buf), pos, ",");
        pos = append_str(buf, sizeof(buf), pos, "\"pid\":{");
        bool first_pid = true;
        if (any_pos) {
            pos = append_str(buf, sizeof(buf), pos, "\"pos\":{");
            bool first_pos = true;
            if (want_pos_kp) { pos = append_str(buf, sizeof(buf), pos, "\"kp\":"); pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.kp, 6); first_pos = false; }
            if (want_pos_ki) { if (!first_pos) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"ki\":"); pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.ki, 6); first_pos = false; }
            if (want_pos_kd) { if (!first_pos) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"kd\":"); pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.kd, 6); first_pos = false; }
            if (want_pos_i_limit) { if (!first_pos) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"i_limit\":"); pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.i_limit, 6); first_pos = false; }
            if (want_pos_output) { if (!first_pos) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"output_limit\":"); pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.output_limit, 6); first_pos = false; }
            if (want_pos_dt) { if (!first_pos) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"dt\":"); pos = append_float_fixed(buf, sizeof(buf), pos, p_cfg.dt, 6); first_pos = false; }
            pos = append_str(buf, sizeof(buf), pos, "}");
            first_pid = false;
        }
        if (any_vel) {
            if (!first_pid) pos = append_str(buf, sizeof(buf), pos, ",");
            pos = append_str(buf, sizeof(buf), pos, "\"vel\":{");
            bool first_vel = true;
            if (want_vel_kp) { pos = append_str(buf, sizeof(buf), pos, "\"kp\":"); pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.kp, 6); first_vel = false; }
            if (want_vel_ki) { if (!first_vel) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"ki\":"); pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.ki, 6); first_vel = false; }
            if (want_vel_kd) { if (!first_vel) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"kd\":"); pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.kd, 6); first_vel = false; }
            if (want_vel_i_limit) { if (!first_vel) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"i_limit\":"); pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.i_limit, 6); first_vel = false; }
            if (want_vel_output) { if (!first_vel) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"output_limit\":"); pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.output_limit, 6); first_vel = false; }
            if (want_vel_dt) { if (!first_vel) pos = append_str(buf, sizeof(buf), pos, ","); pos = append_str(buf, sizeof(buf), pos, "\"dt\":"); pos = append_float_fixed(buf, sizeof(buf), pos, vel_cfg.dt, 6); first_vel = false; }
            pos = append_str(buf, sizeof(buf), pos, "}");
        }
        pos = append_str(buf, sizeof(buf), pos, "}");
    }

    // close chassis and root
    pos = append_str(buf, sizeof(buf), pos, "}");
    pos = append_str(buf, sizeof(buf), pos, "}");

    if (pos > 0 && pos < sizeof(buf)) {
        if (res->config_json.data && res->config_json.capacity >= pos + 1) {
            memcpy(res->config_json.data, buf, pos);
            res->config_json.data[pos] = '\0';
            res->config_json.size = pos;
        } else {
            rosidl_runtime_c__String__assign(&res->config_json, buf);
        }
        res->success = true;
    } else {
        if (res->config_json.data && res->config_json.capacity >= 3) {
            memcpy(res->config_json.data, "{}", 2);
            res->config_json.data[2] = '\0';
            res->config_json.size = 2;
        } else {
            rosidl_runtime_c__String__assign(&res->config_json, "{}");
        }
        res->success = false;
    }
}

void MicroRosTask::cleanupMicroRos() {
    rclc_executor_fini(&executor_);
    // free pre-allocated request/response buffers
    if (get_req_.paths.data) {
        for (size_t _i = 0; _i < get_req_.paths.capacity; ++_i) {
            if (get_req_.paths.data[_i].data) {
                microros_deallocate(get_req_.paths.data[_i].data, NULL);
                get_req_.paths.data[_i].data = NULL;
                get_req_.paths.data[_i].capacity = 0;
                get_req_.paths.data[_i].size = 0;
            }
        }
        rosidl_runtime_c__String__Sequence_fini(&get_req_.paths);
    }
    if (update_req_.json.data) {
        microros_deallocate(update_req_.json.data, NULL);
        update_req_.json.data = NULL; update_req_.json.capacity = 0; update_req_.json.size = 0;
    }
    if (get_res_.config_json.data) {
        microros_deallocate(get_res_.config_json.data, NULL);
        get_res_.config_json.data = NULL; get_res_.config_json.capacity = 0; get_res_.config_json.size = 0;
    }
    rcl_service_fini(&update_params_srv_, &node_);
    rcl_service_fini(&get_params_srv_, &node_);
    rcl_publisher_fini(&pos_pub_, &node_);
    rcl_publisher_fini(&vel_pub_, &node_);
    rcl_publisher_fini(&thr_pub_, &node_);
    rcl_publisher_fini(&zithbt_pub_, &node_);
    rcl_publisher_fini(&status_pub_, &node_);
    rcl_publisher_fini(&pid_status_pub_, &node_);
    rcl_subscription_fini(&setpoint_sub_, &node_);
    rcl_subscription_fini(&arm_sub_, &node_);
    rcl_subscription_fini(&ins_cmd_sub_, &node_);
    rcl_subscription_fini(&pid_sub_, &node_);
    rcl_subscription_fini(&servo_sub_, &node_);
    rcl_subscription_fini(&led_sub_, &node_);
    rcl_node_fini(&node_);
    rclc_support_fini(&support_);
    memset(&support_, 0, sizeof(support_));
    memset(&node_, 0, sizeof(node_));
    memset(&executor_, 0, sizeof(executor_));
}

void MicroRosTask::run() {
    MicroRosTask::instance_ = this;
    rmw_uros_set_custom_transport(true, (void *)&huart2, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);
    
    // Initialize cJSON with micro-ROS allocators
    cJSON_Hooks hooks;
    hooks.malloc_fn = cjson_malloc;
    hooks.free_fn = cjson_free;
    cJSON_InitHooks(&hooks);

    rcutils_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate; allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate; allocator.zero_allocate = microros_zero_allocate;
    rcutils_set_default_allocator(&allocator);
    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();

    uint32_t last_hbt_pub_tick = 0, last_vel_pub_tick = 0, last_thr_pub_tick = 0, last_pos_pub_tick = 0, last_status_pub_tick = 0;
    enum uros_state { WAITING_AGENT, AGENT_CONNECTED } state = WAITING_AGENT;

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        if (state == WAITING_AGENT) {
            if (RCL_RET_OK == rmw_uros_ping_agent(200, 1)) {
                if (RCL_RET_OK == rclc_support_init(&support_, 0, NULL, &rcl_allocator)) {
                    rmw_uros_sync_session(100);
                    rclc_node_init_default(&node_, "zit6_node", "", &support_);
                    std_msgs__msg__Float32MultiArray__init(&pos_fb_msg_); pos_fb_msg_.data.data = pos_buf_; pos_fb_msg_.data.size = 4; pos_fb_msg_.data.capacity = 4;
                    std_msgs__msg__Float32MultiArray__init(&vel_fb_msg_); vel_fb_msg_.data.data = vel_buf_; vel_fb_msg_.data.size = 4; vel_fb_msg_.data.capacity = 4;
                    std_msgs__msg__Float32MultiArray__init(&thr_fb_msg_); thr_fb_msg_.data.data = thr_buf_; thr_fb_msg_.data.size = 4; thr_fb_msg_.data.capacity = 4;
                    std_msgs__msg__UInt32__init(&node_heartbeat_msg_);
                    std_msgs__msg__UInt32__init(&arm_msg_);
                    std_msgs__msg__UInt8__init(&ins_cmd_msg_);
                    std_msgs__msg__UInt8__init(&led_msg_);
                    std_msgs__msg__Float32__init(&servo_msg_);
                    zit6_interfaces__msg__ZitStatus__init(&status_msg_);

                    rclc_publisher_init_default(&pos_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/pos");
                    rclc_publisher_init_default(&vel_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/vel");
                    rclc_publisher_init_default(&thr_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/thr");
                    rclc_publisher_init_default(&zithbt_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/zit6/state/zithbt");
                    rclc_publisher_init_default(&status_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitStatus), "/zit6/state/status");
                    rclc_publisher_init_default(&pid_status_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitPidStatus), "/zit6/state/pid_status");

                    rclc_subscription_init_default(&led_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/zit6/cmd/light");
                    rclc_subscription_init_default(&servo_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/zit6/cmd/servo");
                    rclc_subscription_init_default(&setpoint_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitSetpoint), "/zit6/cmd/setpoint");
                    rclc_subscription_init_default(&arm_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/zit6/cmd/agxhbt");
                    rclc_subscription_init_default(&ins_cmd_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/zit6/cmd/ins");
                    rclc_subscription_init_default(&pid_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitPid), "/zit6/cmd/pid");

                    rclc_executor_init(&executor_, &support_.context, 20, &rcl_allocator);
                    rclc_executor_add_subscription(&executor_, &setpoint_sub_, &setpoint_msg_, &MicroRosTask::setpointCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &arm_sub_, &arm_msg_, &MicroRosTask::armCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &ins_cmd_sub_, &ins_cmd_msg_, &MicroRosTask::insCmdCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &pid_sub_, &pid_msg_, &MicroRosTask::zitPidCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &servo_sub_, &servo_msg_, &MicroRosTask::servoCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &led_sub_, &led_msg_, &MicroRosTask::ledCb, ON_NEW_DATA);
                    // Initialize services for parameter update and query
                    // Initialize update_params service and check return codes
                    {
                        rcl_ret_t rc = rclc_service_init_default(&update_params_srv_, &node_, ROSIDL_GET_SRV_TYPE_SUPPORT(zit6_interfaces, srv, UpdateParams), "/zit6/update_params");
                        (void)rc;
                        zit6_interfaces__srv__UpdateParams_Request__init(&update_req_);
                        // Pre-allocate buffer for incoming JSON string
                        update_req_.json.data = (char*)microros_allocate(1024, NULL);
                        update_req_.json.capacity = 1024;
                        update_req_.json.size = 0;

                        zit6_interfaces__srv__UpdateParams_Response__init(&update_res_);
                        rc = rclc_executor_add_service_with_request_id(&executor_, &update_params_srv_, &update_req_, &update_res_, &MicroRosTask::updateParamsCb);
                        (void)rc;
                    }

                    // Initialize get_params service and check return codes
                    {
                        rcl_ret_t rc = rclc_service_init_default(&get_params_srv_, &node_, ROSIDL_GET_SRV_TYPE_SUPPORT(zit6_interfaces, srv, GetParams), "/zit6/get_params");
                        (void)rc;
                        zit6_interfaces__srv__GetParams_Request__init(&get_req_);
                        // pre-initialize paths sequence to avoid NULL data pointer on incoming requests
                        rosidl_runtime_c__String__Sequence__init(&get_req_.paths, 8);
                        // allocate per-element string buffers to give rmw a place to write incoming path strings
                        if (get_req_.paths.data) {
                            for (size_t _i = 0; _i < get_req_.paths.capacity; ++_i) {
                                get_req_.paths.data[_i].data = (char*)microros_allocate(64, NULL);
                                get_req_.paths.data[_i].capacity = 64;
                                get_req_.paths.data[_i].size = 0;
                            }
                        }
                        zit6_interfaces__srv__GetParams_Response__init(&get_res_);
                        // Pre-allocate buffer for outgoing JSON string
                        get_res_.config_json.data = (char*)microros_allocate(1024, NULL);
                        get_res_.config_json.capacity = 1024;
                        get_res_.config_json.size = 0;
                        rc = rclc_executor_add_service_with_request_id(&executor_, &get_params_srv_, &get_req_, &get_res_, &MicroRosTask::getParamsCb);
                        (void)rc;
                    }
                    state = AGENT_CONNECTED;
                }
            } else vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            static uint32_t last_pid_pub_tick = 0;
            // Robust ping: 500ms timeout, 3 failures
            if (RCL_RET_OK != rmw_uros_ping_agent(500, 3)) {
                cleanupMicroRos(); state = WAITING_AGENT;
            } else {
                auv::device::SoftWatchdog::getInstance().feed(auv::device::SoftWatchdog::Component::MICROROS);
                rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
                if (now_ms - last_hbt_pub_tick >= 1000) { last_hbt_pub_tick = now_ms; node_heartbeat_msg_.data = now_ms; rcl_publish(&zithbt_pub_, &node_heartbeat_msg_, NULL); }
                if (now_ms - last_vel_pub_tick >= 20) { last_vel_pub_tick = now_ms; auto nav = auv::shared::snapshotNavState(); vel_buf_[0] = nav.vx; vel_buf_[1] = nav.vy; vel_buf_[2] = nav.vz; vel_buf_[3] = nav.vyaw; rcl_publish(&vel_pub_, &vel_fb_msg_, NULL); }
                if (now_ms - last_thr_pub_tick >= 33) { last_thr_pub_tick = now_ms; taskENTER_CRITICAL(); for (int i = 0; i < 4; i++) thr_buf_[i] = last_output_forces[i]; taskEXIT_CRITICAL(); rcl_publish(&thr_pub_, &thr_fb_msg_, NULL); }
                if (now_ms - last_pos_pub_tick >= 33) { last_pos_pub_tick = now_ms; auto nav = auv::shared::snapshotNavState(); pos_buf_[0] = nav.x; pos_buf_[1] = nav.y; pos_buf_[2] = nav.z; pos_buf_[3] = nav.yaw; rcl_publish(&pos_pub_, &pos_fb_msg_, NULL); }
                if (now_ms - last_status_pub_tick >= 100) {
                    last_status_pub_tick = now_ms; auv::common::NavState nav = auv::shared::snapshotNavState();
                    taskENTER_CRITICAL();
                    status_msg_.is_armed = is_system_armed; status_msg_.arm_mode = (uint8_t)last_arm_heartbeat_data; status_msg_.control_level = (uint8_t)auv::control::chassis.getControlLevel();
                    status_msg_.ins_state = nav.imu_state; status_msg_.navigation_ready = auv::shared::isNavigationValid(nav); for (int i = 0; i < 4; i++) status_msg_.forces[i] = last_output_forces[i];
                    status_msg_.cycle_time_ms = (float)last_dt_ms; status_msg_.battery_voltage = 0.0f; status_msg_.error_flags = 0;
                    taskEXIT_CRITICAL();
                    rcl_publish(&status_pub_, &status_msg_, NULL);
                }
                if (now_ms - last_pid_pub_tick >= 1000) {
                    last_pid_pub_tick = now_ms;
                    for (int i = 0; i < 4; i++) {
                        auto p_cfg = auv::control::chassis.getPIDConfig(i, true);
                        pid_status_msg_.pos_kp[i] = p_cfg.kp; auv::control::chassis.getProfileLimits(i, pid_status_msg_.pos_max_v[i], pid_status_msg_.pos_max_a[i]); pid_status_msg_.pos_out_limit[i] = p_cfg.output_limit;
                        auto v_cfg = auv::control::chassis.getPIDConfig(i, false);
                        pid_status_msg_.vel_kp[i] = v_cfg.kp; pid_status_msg_.vel_ki[i] = v_cfg.ki; pid_status_msg_.vel_kd[i] = v_cfg.kd; pid_status_msg_.vel_i_limit[i] = v_cfg.i_limit; pid_status_msg_.vel_out_limit[i] = v_cfg.output_limit;
                    }
                    rcl_publish(&pid_status_pub_, &pid_status_msg_, NULL);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void UserApp_MicroRosTask(void *argument) {
    (void)argument;
    MicroRosTask runner;
    runner.run();
}

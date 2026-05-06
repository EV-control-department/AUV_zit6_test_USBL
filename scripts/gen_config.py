import json
import os
import sys

def gen_header(json_path, out_path):
    with open(json_path, 'r') as f:
        config = json.load(f)

    # Accept either a file path or an output directory. If a directory is
    # provided, generated headers will be placed into that directory.
    if os.path.isdir(out_path):
        out_dir = out_path
        soft_watchdog_hpp = os.path.join(out_dir, 'SoftWatchdogConfig.hpp')
    else:
        # treat as a file path if it ends with .hpp, otherwise as dir
        if out_path.endswith('.hpp'):
            out_dir = os.path.dirname(out_path)
            soft_watchdog_hpp = out_path
        else:
            out_dir = out_path
            soft_watchdog_hpp = os.path.join(out_dir, 'SoftWatchdogConfig.hpp')

    os.makedirs(out_dir, exist_ok=True)

    sw_config = config.get('soft_watchdog', {})
    timeout = sw_config.get('timeout_ms', 3000)
    check_microros = str(sw_config.get('check_microros', True)).lower()
    check_ins = str(sw_config.get('check_ins', True)).lower()
    check_depth = str(sw_config.get('check_depth', True)).lower()

    content = f"""#ifndef __SOFT_WATCHDOG_CONFIG_HPP
#define __SOFT_WATCHDOG_CONFIG_HPP

#include <stdint.h>

namespace auv {{
namespace config {{

/**
 * @struct SoftWatchdogConfig
 * @brief 软件看门狗配置结构体 (Auto-generated from config.json)
 */
struct SoftWatchdogConfig {{
    uint32_t timeout_ms;
    bool check_microros;
    bool check_ins;
    bool check_depth;
}};

static const SoftWatchdogConfig DEFAULT_WATCHDOG_CONFIG = {{
    .timeout_ms = {timeout},
    .check_microros = {check_microros},
    .check_ins = {check_ins},
    .check_depth = {check_depth}
}};

}} // namespace config
}} // namespace auv

#endif
"""
    
    # Write SoftWatchdog header only if changed to avoid unnecessary recompilation
    old = None
    if os.path.exists(soft_watchdog_hpp):
        with open(soft_watchdog_hpp, 'r') as f:
            old = f.read()

    if old != content:
        with open(soft_watchdog_hpp, 'w') as f:
            f.write(content)

    # Also generate chassis config header in the same directory
    chassis_hpp = os.path.join(out_dir, 'ChassisConfig.hpp')

    chassis_cfg = config.get('chassis', {})
    profile = chassis_cfg.get('profile', {})
    def_max_v = profile.get('default_max_v', 0.5)
    def_max_a = profile.get('default_max_a', 0.2)

    pid = chassis_cfg.get('pid', {})
    pos = pid.get('pos', {})
    vel = pid.get('vel', {})

    pos_kp = pos.get('kp', 1.0)
    pos_ki = pos.get('ki', 0.0)
    pos_kd = pos.get('kd', 0.0)
    pos_i_limit = pos.get('i_limit', 1.0)
    pos_output_limit = pos.get('output_limit', 1.0)
    pos_dt = pos.get('dt', 0.01)

    vel_kp = vel.get('kp', 2.0)
    vel_ki = vel.get('ki', 0.5)
    vel_kd = vel.get('kd', 0.1)
    vel_i_limit = vel.get('i_limit', 1.0)
    vel_output_limit = vel.get('output_limit', 1.0)
    vel_dt = vel.get('dt', 0.01)

    chassis_content = f"""#ifndef __CHASSIS_CONFIG_HPP
#define __CHASSIS_CONFIG_HPP

#include <stdint.h>

namespace auv {{
namespace config {{

/**
 * @struct ChassisConfig
 * @brief 底盘默认参数配置 (Auto-generated from config.json)
 */
struct ChassisProfile {{
    float default_max_v;
    float default_max_a;
}};

struct PIDConfig {{
    float kp;
    float ki;
    float kd;
    float i_limit;
    float output_limit;
    float dt;
}};

struct ChassisConfig {{
    ChassisProfile profile;
    PIDConfig pos_pid;
    PIDConfig vel_pid;
}};

static const ChassisConfig DEFAULT_CHASSIS_CONFIG = {{
    .profile = {{ .default_max_v = {def_max_v}, .default_max_a = {def_max_a} }},
    .pos_pid = {{ .kp = {pos_kp}, .ki = {pos_ki}, .kd = {pos_kd}, .i_limit = {pos_i_limit}, .output_limit = {pos_output_limit}, .dt = {pos_dt} }},
    .vel_pid = {{ .kp = {vel_kp}, .ki = {vel_ki}, .kd = {vel_kd}, .i_limit = {vel_i_limit}, .output_limit = {vel_output_limit}, .dt = {vel_dt} }},
}};

}} // namespace config
}} // namespace auv

#endif
"""

    # Only write if changed
    write_chassis = True
    if os.path.exists(chassis_hpp):
        with open(chassis_hpp, 'r') as f:
            if f.read() == chassis_content:
                write_chassis = False

    if write_chassis:
        with open(chassis_hpp, 'w') as f:
            f.write(chassis_content)

    # Generate SensorsConfig.hpp
    z_data_sourse = config.get('z_data_sourse', 'use_ins_intergrated_z').lower()
    z_enum = 'USE_MS5837_Z' if z_data_sourse == 'use_ms5837_z' else 'USE_INS_INTEGRATED_Z'

    sensors_hpp = os.path.join(out_dir, 'SensorsConfig.hpp')
    sensors_content = f"""#ifndef __SENSORS_CONFIG_HPP
#define __SENSORS_CONFIG_HPP

namespace auv {{
namespace config {{

enum class ZDataSource {{
    USE_INS_INTEGRATED_Z,
    USE_MS5837_Z
}};

struct SensorsConfig {{
    ZDataSource z_data_source;
}};

static const SensorsConfig DEFAULT_SENSORS_CONFIG = {{
    .z_data_source = ZDataSource::{z_enum}
}};

}} // namespace config
}} // namespace auv

#endif
"""

    if os.path.exists(sensors_hpp):
        with open(sensors_hpp, 'r') as f:
            if f.read() == sensors_content:
                return

    with open(sensors_hpp, 'w') as f:
        f.write(sensors_content)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python gen_config.py <config.json> <output_dir_or.hpp>")
        sys.exit(1)
    gen_header(sys.argv[1], sys.argv[2])

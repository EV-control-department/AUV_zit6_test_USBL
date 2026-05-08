import json
import os
import sys

def get_cpp_type(val):
    if isinstance(val, bool):
        return "ParamType::BOOL", "bool"
    if isinstance(val, int):
        return "ParamType::UINT32", "uint32_t"
    if isinstance(val, float):
        return "ParamType::FLOAT", "float"
    if isinstance(val, str):
        # 特殊处理枚举
        if "use_ms5837_z" in val or "use_ins" in val:
             return "ParamType::ENUM_Z", "ZDataSource"
        return "ParamType::STRING", "const char*"
    return None, None

def collect_params(data, prefix=""):
    params = []
    for k, v in data.items():
        path = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            params.extend(collect_params(v, path))
        else:
            p_type, cpp_type = get_cpp_type(v)
            if p_type:
                params.append({
                    "path": path,
                    "key": k,
                    "type": p_type,
                    "cpp_type": cpp_type,
                    "val": v
                })
    return params

def gen_system_config(json_path, out_dir):
    with open(json_path, 'r') as f:
        config = json.load(f)

    os.makedirs(out_dir, exist_ok=True)
    header_path = os.path.join(out_dir, 'SystemConfig.hpp')

    # 生成结构体定义 (这里简化处理，手动定义核心结构以保证兼容性，元数据自动生成)
    # 实际上可以完全自动生成，但为了保持现有代码不崩溃，我们先定义好顶层结构
    
    content = """#ifndef __SYSTEM_CONFIG_HPP
#define __SYSTEM_CONFIG_HPP

#include <stdint.h>
#include <stddef.h>

namespace auv {
namespace config {

enum class ParamType {
    FLOAT,
    UINT32,
    INT32,
    BOOL,
    STRING,
    ENUM_Z
};

struct ParamMeta {
    const char* path;
    void* ptr;
    ParamType type;
};

// 传感器枚举定义
enum class ZDataSource {
    USE_INS_INTEGRATED_Z,
    USE_MS5837_Z
};

// --- 自动生成的配置结构体 ---

struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float i_limit;
    float output_limit;
    float dt;
};

struct ChassisProfile {
    float default_max_v;
    float default_max_a;
};

struct ChassisConfig {
    ChassisProfile profile;
    PIDConfig pos_pid;
    PIDConfig vel_pid;
};

struct SoftWatchdogConfig {
    uint32_t timeout_ms;
    bool check_microros;
    bool check_ins;
    bool check_depth;
};

struct InsConfig {
    float init_lat;
    float init_lon;
};

struct SensorsConfig {
    ZDataSource z_data_source;
};

struct SystemConfig {
    ChassisConfig chassis;
    InsConfig ins;
    SoftWatchdogConfig soft_watchdog;
    SensorsConfig sensors;
};

// 全局配置实例声明
extern SystemConfig sys_config;

// 参数注册表声明
extern const ParamMeta SYSTEM_PARAMS[];
extern const size_t SYSTEM_PARAMS_COUNT;

} // namespace config
} // namespace auv

#endif
"""

    # 生成注册表源文件 (SystemConfig.cpp)
    params = collect_params(config)
    
    cpp_content = f"""#include "SystemConfig.hpp"

namespace auv {{
namespace config {{

SystemConfig sys_config = {{
    .chassis = {{
        .profile = {{ {config['chassis']['profile']['default_max_v']}, {config['chassis']['profile']['default_max_a']} }},
        .pos_pid = {{ {config['chassis']['pid']['pos']['kp']}, {config['chassis']['pid']['pos']['ki']}, {config['chassis']['pid']['pos']['kd']}, {config['chassis']['pid']['pos']['i_limit']}, {config['chassis']['pid']['pos']['output_limit']}, {config['chassis']['pid']['pos']['dt']} }},
        .vel_pid = {{ {config['chassis']['pid']['vel']['kp']}, {config['chassis']['pid']['vel']['ki']}, {config['chassis']['pid']['vel']['kd']}, {config['chassis']['pid']['vel']['i_limit']}, {config['chassis']['pid']['vel']['output_limit']}, {config['chassis']['pid']['vel']['dt']} }}
    }},
    .ins = {{ {config['ins']['init_lat']}, {config['ins']['init_lon']} }},
    .soft_watchdog = {{ {config['soft_watchdog']['timeout_ms']}, {str(config['soft_watchdog']['check_microros']).lower()}, {str(config['soft_watchdog']['check_ins']).lower()}, {str(config['soft_watchdog']['check_depth']).lower()} }},
    .sensors = {{ ZDataSource::{ "USE_MS5837_Z" if config['z_data_sourse'] == 'use_ms5837_z' else "USE_INS_INTEGRATED_Z" } }}
}};

const ParamMeta SYSTEM_PARAMS[] = {{
"""
    
    for p in params:
        cpp_path = p['path']
        # 修正 JSON 路径到 C++ 成员路径的映射
        if cpp_path == "z_data_sourse": cpp_path = "sensors.z_data_source"
        elif "chassis.pid.pos." in cpp_path: cpp_path = cpp_path.replace("chassis.pid.pos.", "chassis.pos_pid.")
        elif "chassis.pid.vel." in cpp_path: cpp_path = cpp_path.replace("chassis.pid.vel.", "chassis.vel_pid.")
        elif "soft_watchdog." in cpp_path: cpp_path = cpp_path.replace("soft_watchdog.", "soft_watchdog.") # 无需变化但保持结构

        cpp_content += f'    {{"{p["path"]}", &sys_config.{cpp_path}, {p["type"]}}},\n'
    
    cpp_content += "    {NULL, NULL, ParamType::FLOAT}\n"
    cpp_content += "};\n\n"
    cpp_content += f"const size_t SYSTEM_PARAMS_COUNT = {len(params)};\n\n"
    cpp_content += "} // namespace config\n"
    cpp_content += "} // namespace auv\n"

    # 写入文件
    with open(header_path, 'w') as f:
        f.write(content)
    
    with open(os.path.join(out_dir, 'SystemConfig.cpp'), 'w') as f:
        f.write(cpp_content)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(1)
    gen_system_config(sys.argv[1], sys.argv[2])

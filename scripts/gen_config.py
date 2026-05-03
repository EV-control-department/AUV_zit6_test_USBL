import json
import os
import sys

def gen_header(json_path, hpp_path):
    with open(json_path, 'r') as f:
        config = json.load(f)
    
    sw_config = config.get('soft_watchdog', {})
    timeout = sw_config.get('timeout_ms', 3000)
    check_microros = str(sw_config.get('check_microros', True)).lower()
    check_ins = str(sw_config.get('check_ins', True)).lower()
    check_depth = str(sw_config.get('check_depth', True)).lower()

    content = f"""#ifndef __SOFT_WATCHDOG_CONFIG_HPP
#define __SOFT_WATCHDOG_CONFIG_HPP

#include <stdint.h>

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

#endif
"""
    
    # Only write if changed to avoid unnecessary recompilation
    if os.path.exists(hpp_path):
        with open(hpp_path, 'r') as f:
            if f.read() == content:
                return

    with open(hpp_path, 'w') as f:
        f.write(content)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python gen_config.py <config.json> <output.hpp>")
        sys.exit(1)
    gen_header(sys.argv[1], sys.argv[2])

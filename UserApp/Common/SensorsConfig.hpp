#ifndef __SENSORS_CONFIG_HPP
#define __SENSORS_CONFIG_HPP

namespace auv {
namespace config {

enum class ZDataSource {
    USE_INS_INTEGRATED_Z,
    USE_MS5837_Z
};

struct SensorsConfig {
    ZDataSource z_data_source;
};

static const SensorsConfig DEFAULT_SENSORS_CONFIG = {
    .z_data_source = ZDataSource::USE_MS5837_Z
};

} // namespace config
} // namespace auv

#endif

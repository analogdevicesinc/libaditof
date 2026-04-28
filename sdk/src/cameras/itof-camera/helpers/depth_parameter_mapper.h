/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef DEPTH_PARAMETER_MAPPER_H
#define DEPTH_PARAMETER_MAPPER_H

#include <aditof/status_definitions.h>

#include <functional>
#include <map>
#include <string>
#include <vector>

// Forward declaration - CameraItof is in global namespace
class CameraItof;

namespace aditof {

/**
 * @brief Maps INI parameter names to ADSD3500 setter methods.
 *
 * Provides a table-driven approach to applying depth computation parameters
 * from INI files to the ADSD3500 hardware. Eliminates repetitive if-else
 * chains by using a parameter descriptor table.
 */
class DepthParameterMapper {
  public:
    DepthParameterMapper(CameraItof *camera);
    ~DepthParameterMapper() = default;

    /**
     * @brief Applies INI parameters to ADSD3500 hardware.
     *
     * @param iniKeyValPairs Parameter name-value map
     * @param updateDepthMap If true, updates cached depth params
     * @param updateCallback Callback for parameter map updates
     * @return Status::OK (always, with per-parameter warnings)
     */
    Status applyParameters(
        const std::map<std::string, std::string> &iniKeyValPairs,
        bool updateDepthMap,
        std::function<void(bool, const char *, std::string)> updateCallback);

  private:
    CameraItof *m_camera;

    // Parameter descriptor for table-driven application
    struct ParamDescriptor {
        const char *key;
        std::function<Status(const std::string &)> setter;
        bool required;
    };

    std::vector<ParamDescriptor> m_paramTable;

    void buildParameterTable();
};

} // namespace aditof

#endif // DEPTH_PARAMETER_MAPPER_H

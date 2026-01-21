// Copyright Aeva 2024

#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include <yaml.h>

#include "Carla/Sensor/LidarDescription.h"
#include "carla/geom/Math.h"

// Class representing a set of vectors arranged in beams (lasers) and lines to be raycast by the simulator.
class RayCastLidarPattern {
public:
  RayCastLidarPattern() = default;
  RayCastLidarPattern(const FLidarDescription &LidarDescription) {
    // Load scan pattern YAML
    UE_LOG(LogCarla, Log, TEXT("Using scan pattern YAML: %s"), *LidarDescription.PatternFilePath);
    const auto YamlPath = std::string(TCHAR_TO_UTF8(*LidarDescription.PatternFilePath));

    if (YamlPath == "default") {
      UE_LOG(LogCarla, Log, TEXT("Missing scan pattern YAML, will use default uniform pattern."));
      return;
    }

    const auto YamlNode = YAML::LoadFile(YamlPath);
    verifyf([&YamlNode](){
      return !YamlNode.IsNull() && YamlNode["patterns"];
    }(), TEXT("Could not load pattern file"));

    const auto PatternNode =
        YamlNode["patterns"]
            [std::string(TCHAR_TO_UTF8(*LidarDescription.PatternName))];

    ElevationsDeg = PatternNode["elevation_steps_deg"].as<std::vector<float>>();

    UE_LOG(LogCarla, Log, TEXT("Scan pattern set to %s"), *LidarDescription.PatternName);
  }

  std::vector<float> ElevationsDeg;

};

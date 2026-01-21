// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/RadarData.h>
#include <compiler/enable-ue4-macros.h>

#include "Radar.generated.h"

/// A ray-cast based Radar sensor.
UCLASS()
class CARLA_API ARadar : public ASensor
{
  GENERATED_BODY()

  using FRadarData = carla::sensor::data::RadarData;

public:

  static FActorDefinition GetSensorDefinition();

  ARadar(const FObjectInitializer &ObjectInitializer);

  void Set(const FActorDescription &Description) override;

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetHorizontalFOV(float NewHorizontalFOV);

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetVerticalFOV(float NewVerticalFOV);

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetRange(float NewRange);

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetPointsPerSecond(int NewPointsPerSecond);

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetRadarType(FString SetRadarType);

protected:

  void BeginPlay() override;

  // virtual void PrePhysTick(float DeltaTime) override;
  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Detection")
  float Range;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Detection")
  float HorizontalFOV;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Detection")
  float VerticalFOV;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Detection")
  int PointsPerSecond;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Detection")
  FString RadarType;

private:

  void CalculateCurrentVelocity(const float DeltaTime);

  void SendLineTraces(float DeltaTime);

  float CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation);

  FRadarData RadarData;

  FCollisionQueryParams TraceParams;

  FVector CurrentVelocity;

  /// Used to compute the velocity of the radar
  FVector PrevLocation;

  static constexpr double kDegToRad = 0.01745329;

  struct RayData {
    float Radius;
    float Angle;
    bool Hitted;
    float RelativeVelocity;
    FVector2D AzimuthAndElevation;
    float Distance;
    uint32_t ActorId;
  };

  struct AltosBeamParams {
    float HFOV = 110.0f * kDegToRad;
    float VFOV = 23.0f * kDegToRad;

    float AzRes = 1.38f * kDegToRad;
    float ElRes = 1.43f * kDegToRad;

    float AngleNoiseStd = 0.15f * kDegToRad;
  };

  static AltosBeamParams AltosParams;

  std::vector<RayData> Rays;
};

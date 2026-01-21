// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>

#include "Carla.h"
#include "Carla/Sensor/Radar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include <compiler/enable-ue4-macros.h>

FActorDefinition ARadar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeRadarDefinition();
}

ARadar::ARadar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));

  TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;

}

void ARadar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetRadar(ActorDescription, this);
}

void ARadar::SetHorizontalFOV(float NewHorizontalFOV)
{
  HorizontalFOV = NewHorizontalFOV;
}

void  ARadar::SetVerticalFOV(float NewVerticalFOV)
{
  VerticalFOV = NewVerticalFOV;
}

void ARadar::SetRange(float NewRange)
{
  Range = NewRange;
}

void ARadar::SetPointsPerSecond(int NewPointsPerSecond)
{
  PointsPerSecond = NewPointsPerSecond;
  RadarData.SetResolution(PointsPerSecond);
}

void ARadar::SetRadarType(FString NewRadarType)
{
  RadarType = NewRadarType;
}

void ARadar::BeginPlay()
{
  Super::BeginPlay();

  PrevLocation = GetActorLocation();
}

void ARadar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARadar::PostPhysTick);
  CalculateCurrentVelocity(DeltaTime);

  RadarData.Reset();
  SendLineTraces(DeltaTime);

  auto DataStream = GetDataStream(*this);

  // ROS2
  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromRadar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, RadarData, this);
    }
    else
    {
      ROS2->ProcessDataFromRadar(DataStream.GetSensorType(), StreamId, DataStream.GetSensorTransform(), RadarData, this);
    }
  }
  #endif

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    DataStream.SerializeAndSend(*this, RadarData, DataStream.PopBufferFromPool());
  }
}

void ARadar::CalculateCurrentVelocity(const float DeltaTime)
{
  const FVector RadarLocation = GetActorLocation();
  CurrentVelocity = (RadarLocation - PrevLocation) / DeltaTime;
  PrevLocation = RadarLocation;
}

void ARadar::SendLineTraces(float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARadar::SendLineTraces);
  constexpr float TO_METERS = 1e-2;
  const FTransform& ActorTransform = GetActorTransform();
  const FRotator& TransformRotator = ActorTransform.Rotator();
  const FVector& RadarLocation = GetActorLocation();
  const FVector& ForwardVector = GetActorForwardVector();
  const FVector TransformXAxis = ActorTransform.GetUnitAxis(EAxis::X);
  const FVector TransformYAxis = ActorTransform.GetUnitAxis(EAxis::Y);
  const FVector TransformZAxis = ActorTransform.GetUnitAxis(EAxis::Z);

  // Maximum radar radius in horizontal and vertical direction
  const float MaxRx = FMath::Tan(FMath::DegreesToRadians(HorizontalFOV * 0.5f)) * Range;
  const float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f)) * Range;

  // Generate the parameters of the rays in a deterministic way
  Rays.clear();
  int NumPoints;
  if (RadarType == "Default") {
    NumPoints = (int)(PointsPerSecond * DeltaTime);
    Rays.resize(NumPoints);
    for (int i = 0; i < Rays.size(); i++) {
      Rays[i].Radius = RandomEngine->GetUniformFloat();
      Rays[i].Angle = RandomEngine->GetUniformFloatInRange(0.0f, carla::geom::Math::Pi2<float>());
      Rays[i].Hitted = false;
    }
  } else {
    // Use angular-resolution beams
    float AzRes = FMath::DegreesToRadians(1.38f);
    float ElRes = FMath::DegreesToRadians(1.43f);

    // const int NumAz = FMath::RoundToInt(HorizontalFOV / FMath::RadiansToDegrees(AzRes));
    // const int NumEl = FMath::RoundToInt(VerticalFOV   / FMath::RadiansToDegrees(ElRes));
    float HFOV_rad = FMath::DegreesToRadians(HorizontalFOV);
    float VFOV_rad = FMath::DegreesToRadians(VerticalFOV);
    int NumAz = FMath::RoundToInt(HFOV_rad / AzRes);
    int NumEl = FMath::RoundToInt(VFOV_rad / ElRes);

    NumPoints = NumAz * NumEl;
    Rays.resize(NumPoints);

    float AzStart = -HFOV_rad * 0.5f;
    float ElStart = -VFOV_rad * 0.5f;

    int idx = 0;
    for (int ia = 0; ia < NumAz; ia++) {
      for (int ie = 0; ie < NumEl; ie++) {

        float az = AzStart + ia * AzRes;
        float el = ElStart + ie * ElRes;

        // Add Altos angular accuracy noise (0.15°)
        az += RandomEngine->GetNormalDistribution(0.0f, 0.05) * FMath::DegreesToRadians(0.15f);
        el += RandomEngine->GetNormalDistribution(0.0f, 0.05) * FMath::DegreesToRadians(0.15f);

        Rays[idx].Radius = az;
        Rays[idx].Angle = el;
        Rays[idx].Hitted = false;

        idx++;
      }
    }
  }

  FCriticalSection Mutex;
  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(NumPoints, [&](int32 idx) {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);
      FHitResult OutHit(ForceInit);
      const float Radius = Rays[idx].Radius;
      const float Angle  = Rays[idx].Angle;

      float Sin, Cos;
      FMath::SinCos(&Sin, &Cos, Angle);

      FVector EndLocation;
      if (RadarType == "Default") {
        EndLocation = RadarLocation + TransformRotator.RotateVector({
          Range,
          MaxRx * Radius * Cos,
          MaxRy * Radius * Sin
        });
      } else {
        // Convert spherical angles to Cartesian direction
        FVector Direction = TransformRotator.RotateVector(
          FVector(
              FMath::Cos(Angle) * FMath::Cos(Radius),
              FMath::Cos(Angle) * FMath::Sin(Radius),
              FMath::Sin(Angle)
          )
        );
        EndLocation = RadarLocation + Direction * Range;
      }

      const bool Hitted = GetWorld()->ParallelLineTraceSingleByChannel(
        OutHit,
        RadarLocation,
        EndLocation,
        ECC_GameTraceChannel2,
        TraceParams,
        FCollisionResponseParams::DefaultResponseParam
      );

      const TWeakObjectPtr<AActor> HittedActor = OutHit.Actor;
      if (Hitted && HittedActor.Get()) {
        Rays[idx].Hitted = true;
        // UE_LOG(LogCarla, Log, TEXT("%f, %f, %f"), EndLocation.X, EndLocation.Y, EndLocation.Z);

        if (RadarType == "Default") {
          Rays[idx].RelativeVelocity = CalculateRelativeVelocity(OutHit, RadarLocation);
        } else {
          float v = Rays[idx].RelativeVelocity;
          // Clamp measurable velocity
          // if (v < -110 || v > 55)
          //     continue;

          // Quantize to Doppler bins
          float VelocityResolution = 0.2;
          float VelocityAccuracyStd = 0.02;
          v = FMath::RoundToFloat(v / VelocityResolution) * VelocityResolution;
          // Add Doppler noise
          v += RandomEngine->GetNormalDistribution(0.0f, 0.05) * VelocityAccuracyStd;

          Rays[idx].RelativeVelocity = v;
        }

        Rays[idx].AzimuthAndElevation = FMath::GetAzimuthAndElevation (
          (EndLocation - RadarLocation).GetSafeNormal() * Range,
          TransformXAxis,
          TransformYAxis,
          TransformZAxis
        );

        Rays[idx].Distance = OutHit.Distance * TO_METERS;
        // if (RadarType == "Altos") {
        //   float d = Rays[idx].Distance;

        //   // // Altos max range
        //   // if (d > 80) continue;
        //   // Quantize to bins
        //   d = FMath::RoundToFloat(d / 0.35) * 0.35;
        //   // Add Gaussian error
        //   d += RandomEngine->GetNormalDistribution(0.0f, 0.05) * 0.05;

        //   Rays[idx].Distance = d;
        // }
        const FActorRegistry &Registry = GetEpisode().GetActorRegistry();
        const FCarlaActor* view = Registry.FindCarlaActor(HittedActor.Get());
        if(view)
          Rays[idx].ActorId = view->GetActorId();
        }
    });

    // Limit points per frame
    if (RadarType == "Altos") {
      int MaxAltosPoints = 3000;
      if (Rays.size() > MaxAltosPoints) {
          auto rng = std::default_random_engine {};
          std::shuffle(std::begin(Rays), std::end(Rays), rng);
          Rays.resize(MaxAltosPoints);
      }
    }
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  // // Write the detections in the output structure
  // for (auto& ray : Rays) {
  //   if (ray.Hitted) {
  //     RadarData.WriteDetection({
  //       ray.RelativeVelocity,
  //       ray.AzimuthAndElevation.X,
  //       ray.AzimuthAndElevation.Y,
  //       ray.Distance,
  //       ray.ActorId
  //     });
  //   }
  // }

  // Write noises
  // Tunable parameters
  const float BaseRangeStd = 0.1f;                 // m, near range accuracy
  const float RangeStdSlope = 0.005f;               // m noise per meter
  const float BaseAngleStdDeg = 0.20f;              // deg at near range
  const float AngleStdSlopeDeg = 0.30f;             // extra deg at max range
  const int   ClusterPointsPerHit = 3;              // extra cluster points
  const float DropoutMinPD = 0.15f;                 // min detection prob
  const float DropoutMaxPD = 0.85f;                 // max detection prob
  const float ClutterProb = 0.02f;                  // chance of clutter per hit
  const float GhostProb   = 0.08f;                  // chance of ghost point

  for (auto& ray : Rays) {
    if (!ray.Hitted)
      continue;

    float range = ray.Distance;
    float az    = ray.AzimuthAndElevation.X;
    float el    = ray.AzimuthAndElevation.Y;

    // --- Detection probability (dropouts) ---
    float pd = 1.0f - (range / Range);
    pd = FMath::Clamp(pd, DropoutMinPD, DropoutMaxPD);

    if (RandomEngine->GetUniformFloat() > pd)
    {
      // Missed detection: skip this ray entirely
      continue;
    }

    // --- Base range and angle noise (distance dependent) ---
    const float range_std = BaseRangeStd + RangeStdSlope * range;
    const float angle_std_deg =
      BaseAngleStdDeg + AngleStdSlopeDeg * (range / Range);
    const float angle_std = FMath::DegreesToRadians(angle_std_deg);

    float noisy_range =
      range + RandomEngine->GetNormalDistribution(0.0f, range_std);
    float noisy_az =
      az + RandomEngine->GetNormalDistribution(0.0f, angle_std);
    float noisy_el =
      el + RandomEngine->GetNormalDistribution(0.0f, angle_std);

    // --- Main detection (noisy) ---
    RadarData.WriteDetection({
      ray.RelativeVelocity,
      noisy_az,
      noisy_el,
      noisy_range,
      ray.ActorId
    });

    // --- Cluster points around main detection ---
    for (int k = 0; k < ClusterPointsPerHit; ++k)
    {
      float cr = noisy_range +
        RandomEngine->GetNormalDistribution(0.0f, range_std * 2.0f);
      float caz = noisy_az +
        RandomEngine->GetNormalDistribution(0.0f, angle_std * 2.5f);
      float cel = noisy_el +
        RandomEngine->GetNormalDistribution(0.0f, angle_std * 2.5f);

      RadarData.WriteDetection({
        ray.RelativeVelocity,
        caz,
        cel,
        cr,
        ray.ActorId
      });
    }

    // --- Clutter: random scatter near strong targets ---
    if (RandomEngine->GetUniformFloat() < ClutterProb)
    {
      float cr = noisy_range +
        RandomEngine->GetNormalDistribution(0.0f, range_std * 4.0f);
      float caz = noisy_az +
        RandomEngine->GetNormalDistribution(0.0f, angle_std * 4.0f);
      float cel = noisy_el +
        RandomEngine->GetNormalDistribution(0.0f, angle_std * 4.0f);

      RadarData.WriteDetection({
        0.0f,      // clutter → no meaningful radial velocity
        caz,
        cel,
        cr,
        ray.ActorId
      });
    }

    // --- Simple ghost reflection (multipath) ---
    if (RandomEngine->GetUniformFloat() < GhostProb)
    {
      // Mirror elevation around sensor horizon (approximate ground reflection)
      float ghost_el = -noisy_el +
        RandomEngine->GetNormalDistribution(0.0f, angle_std * 1.5f);
      float ghost_range = noisy_range +
        RandomEngine->GetNormalDistribution(0.0f, range_std * 1.5f);

      RadarData.WriteDetection({
        ray.RelativeVelocity,
        noisy_az,
        ghost_el,
        ghost_range,
        ray.ActorId
      });
    }
  }
}

float ARadar::CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation)
{
  constexpr float TO_METERS = 1e-2;

  const TWeakObjectPtr<AActor> HittedActor = OutHit.Actor;
  const FVector TargetVelocity = HittedActor->GetVelocity();
  const FVector TargetLocation = OutHit.ImpactPoint;
  const FVector Direction = (TargetLocation - RadarLocation).GetSafeNormal();
  const FVector DeltaVelocity = (TargetVelocity - CurrentVelocity);
  const float V = TO_METERS * FVector::DotProduct(DeltaVelocity, Direction);

  return V;
}

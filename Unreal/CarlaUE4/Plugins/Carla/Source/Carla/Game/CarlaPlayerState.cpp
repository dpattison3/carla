// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "CarlaPlayerState.h"

void ACarlaPlayerState::Reset()
{
  Super::Reset();
  // Reset incremental values.
  GameTimeStamp = 0.0f;
  CollisionIntensityCars = 0.0f;
  CollisionIntensityPedestrians = 0.0f;
  CollisionIntensityOther = 0.0f;
}

void ACarlaPlayerState::CopyProperties(APlayerState *PlayerState)
{
  Super::CopyProperties(PlayerState);
  if ((PlayerState != nullptr) && (this != PlayerState))
  {
    ACarlaPlayerState *Other = Cast<ACarlaPlayerState>(PlayerState);
    if (Other != nullptr)
    {
      FramesPerSecond = Other->FramesPerSecond;
      PlatformTimeStamp = Other->PlatformTimeStamp;
      GameTimeStamp = Other->GameTimeStamp;
      Transform = Other->Transform;
      ForwardSpeed = Other->ForwardSpeed;
      Acceleration = Other->Acceleration;
      Velocity = Other->Velocity;
      AngularRate = Other->AngularRate;
      Throttle = Other->Throttle;
      Steer = Other->Steer;
      Brake = Other->Brake;
      bHandBrake = Other->bHandBrake;
      CurrentGear = Other->CurrentGear;
      SpeedLimit = Other->SpeedLimit;
      TrafficLightState = Other->TrafficLightState;
      CollisionIntensityCars = Other->CollisionIntensityCars;
      CollisionIntensityPedestrians = Other->CollisionIntensityPedestrians;
      CollisionIntensityOther = Other->CollisionIntensityOther;
      OtherLaneIntersectionFactor = Other->OtherLaneIntersectionFactor;
      OffRoadIntersectionFactor = Other->OffRoadIntersectionFactor;
      UE_LOG(LogCarla, Log, TEXT("Copied properties of ACarlaPlayerState"));
    }
  }
}

void ACarlaPlayerState::RegisterCollision(
    AActor * /*Actor*/,
    AActor * /*OtherActor*/,
    const FVector &NormalImpulse,
    const FHitResult &Hit)
{
  switch (ATagger::GetTagOfTaggedComponent(*Hit.Component)) {
    case ECityObjectLabel::Vehicles:
      CollisionIntensityCars += NormalImpulse.Size();
      break;
    case ECityObjectLabel::Pedestrians:
      CollisionIntensityPedestrians += NormalImpulse.Size();
      break;
    default:
      CollisionIntensityOther += NormalImpulse.Size();
      break;
  }
}

static int32 RoundToMilliseconds(float Seconds)
{
  return FMath::RoundHalfToZero(1000.0 * Seconds);
}

void ACarlaPlayerState::UpdateTimeStamp(float DeltaSeconds)
{
  FramesPerSecond = 1.0f / DeltaSeconds;
  PlatformTimeStamp = RoundToMilliseconds(FPlatformTime::Seconds());
  GameTimeStamp += RoundToMilliseconds(DeltaSeconds);
}

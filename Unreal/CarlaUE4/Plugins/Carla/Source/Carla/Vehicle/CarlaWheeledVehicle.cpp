// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "CarlaWheeledVehicle.h"

#include "Agent/VehicleAgentComponent.h"
#include "Vehicle/VehicleControl.h"

#include "Components/BoxComponent.h"
#include "Engine/CollisionProfile.h"

#include <iostream>

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

ACarlaWheeledVehicle::ACarlaWheeledVehicle(const FObjectInitializer& ObjectInitializer) :
  Super(ObjectInitializer)
{
  VehicleBounds = CreateDefaultSubobject<UBoxComponent>(TEXT("VehicleBounds"));
  VehicleBounds->SetupAttachment(RootComponent);
  VehicleBounds->SetHiddenInGame(true);
  VehicleBounds->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

  VehicleAgentComponent = CreateDefaultSubobject<UVehicleAgentComponent>(TEXT("VehicleAgentComponent"));
  VehicleAgentComponent->SetupAttachment(RootComponent);

  GetVehicleMovementComponent()->bReverseAsBrake = false;
}

ACarlaWheeledVehicle::~ACarlaWheeledVehicle() {}

// =============================================================================
// -- Get functions ------------------------------------------------------------
// =============================================================================

FTransform ACarlaWheeledVehicle::GetVehicleTransform() const
{
  FTransform Transform = VehicleBounds->GetComponentTransform();
  Transform.SetScale3D(GetActorTransform().GetScale3D());
  return Transform;
}

float ACarlaWheeledVehicle::GetVehicleForwardSpeed() const
{
  // std::cout << TCHAR_TO_UTF8(*(GetVelocity().ToString())) << std::endl;
  // std::cout << TCHAR_TO_UTF8(*(GetVelocity().ToString())) << std::endl;
  // UE_LOG(LogTemp, Warning, TEXT("Velocity is %s"),*GetVelocity().ToString());
  // UE_LOG(LogTemp, Warning, TEXT("Acceleration is %s"),*GetVehicleMovementComponent()->Velocity.ToString());
  // UE_LOG(LogTemp, Warning, TEXT("AngularVelocity is %s"),*GetRotationRate()->ToString());
  return GetVehicleMovementComponent()->GetForwardSpeed();
}

FVector ACarlaWheeledVehicle::GetVehicleVelocity() const
{
  return GetVelocity();
}

FVector ACarlaWheeledVehicle::GetVehicleOrientation() const
{
  return GetVehicleTransform().GetRotation().GetForwardVector();
}

int32 ACarlaWheeledVehicle::GetVehicleCurrentGear() const
{
  return GetVehicleMovementComponent()->GetCurrentGear();
}

FVector ACarlaWheeledVehicle::GetVehicleBoundsExtent() const
{
  return VehicleBounds->GetScaledBoxExtent();
}

float ACarlaWheeledVehicle::GetMaximumSteerAngle() const
{
  const auto &Wheels = GetVehicleMovementComponent()->Wheels;
  check(Wheels.Num() > 0);
  const auto *FrontWheel = Wheels[0];
  check(FrontWheel != nullptr);
  return FrontWheel->SteerAngle;
}

// =============================================================================
// -- Set functions ------------------------------------------------------------
// =============================================================================

void ACarlaWheeledVehicle::ApplyVehicleControl(const FVehicleControl &VehicleControl)
{
  SetThrottleInput(VehicleControl.Throttle);
  SetSteeringInput(VehicleControl.Steer);
  SetBrakeInput(VehicleControl.Brake);
  SetHandbrakeInput(VehicleControl.bHandBrake);
  SetReverse(VehicleControl.bReverse);
}

void ACarlaWheeledVehicle::SetThrottleInput(const float Value)
{
  GetVehicleMovementComponent()->SetThrottleInput(Value);
}

void ACarlaWheeledVehicle::SetSteeringInput(const float Value)
{
  GetVehicleMovementComponent()->SetSteeringInput(Value);
}

void ACarlaWheeledVehicle::SetBrakeInput(const float Value)
{
  GetVehicleMovementComponent()->SetBrakeInput(Value);
}

void ACarlaWheeledVehicle::SetReverse(const bool Value)
{
  if (Value != bIsInReverse) {
    bIsInReverse = Value;
    auto MovementComponent = GetVehicleMovementComponent();
    MovementComponent->SetUseAutoGears(!bIsInReverse);
    MovementComponent->SetTargetGear(bIsInReverse ? -1 : 1, true);
  }
}

void ACarlaWheeledVehicle::SetHandbrakeInput(const bool Value)
{
  GetVehicleMovementComponent()->SetHandbrakeInput(Value);
}

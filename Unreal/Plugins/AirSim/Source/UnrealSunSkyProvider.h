// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DirectionalLight.h"
#include "UObject/NoExportTypes.h"
#include "SunSkyProvider.h"
#include "common/CommonStructs.hpp"

#include "UnrealSunSkyProvider.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API UUnrealSunSkyProvider : public UObject, public ISunSkyProvider
{
	GENERATED_BODY()
	msr::airlib::HomeGeoPoint origin_geopoint_;

	UPROPERTY()
	UClass* sky_sphere_class_;

	UPROPERTY()
	AActor* sky_sphere_;

	UPROPERTY()
	ADirectionalLight* sun_;
	FRotator default_sun_rotation_;
	
public:
	UUnrealSunSkyProvider();

	void initialize(msr::airlib::HomeGeoPoint origin_geopoint);

	virtual void setTimeOfDay(uint64_t tod) override;
};

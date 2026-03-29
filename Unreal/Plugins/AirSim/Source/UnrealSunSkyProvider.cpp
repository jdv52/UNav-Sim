// Fill out your copyright notice in the Description page of Project Settings.


#include "UnrealSunSkyProvider.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/OutputDeviceNull.h"

#include "AirBlueprintLib.h"
#include "common/EarthCelestial.hpp"

UUnrealSunSkyProvider::UUnrealSunSkyProvider()
{
    static ConstructorHelpers::FClassFinder<AActor> sky_sphere_class(TEXT("Blueprint'/Engine/EngineSky/BP_Sky_Sphere'"));
    sky_sphere_class_ = sky_sphere_class.Succeeded() ? sky_sphere_class.Class : nullptr;
}

void UUnrealSunSkyProvider::initialize(msr::airlib::HomeGeoPoint origin_geopoint)
{
	origin_geopoint_ = origin_geopoint;
    TArray<AActor*> sky_spheres;
    UGameplayStatics::GetAllActorsOfClass(this->GetWorld(), sky_sphere_class_, sky_spheres);

    if (sky_spheres.Num() > 1)
        UAirBlueprintLib::LogMessage(TEXT("More than BP_Sky_Sphere were found. "),
            TEXT("TimeOfDay settings would be applied to first one."),
            LogDebugLevel::Failure);

    if (sky_spheres.Num() >= 1) {
        sky_sphere_ = sky_spheres[0];
        static const FName sun_prop_name(TEXT("Directional light actor"));
        auto* p = sky_sphere_class_->FindPropertyByName(sun_prop_name);

#if ENGINE_MINOR_VERSION > 24
        FObjectProperty* sun_prop = CastFieldChecked<FObjectProperty>(p);
#else
        FObjectProperty* sun_prop = Cast<FObjectProperty>(p);
#endif

        UObject* sun_obj = sun_prop->GetObjectPropertyValue_InContainer(sky_sphere_);
        sun_ = Cast<ADirectionalLight>(sun_obj);
        if (sun_) {
            default_sun_rotation_ = sun_->GetActorRotation();

            sun_->GetRootComponent()->Mobility = EComponentMobility::Movable;
        }
    }
    else {
		UAirBlueprintLib::LogMessage(TEXT("BP_Sky_Sphere was not found. "),
			TEXT("TimeOfDay settings would not be applied."),
			LogDebugLevel::Failure);
    }
}

void UUnrealSunSkyProvider::setTimeOfDay(uint64_t tod)
{
    if (sun_ && sky_sphere_) {
        auto coord = msr::airlib::EarthCelestial::getSunCoordinates(tod, origin_geopoint_.home_geo_point.latitude, origin_geopoint_.home_geo_point.longitude);

        FRotator rotation(-coord.altitude, coord.azimuth, 0);

        UAirBlueprintLib::RunCommandOnGameThread([this, rotation]() {
            sun_->SetActorRotation(rotation);

            FOutputDeviceNull ar;
            sky_sphere_->CallFunctionByNameWithArguments(TEXT("UpdateSunDirection"), ar, NULL, true);
            },
            true /*wait*/);
    }
}
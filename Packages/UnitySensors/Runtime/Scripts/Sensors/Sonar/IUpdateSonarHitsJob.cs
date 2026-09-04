// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Per-cycle: turns raw RaycastHit results into sonar-frame hit data (local point +
    /// Lambertian return intensity). Material reflectivity is resolved Burst-side from
    /// <see cref="AcousticSurfaceRegistry"/> keyed on <c>RaycastHit.colliderEntityId</c>
    /// (a blittable substitute for the managed <c>hit.collider</c>), so the whole pipeline
    /// runs as one job chain with no main-thread Collider access in the middle.
    /// </summary>
    [BurstCompile]
    internal struct IUpdateSonarHitsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<RaycastHit> Results;
        [ReadOnly] public NativeArray<float3> LocalDirections;
        [ReadOnly] public NativeArray<float> BeamProfile;          // length == NumRaysPerBeam
        [ReadOnly] public NativeHashMap<int, float> ReflectivityMap; // colliderEntityId -> reflectivity, sealed at scene load

        public float DefaultReflectivity;                          // used for colliders with no AcousticSurface
        public int NumRaysPerBeam;
        public float MaxRange;
        public quaternion WorldToLocalRotation;

        [WriteOnly] public NativeArray<float3> LocalPoints;
        [WriteOnly] public NativeArray<float> ReturnIntensities;

        public void Execute(int i)
        {
            RaycastHit hit = Results[i];
            // colliderEntityId is the Burst-safe substitute for `hit.collider` (0 == miss);
            // it implicitly converts to the int key used by ReflectivityMap.
            bool didHit = hit.colliderEntityId != 0;

            float3 localDir = LocalDirections[i];

            LocalPoints[i] = didHit ? localDir * hit.distance : float3.zero;

            if (!didHit)
            {
                ReturnIntensities[i] = 0f;
                return;
            }

            float beamIntensity = BeamProfile[i % NumRaysPerBeam];

            // 1) Distance traveled by the beam.
            float hitDistIntensity = (MaxRange - hit.distance) / MaxRange;

            // 2) Angle of hit. abs(dot(normalize(-rayDir), normal)) is the same quantity as
            // abs(cos(Vector3.Angle(...))) but skips the acos/cos round-trip; angle is
            // rotation-invariant so local-frame vectors give the same result as the
            // equivalent world-space computation.
            float3 localNormal = math.normalizesafe(math.mul(WorldToLocalRotation, (float3)hit.normal));
            float cosAngle = math.dot(math.normalizesafe(-localDir), localNormal);
            float hitAngleIntensity = math.abs(cosAngle);

            // 3) Material reflectivity, resolved Burst-side from the collider instance ID.
            // Colliders without an AcousticSurface (or added after the map was sealed) use
            // the sensor default.
            float reflectivity = ReflectivityMap.TryGetValue(hit.colliderEntityId, out float r)
                ? r
                : DefaultReflectivity;

            // Lambert's cosine law, K = 1.
            float intensity = beamIntensity * hitDistIntensity * hitAngleIntensity * reflectivity;
            ReturnIntensities[i] = math.clamp(intensity, 0f, 1f);
        }
    }
}

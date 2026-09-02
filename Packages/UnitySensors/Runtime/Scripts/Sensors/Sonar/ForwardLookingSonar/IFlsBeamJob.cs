// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

// Ported from the SMARC project (smarc2):
// SMARCAssets/Runtime/Scripts/VehicleComponents/Sensors/Sonar.cs (the FLS branch of
// SetupSonarRaycastJob.Execute). Run once at init by ForwardLookingSonarSensor to bake
// the ray fan; the fed axes are the sensor-LOCAL basis, so the output directions are in
// the sensor's own frame. Per-cycle world rays are built by BuildRaycastCommandsJob.

using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>FLS: vertical fans swept side-to-side across the FOV, tilted downwards.</summary>
    [BurstCompile]
    internal struct IFlsBeamJob : IJobParallelFor
    {
        [WriteOnly] public NativeArray<float3> Directions;
        public int NumRaysPerBeam;
        public Vector3 LocalUp;
        public Vector3 LocalForward;
        public Vector3 LocalRight;
        public float DegreesPerRayInBeam;
        public float TiltAngleDeg;
        public float FLSFOVDeg;
        public float DegreesPerBeamInFLS;

        public void Execute(int i)
        {
            var (beamNum, rayNum) = SonarSensor.BeamNumRayNumFromRayIndex(i, NumRaysPerBeam);

            // FLS is MBES, but the first ray is not in the center, its at the edge and is tilted.
            // there are also >1 beams.

            // we want 0 degrees at the edge and BeamBreadt degrees at the other edge of the beam.
            var rayAngle = rayNum * DegreesPerRayInBeam;
            // FLS beams are defined as vertical fans, sweeping side-to-side
            // so we start a ray forward first.
            var direction = LocalForward;
            // then we rotate _that_ to the ray angle within the beam, around the side-axis
            // plus the tilt angle which is measured from the horizontal plane down
            direction = Quaternion.AngleAxis(rayAngle + TiltAngleDeg, LocalRight) * direction;
            // then we rotate it to the beam angle, around the UP axis
            var beamAngle = (beamNum * DegreesPerBeamInFLS) - FLSFOVDeg / 2;
            direction = Quaternion.AngleAxis(beamAngle, LocalUp) * direction;

            Directions[i] = direction;
        }
    }
}

// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

// Ported from the SMARC project (smarc2):
// SMARCAssets/Runtime/Scripts/VehicleComponents/Sensors/Sonar.cs (the FLS branch).

using UnityEngine;

using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Forward-looking sonar: vertical fans swept side-to-side across <see cref="FLSFOVDeg"/>,
    /// each fan tilted down by <see cref="TiltAngleDeg"/>.
    /// </summary>
    public class ForwardLookingSonarSensor : SonarSensor
    {
        [Header("Forward-Looking Sonar")]
        [Tooltip("Tilt of the beams downwards, measured from the forward-right plane.")]
        public float TiltAngleDeg = 15;
        [Tooltip("Field of view swept by the beams.")]
        public float FLSFOVDeg = 30;

        // NumBeams == 1 has no spread to divide across; guarded to avoid a division by
        // zero (FLSFOVDeg / 0 = Infinity, which turns into NaN as soon as it's multiplied
        // by a beam index of 0 in IFlsBeamJob, corrupting every ray direction).
        public float DegreesPerBeamInFLS => NumBeams > 1 ? FLSFOVDeg / (NumBeams - 1) : 0f;

        // UnitySensor.OnValidate is private, so this hides it rather than overrides it.
        // End with `Frequency = Frequency;` so the base re-derives its cached _frequency_inv.
        private void OnValidate()
        {
            if (NumRaysPerBeam <= 0) NumRaysPerBeam = 1;
            if (NumBeams <= 0) NumBeams = 1;
            Frequency = Frequency;
        }

        protected override void FillLocalDirections(NativeArray<float3> directions)
        {
            var job = new IFlsBeamJob
            {
                Directions = directions,
                NumRaysPerBeam = NumRaysPerBeam,
                NumBeams = NumBeams,
                LocalUp = Vector3.up,
                LocalForward = Vector3.forward,
                LocalRight = Vector3.right,
                DegreesPerRayInBeam = DegreesPerRayInBeam,
                TiltAngleDeg = TiltAngleDeg,
                FLSFOVDeg = FLSFOVDeg,
                DegreesPerBeamInFLS = DegreesPerBeamInFLS
            };
            job.Schedule(directions.Length, 10).Complete();
        }
    }
}

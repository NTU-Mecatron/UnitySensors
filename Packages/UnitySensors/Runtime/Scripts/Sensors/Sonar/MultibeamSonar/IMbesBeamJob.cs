// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

// Ported from the SMARC project (smarc2):
// SMARCAssets/Runtime/Scripts/VehicleComponents/Sensors/Sonar.cs (the MBES branch of
// SetupSonarRaycastJob.Execute). Run once at init by MultibeamSonarSensor to bake the
// ray fan; the fed axes are the sensor-LOCAL basis, so the output directions are in the
// sensor's own frame. Per-cycle world rays are built by BuildRaycastCommandsJob.

using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>MBES: one beam looking directly down, fanned about the forward axis.</summary>
    [BurstCompile]
    internal struct IMbesBeamJob : IJobParallelFor
    {
        [WriteOnly] public NativeArray<float3> Directions;
        public int NumRaysPerBeam;
        public Vector3 LocalUp;
        public Vector3 LocalForward;
        public float DegreesPerRayInBeam;
        public float BeamBreadthDeg;

        public void Execute(int i)
        {
            var (_, rayNum) = SonarSensor.BeamNumRayNumFromRayIndex(i, NumRaysPerBeam);

            // MBES is just one beam looking down directly. Simplest.
            // start a beam looking directly down
            var direction = -LocalUp;
            // we want 0 degrees in the center and then +-Breadth/2 on the sides.
            var rayAngle = (rayNum * DegreesPerRayInBeam) - BeamBreadthDeg / 2;
            // rotate it around the forward axis by its ray number in the beam
            // offset half-way so the middle is directly down.
            direction = Quaternion.AngleAxis(rayAngle, LocalForward) * direction;

            Directions[i] = direction;
        }
    }
}

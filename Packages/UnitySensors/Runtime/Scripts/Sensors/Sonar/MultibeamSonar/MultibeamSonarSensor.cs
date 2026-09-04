// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

// Ported from the SMARC project (smarc2):
// SMARCAssets/Runtime/Scripts/VehicleComponents/Sensors/Sonar.cs (the MBES branch).

using UnityEngine;

using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Multibeam echosounder: a single beam looking straight down, its rays fanned out
    /// about the sensor's forward axis.
    /// </summary>
    public class MultibeamSonarSensor : SonarSensor
    {
        // UnitySensor.OnValidate is private, so this hides it rather than overrides it.
        // End with `Frequency = Frequency;` so the base re-derives its cached _frequency_inv.
        private void OnValidate()
        {
            NumBeams = 1;
            if (NumRaysPerBeam <= 0) NumRaysPerBeam = 1;
            Frequency = Frequency;
        }

        protected override void FillLocalDirections(NativeArray<float3> directions)
        {
            var job = new IMbesBeamJob
            {
                Directions = directions,
                NumRaysPerBeam = NumRaysPerBeam,
                LocalUp = Vector3.up,
                LocalForward = Vector3.forward,
                DegreesPerRayInBeam = DegreesPerRayInBeam,
                BeamBreadthDeg = BeamBreadthDeg
            };
            job.Schedule(directions.Length, 10).Complete();
        }
    }
}

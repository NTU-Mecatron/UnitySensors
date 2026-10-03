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
    /// Per-cycle: rotates the sensor's static local ray fan (<see cref="LocalDirections"/>,
    /// baked once at init) into world space by the sensor's current pose and builds the
    /// raycast batch. Replaces the per-modality beam jobs in the hot path.
    /// </summary>
    [BurstCompile]
    internal struct IUpdateRaycastCommandsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> LocalDirections;
        public Vector3 Origin;
        public quaternion Rotation;
        public float MaxRange;
        [WriteOnly] public NativeArray<RaycastCommand> Commands;

        public void Execute(int i)
        {
            Commands[i] = new RaycastCommand(Origin, math.mul(Rotation, LocalDirections[i]), QueryParameters.Default, MaxRange);
        }
    }
}

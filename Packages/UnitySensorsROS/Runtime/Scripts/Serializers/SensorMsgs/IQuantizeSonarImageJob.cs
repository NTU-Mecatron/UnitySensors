// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [BurstCompile]
    internal struct IQuantizeSonarImageJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> Raw; // normalized (not yet 0-255)
        public float Contrast;
        [WriteOnly] public NativeArray<byte> Dst;

        public void Execute(int index)
        {
            Dst[index] = (byte)math.clamp(Raw[index] * 255f * Contrast, 0f, 255f);
        }
    }
}

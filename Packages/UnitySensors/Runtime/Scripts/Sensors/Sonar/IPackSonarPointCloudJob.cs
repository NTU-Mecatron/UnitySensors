// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using UnitySensors.DataType.Sensor.PointCloud;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Packs the sonar's struct-of-arrays hit output -- sensor-local point and [0, 1]
    /// Lambertian return intensity -- into the interleaved <see cref="PointXYZI"/> array
    /// consumed by the point-cloud serializer / visualizer. Runs as the last link in the
    /// per-cycle job chain, right after <see cref="IUpdateSonarHitsJob"/>. Missed rays
    /// arrive as <c>float3.zero</c> with zero intensity and are packed unchanged.
    /// </summary>
    [BurstCompile]
    internal struct IPackSonarPointCloudJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> LocalPoints;
        [ReadOnly] public NativeArray<float> ReturnIntensities;
        public float IntensityScale;

        [WriteOnly] public NativeArray<PointXYZI> Points;

        public void Execute(int i)
        {
            Points[i] = new PointXYZI
            {
                position = LocalPoints[i],
                intensity = ReturnIntensities[i] * IntensityScale
            };
        }
    }
}

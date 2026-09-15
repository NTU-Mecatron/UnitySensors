// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

using UnitySensors.DataType.Sensor.PointCloud;

namespace UnitySensors.ROS.Serializer.Sensor
{
    /// <summary>
    /// Bins a sonar's per-cycle <see cref="PointXYZI"/> hits into a rectangular bearing
    /// (column = beam index) x range (row = range bin) grayscale grid -- the raw image
    /// format most FLS ROS drivers publish. Parallelized per beam, since each beam only
    /// ever writes its own column: beam b's rays live at [b * NumRaysPerBeam, (b + 1) *
    /// NumRaysPerBeam) in <see cref="Points"/> (see <c>SonarSensor.BeamNumRayNumFromRayIndex</c>).
    /// Multiple rays in a beam landing in the same range bin (its elevation spread) are
    /// reduced to the strongest one, like a real sonar's beam integrating across elevation.
    /// Image is row-major, matching the sonoptix_sonar driver's raw layout: row 0 = near
    /// field/transducer, the last row = max range (row = bin * NumBeams + beam) -- the
    /// caller overwrites that last row's leading pixels with range telemetry afterward,
    /// same as the real driver does.
    /// </summary>
    [BurstCompile]
    internal struct IUpdateSonarImageJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<PointXYZI> Points;
        public int NumBeams;
        public int NumRaysPerBeam;
        public int NumRangeBins;
        public float MaxRange;

        [NativeDisableParallelForRestriction]
        public NativeArray<byte> Image;

        public void Execute(int beam)
        {
            for (int r = 0; r < NumRangeBins; r++) Image[r * NumBeams + beam] = 0;

            int start = beam * NumRaysPerBeam;
            for (int ray = 0; ray < NumRaysPerBeam; ray++)
            {
                PointXYZI p = Points[start + ray];
                float distance = math.length(p.position);
                if (distance <= 0f) continue; // a missed ray sits at the origin

                float t = MaxRange > 0f ? distance / MaxRange : 0f;
                int bin = math.clamp((int)math.floor(t * NumRangeBins), 0, NumRangeBins - 1);

                byte value = (byte)math.clamp(p.intensity * 255f, 0f, 255f);
                int idx = bin * NumBeams + beam;
                if (value > Image[idx]) Image[idx] = value;
            }
        }
    }
}

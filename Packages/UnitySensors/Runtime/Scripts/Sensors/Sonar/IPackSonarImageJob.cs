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

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Bins a sonar's per-cycle <see cref="PointXYZI"/> hits into a rectangular bearing
    /// (column = beam index) x range (row = range bin) grayscale grid. Each bin is the sum
    /// of the intensities that landed in it divided by NumRaysPerBeam -- a fixed normalizer,
    /// not the number of rays that happened to hit that bin -- so a bin only a few rays
    /// grazed reads dimmer than one the whole elevation fan covered, the same way partial
    /// surface coverage attenuates a real sonar return instead of reading as fully bright.
    /// </summary>
    [BurstCompile]
    internal struct IPackSonarImageJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<PointXYZI> LocalPoints;
        public int NumBeams;
        public int NumRaysPerBeam;
        public int NumRangeBins;
        public float MaxRange;

        [NativeDisableParallelForRestriction]
        public NativeArray<float> Image;

        public void Execute(int beam)
        {
            NativeArray<float> binSums = new NativeArray<float>(NumRangeBins, Allocator.Temp);

            // 1. Accumulate all intensities into their respective bins.
            int start = beam * NumRaysPerBeam;
            for (int ray = 0; ray < NumRaysPerBeam; ray++)
            {
                PointXYZI p = LocalPoints[start + ray];
                float distance = math.length(p.position);
                if (distance <= 0f) continue; // a missed ray sits at the origin

                float t = MaxRange > 0f ? distance / MaxRange : 0f;
                int bin = math.clamp((int)math.floor(t * NumRangeBins), 0, NumRangeBins - 1);

                binSums[bin] += p.intensity;
            }

            // 2. Divide by the beam's fixed ray budget, not by how many of those rays
            // happened to land in this particular bin -- a bin only a few rays grazed
            // should come out dimmer than one the whole elevation fan covered.
            float normalizer = NumRaysPerBeam > 0 ? NumRaysPerBeam : 1;
            for (int r = 0; r < NumRangeBins; r++)
            {
                int idx = r * NumBeams + beam;
                Image[idx] = binSums[r] / normalizer;
            }

            binSums.Dispose();
        }
    }
}

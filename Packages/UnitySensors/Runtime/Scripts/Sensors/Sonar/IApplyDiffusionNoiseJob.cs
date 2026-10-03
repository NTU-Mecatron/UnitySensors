// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using Random = Unity.Mathematics.Random;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Applies exponential ("diffusion"/speckle) noise to each sonar hit, in place
    /// </summary>
    [BurstCompile]
    internal struct IApplyDiffusionNoiseJob : IJobParallelFor
    {
        public NativeArray<float3> LocalPoints;
        public NativeArray<float> Intensities;

        public float RangeNoiseMean;
        public float IntensityNoiseExponent;
        public uint Seed;                // per-cycle seed; the caller refreshes this every cycle

        public void Execute(int i)
        {
            float3 point = LocalPoints[i];
            float range = math.length(point);
            if (range <= 0f) return;

            Random rng = Random.CreateFromIndex(math.hash(new uint2((uint)i, Seed)));
            float u = 1f - rng.NextFloat(); 

            // Apply exponential noise on returned range.
            float noisedRange = range + -RangeNoiseMean * math.log(u);
            LocalPoints[i] = point / range * noisedRange;

            // Diminishing intensity is modelled using scaled exponential PDF, which perfectly reduces back to 'u'. 
            // The exponent is to tune the noise level. <1 reduces intensity penalty. >1 creates aggressive signal loss for highly turbid environments.
            float intensityScale = math.pow(u, IntensityNoiseExponent);
            Intensities[i] = Intensities[i] * intensityScale;
        }
    }
}

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
    /// Applies exponential ("diffusion"/speckle) noise to each sonar hit, in place, between
    /// <see cref="IUpdateSonarHitsJob"/> and <see cref="IPackSonarPointCloudJob"/>.
    ///
    /// Intensity gets unit-mean *multiplicative* noise: exponential is the power-domain
    /// speckle model -- when many random-phase scatterers sum inside one resolution cell,
    /// return intensity (power) is exponentially distributed even though the deterministic
    /// Lambertian value is only the correct mean. IntensityNoiseMean = 1 keeps it unbiased
    /// in expectation.
    ///
    /// Range gets *additive* noise instead, applied along the ray direction so a noised
    /// point stays on its original ray: raw range + Exponential(RangeNoiseMean). This models
    /// reverberation/multipath path-length spread rather than measurement jitter, so unlike
    /// the intensity term it is intentionally one-sided -- a hit only ever gets pushed
    /// farther, never closer, and RangeNoiseMean is the average extra distance in world
    /// units, not a "1 = unbiased" factor.
    ///
    /// Parallelized per ray. <see cref="Random"/> is a value type re-copied per worker
    /// batch, so one shared field advanced sequentially would replay the same sequence at
    /// the start of every batch. Instead each <see cref="Execute"/> derives its own stream
    /// from a hash of the ray index and the per-cycle <see cref="Seed"/> -- independent
    /// across rays within a cycle, and independent of itself across cycles as long as the
    /// caller changes <see cref="Seed"/> every cycle (e.g. from a frame counter); reusing
    /// the same seed freezes the speckle pattern to ray/bin index instead of letting it
    /// decorrelate the way real coherent speckle does.
    /// </summary>
    [BurstCompile]
    internal struct IApplyDiffusionNoiseJob : IJobParallelFor
    {
        public NativeArray<float3> LocalPoints;
        public NativeArray<float> Intensities;

        public float IntensityNoiseMean; // mean of the unit multiplicative speckle; 1 = unbiased
        public float RangeNoiseMean;     // mean extra distance (world units) added to range
        public uint Seed;                // per-cycle seed; the caller refreshes this every cycle

        public void Execute(int i)
        {
            float3 point = LocalPoints[i];
            float range = math.length(point);
            if (range <= 0f) return;

            Random rng = Random.CreateFromIndex(math.hash(new uint2((uint)i, Seed)));

            // Two independent draws from the same stream -- intensity and range noise are
            // different physical effects (speckle vs. multipath spread) and shouldn't be
            // coupled to a single sample.
            float intensityNoise = SampleExponential(ref rng, IntensityNoiseMean);
            Intensities[i] = Intensities[i] * intensityNoise;

            float rangeNoise = SampleExponential(ref rng, RangeNoiseMean);
            float noisedRange = range + rangeNoise;
            LocalPoints[i] = point / range * noisedRange;
        }

        private static float SampleExponential(ref Random rng, float mean)
        {
            mean = math.abs(mean);
            float u = 1f - rng.NextFloat(); // (0, 1], avoids log(0)
            return -mean * math.log(u);
        }
    }
}

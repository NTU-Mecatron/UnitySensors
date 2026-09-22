// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Random = Unity.Mathematics.Random;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Applies range/azimuth-dependent noise to the packed sonar image, in place, after
    /// <see cref="IPackSonarImageJob"/>. Runs last in the per-cycle chain since it needs the
    /// fully-binned <see cref="Image"/>.
    ///
    /// Parallelized per pixel. <see cref="Random"/> is a value type re-copied per worker
    /// batch, so each <see cref="Execute"/> derives its own stream from a hash of the pixel
    /// index and the per-cycle <see cref="Seed"/> -- same reasoning as
    /// <see cref="IApplyDiffusionNoiseJob"/>: independent across pixels within a cycle, and
    /// independent of itself across cycles as long as the caller changes <see cref="Seed"/>
    /// every cycle.
    /// </summary>
    [BurstCompile]
    internal struct IApplySonarImageNoiseJob : IJobParallelFor
    {
        public int NumRangeBins;
        public int NumBeams;
        public float MaxRange;
        public float FovDeg;
        public float NormalMean;
        public float NormalSigma;
        public float RayleighSigma;
        public uint Seed; // per-cycle seed; the caller refreshes this every cycle
        [NativeDisableParallelForRestriction]
        public NativeArray<float> Image;
        public void Execute(int i)
        {
            // Image is packed row-major by IPackSonarImageJob: idx = r * NumBeams + beam.
            int r = i / NumBeams;
            int beam = i % NumBeams;

            float range = (r + 0.5f) / NumRangeBins * MaxRange;
            float azimuthDegAbs = NumBeams > 1 ? Mathf.Abs((beam / (float)(NumBeams - 1) - 0.5f) * FovDeg - FovDeg / 2): 0f;
            float azimuthRadAbs = azimuthDegAbs * Mathf.Deg2Rad;
            
            Random rng = Random.CreateFromIndex(math.hash(new uint2((uint)i, Seed)));

            float w_sa = Mathf.Pow(range / MaxRange, 2) * (float)(1 + 0.5 * Mathf.Exp(-azimuthRadAbs * azimuthRadAbs)) * SampleRayleighNoise(ref rng, RayleighSigma);
            float w_sm = SampleGaussianNoise(ref rng, NormalMean, NormalSigma);

            // Apply additive and multiplicative noise
            Image[i] = Image[i] * (float)(0.5 + w_sm) + w_sa;

            // The gain-response curve below (2x - x^2) is only monotonic on [0, 1]; past
            // x = 1 it turns over and starts decreasing, so a bright/heavily-noised pixel
            // could fold back down toward a dark output. Clamp first so it always sees a
            // well-behaved input.
            Image[i] = math.clamp(Image[i], 0f, 1f);

            // Sensor gain-response curve (soft compression toward the bright end).
            Image[i] = 1 - Mathf.Pow((1 - Image[i]), 2);
            // Image[i] = Mathf.Pow(Image[i] * Image[i], 2) // If sonar has on-board gain reduction
        }

        private static float SampleGaussianNoise(ref Random random, float normalMean, float normalSigma)
        {
            // Box-Muller transform. u1 must be in (0, 1] so log(u1) is finite:
            // NextFloat() returns [0, 1), so flip it.
            float u1 = 1f - random.NextFloat();
            float u2 = random.NextFloat();

            float r = math.sqrt(-2f * math.log(u1));
            float theta = 2f * math.PI * u2;

            return normalMean + normalSigma * r * math.cos(theta);
        }
        private static float SampleRayleighNoise(ref Random random, float rayleighSigma)
        {
            // Inverse CDF: F(x) = 1 - exp(-x^2 / (2 sigma^2))  =>  x = sigma * sqrt(-2 ln(1 - u))
            // Using u' = 1 - u in (0, 1] keeps the log finite.
            // Mean of the result is sigma * sqrt(pi/2) ~= 1.2533 * sigma.
            float u = 1f - random.NextFloat();
            return rayleighSigma * math.sqrt(-2f * math.log(u));
        }
    }
}

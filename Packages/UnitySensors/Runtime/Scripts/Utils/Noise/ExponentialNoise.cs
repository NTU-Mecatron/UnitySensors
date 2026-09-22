// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using System;
using Vector3 = UnityEngine.Vector3;

namespace UnitySensors.Utils.Noise
{
    /// <summary>
    /// Exponentially-distributed noise, sampled via inverse-CDF. This is the power/intensity
    /// counterpart of <see cref="RayleighNoise"/>: if a signal's amplitude envelope is
    /// Rayleigh-distributed (many random-phase scatterers summing in one resolution cell),
    /// its squared magnitude -- i.e. return intensity, which is what a sonar/radar pixel
    /// actually reports -- is exponentially distributed. Parameterized directly by its mean,
    /// so passing mean = 1 gives unit-mean multiplicative speckle to apply straight onto an
    /// already-normalized intensity value.
    /// </summary>
    public class ExponentialNoise
    {
        private Random _random;

        public ExponentialNoise()
        {
            _random = new Random(Environment.TickCount);
        }

        public ExponentialNoise(int seed)
        {
            _random = new Random(seed);
        }

        public void Init(int seed)
        {
            _random = new Random(seed);
        }

        /// <summary>
        /// Generate a single exponentially-distributed sample with the given mean
        /// (mean = 1 / rate).
        /// </summary>
        public double GetNoise(double mean = 1.0d)
        {
            mean = Math.Abs(mean);

            // Inverse CDF: F(x) = 1 - exp(-x / mean) => x = -mean * ln(u).
            // u is drawn from (0, 1] so log(u) never blows up.
            double u = 1.0d - _random.NextDouble();
            return -mean * Math.Log(u);
        }

        public Vector3 GetNoise(Vector3 meanVector)
        {
            return new Vector3(
                (float)GetNoise(meanVector.x),
                (float)GetNoise(meanVector.y),
                (float)GetNoise(meanVector.z)
            );
        }
    }
}

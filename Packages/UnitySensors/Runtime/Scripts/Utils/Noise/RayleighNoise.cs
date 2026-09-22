// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using System;
using Vector3 = UnityEngine.Vector3;

namespace UnitySensors.Utils.Noise
{
    /// <summary>
    /// Rayleigh-distributed noise, sampled via inverse-CDF (no rejection loop, unlike
    /// <see cref="GaussianNoise"/>'s Box-Muller). Models the amplitude-domain envelope of
    /// coherent speckle: when many random-phase scatterers sum inside a resolution cell,
    /// the in-phase/quadrature components are each ~Gaussian, and the envelope of that sum
    /// is Rayleigh-distributed. Use this for a multiplicative amplitude speckle factor;
    /// see <see cref="ExponentialNoise"/> for the equivalent on squared (power/intensity)
    /// values.
    /// </summary>
    public class RayleighNoise
    {
        private Random _random;

        public RayleighNoise()
        {
            _random = new Random(Environment.TickCount);
        }

        public RayleighNoise(int seed)
        {
            _random = new Random(seed);
        }

        public void Init(int seed)
        {
            _random = new Random(seed);
        }

        /// <summary>
        /// Generate a single Rayleigh-distributed sample with scale <paramref name="sigma"/>
        /// (the distribution's mode). Mean = sigma * sqrt(pi/2) ~= 1.2533 * sigma, so to get
        /// unit-mean multiplicative speckle pass sigma = sqrt(2/pi) ~= 0.7979.
        /// </summary>
        public double GetNoise(double sigma = 1.0d)
        {
            sigma = Math.Abs(sigma);

            // Inverse CDF: F(x) = 1 - exp(-x^2 / (2*sigma^2)) => x = sigma * sqrt(-2 * ln(u)).
            // u is drawn from (0, 1] so log(u) never blows up.
            double u = 1.0d - _random.NextDouble();
            return sigma * Math.Sqrt(-2.0d * Math.Log(u));
        }

        public Vector3 GetNoise(Vector3 sigmaVector)
        {
            return new Vector3(
                (float)GetNoise(sigmaVector.x),
                (float)GetNoise(sigmaVector.y),
                (float)GetNoise(sigmaVector.z)
            );
        }
    }
}

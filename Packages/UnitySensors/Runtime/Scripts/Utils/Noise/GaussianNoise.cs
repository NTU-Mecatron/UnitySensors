using System;
using Vector3 = UnityEngine.Vector3;

namespace UnitySensors.Utils.Noise
{
    public class GaussianNoise
    {
        private Random _random;
        private double? _spare; // To store the spare (second) value from Box-Muller transform

        public GaussianNoise()
        {
            _random = new Random(Environment.TickCount);
        }

        public GaussianNoise(int seed)
        {
            _random = new Random(seed);
        }

        public void Init(int seed)
        {
            _random = new Random(seed);
        }

        /// <summary>
        /// Generate a single value of noise based on standard deviation.
        /// </summary>
        public double GetNoise(double sigma = 1.0d)
        {
            // Make sure sigma is positive
            sigma = Math.Abs(sigma);

            // Use spare value from previous call if available
            if (_spare.HasValue)
            {
                double noise = _spare.Value;
                _spare = null; // Clear spare
                return sigma * noise;
            }

            // Box-Muller transform to generate two independent standard normal variables
            double u, v, s;
            do
            {
                u = _random.NextDouble() * 2.0 - 1.0;
                v = _random.NextDouble() * 2.0 - 1.0;
                s = u * u + v * v;
            } while (s == 0.0 || s >= 1.0); // Throw away samples outside the unit circle

            // Turn u and v flat randoms into Gaussian
            double multiplier = Math.Sqrt(-2.0 * Math.Log(s) / s);

            // Save the second result for next call
            _spare = v * multiplier;

            // Return the first result
            return sigma * (u * multiplier);
        }

        public Vector3 GetNoise(Vector3 sigmaVector)
        {
            // Implementation Note: GetNoise(1.0) returns N(0,1). 
            // We then multiply by the specific sigma for that axis.
            return new Vector3(
                (float)(GetNoise(1.0) * sigmaVector.x),
                (float)(GetNoise(1.0) * sigmaVector.y),
                (float)(GetNoise(1.0) * sigmaVector.z)
            );
        }
    }
}
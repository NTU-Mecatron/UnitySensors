using UnityEngine;

namespace UnitySensors.Interface.Sensor
{
    public interface IImuDeltaDataInterface
    {
        public double DeltaTime { get; }
        public Vector3 DeltaVelocity { get; }
        public Quaternion DeltaOrientation { get; }
    }
}

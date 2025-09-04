using UnityEngine;

namespace UnitySensors.Interface.Sensor
{
    public interface IImuDataInterface
    {
        public Vector3 linearAcceleration { get; }
        public Quaternion orientation { get; }
        public Vector3 angularVelocity { get; }
    }
}

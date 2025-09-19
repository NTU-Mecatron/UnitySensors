using UnityEngine;

namespace UnitySensors.Interface.Sensor
{
    public interface IImuDataInterface
    {
        public Vector3 LinearAcceleration { get; }
        public Quaternion Orientation { get; }
        public Vector3 AngularVelocity { get; }
    }
}

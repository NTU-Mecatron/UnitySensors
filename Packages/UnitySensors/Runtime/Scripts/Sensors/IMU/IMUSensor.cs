using System.Collections;
using UnityEngine;
using UnityEngine.Assertions;
using UnitySensors.Attribute;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.IMU
{
    public class IMUSensor : UnityPhysicsSensor, IImuDataInterface
    {
        // Mostly copied from https://github.com/MARUSimulator/marus-core/blob/21c003a384335777b9d9fb6805eeab1cdb93b2f0/Scripts/Sensors/Primitive/ImuSensor.cs
        // Thank you guys <3
        [Header("IMU")]
        public bool withGravity = true;

        [SerializeField] Rigidbody rigidBody;

        [Header("Current values")]
        public Vector3 linearVelocity;
        public Vector3 linearAcceleration { get; private set; }
        [SerializeField] Vector3 _linearAcceleration;
        [HideInInspector] public double[] linearAccelerationCovariance = new double[9];

        public Vector3 angularVelocity { get; private set; }
        [SerializeField] Vector3 _angularVelocity;
        [HideInInspector] public double[] angularVelocityCovariance = new double[9];

        public Vector3 eulerAngles;
        public Quaternion orientation { get; private set; }
        [SerializeField] Quaternion _orientation;
        [HideInInspector] public double[] orientationCovariance = new double[9];

        private Vector3 lastVelocity = Vector3.zero;

        public override bool UpdateSensor(double deltaTime)
        {
            linearVelocity = rigidBody.transform.InverseTransformVector(rigidBody.velocity);

            if (deltaTime > 0)
            {
                Vector3 deltaLinearAcceleration = linearVelocity - lastVelocity;
                linearAcceleration = deltaLinearAcceleration / (float)deltaTime;
            }

            angularVelocity = -rigidBody.transform.InverseTransformVector(rigidBody.angularVelocity);
            eulerAngles = rigidBody.transform.rotation.eulerAngles;
            orientation = Quaternion.Euler(eulerAngles);

            lastVelocity = linearVelocity;

            if (withGravity)
            {
                // Find the global gravity in the local frame and add to the computed linear acceleration
                Vector3 localGravity = rigidBody.transform.InverseTransformDirection(Physics.gravity);
                linearAcceleration += localGravity;
            }

            _linearAcceleration = linearAcceleration;
            _angularVelocity = angularVelocity;
            _orientation = orientation;

            return true;
        }

        protected override void Init()
        {
            Assert.IsNotNull(rigidBody, "No Rigidbody assigned to IMU sensor!");
        }

        protected override void OnSensorDestroy()
        {
        }
    }
}

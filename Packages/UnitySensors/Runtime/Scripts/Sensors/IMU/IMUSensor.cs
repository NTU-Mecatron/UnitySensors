using System;
using UnityEngine;
using UnityEngine.Assertions;
using UnitySensors.Interface.Sensor;
using UnitySensors.Utils.Noise;

namespace UnitySensors.Sensor.IMU
{
    [RequireComponent(typeof(ArticulationBody))]
    public class IMUSensor : UnityPhysicsSensor, IImuDataInterface, IImuDeltaDataInterface
    {
        // Mostly copied from https://github.com/MARUSimulator/marus-core/blob/21c003a384335777b9d9fb6805eeab1cdb93b2f0/Scripts/Sensors/Primitive/ImuSensor.cs
        // Thank you guys <3
        [Header("IMU")]
        public bool withGravity = true;

        ArticulationBody artiBody;

        [Header("Current values (for monitoring only)")]
        public Vector3 linearVelocity;
        public Vector3 LinearAcceleration { get; private set; }
        [SerializeField] Vector3 _linearAcceleration;
        public double[] linearAccelerationCovariance = new double[9];

        public Vector3 AngularVelocity { get; private set; }
        [SerializeField] Vector3 _angularVelocity;
        public double[] angularVelocityCovariance = new double[9];

        public Vector3 eulerAngles;
        public Quaternion Orientation { get; private set; }
        [SerializeField] Quaternion _orientation;
        public double[] orientationCovariance = new double[9];

        public double DeltaTime { get; private set; }
        public Vector3 DeltaVelocity { get; private set; }
        public Quaternion DeltaOrientation { get; private set; }    // Not implemented for now

        private Vector3 lastVelocity = Vector3.zero;

        private Quaternion orientationOnReset;  // Store the initial heading of the robot (no roll or pitch) to use as the reference orientation for the IMU readings
        private Vector3 gravity = new (0, 9.8065f, 0);

        GaussianNoise _noise;
        Vector3 _accelStd;
        Vector3 _gyroStd;
        Vector3 _oriStd;

        protected override void Init()
        {
            artiBody = GetComponent<ArticulationBody>();
            Assert.IsNotNull(artiBody, "No ArticulationBody for IMU sensor!");

            ResetHeading();

            _noise = new();

            float accelStdDevX = (float)Math.Sqrt(Math.Max(1e-8, linearAccelerationCovariance[0]));
            float accelStdDevY = (float)Math.Sqrt(Math.Max(1e-8, linearAccelerationCovariance[4]));
            float accelStdDevZ = (float)Math.Sqrt(Math.Max(1e-8, linearAccelerationCovariance[8]));
            _accelStd = new Vector3(accelStdDevX, accelStdDevY, accelStdDevZ);

            float gyroStdDevX = (float)Math.Sqrt(Math.Max(1e-8, angularVelocityCovariance[0]));
            float gyroStdDevY = (float)Math.Sqrt(Math.Max(1e-8, angularVelocityCovariance[4]));
            float gyroStdDevZ = (float)Math.Sqrt(Math.Max(1e-8, angularVelocityCovariance[8]));
            _gyroStd = new Vector3(gyroStdDevX, gyroStdDevY, gyroStdDevZ);

            float oriStdDevX = (float)Math.Sqrt(Math.Max(1e-8, orientationCovariance[0]));
            float oriStdDevY = (float)Math.Sqrt(Math.Max(1e-8, orientationCovariance[4]));
            float oriStdDevZ = (float)Math.Sqrt(Math.Max(1e-8, orientationCovariance[8]));
            _oriStd = new Vector3(oriStdDevX, oriStdDevY, oriStdDevZ);
        }

        public override bool UpdateSensor(double deltaTime)
        {
            linearVelocity = artiBody.transform.InverseTransformVector(artiBody.linearVelocity);

            // Noise
            Vector3 accNoise = _noise.GetNoise(_accelStd);
            Vector3 gyroNoise = _noise.GetNoise(_gyroStd);
            Vector3 oriNoise = _noise.GetNoise(_oriStd);
            //Vector3 bias = new(1e-2f, 3e-2f, 5e-3f);
            Vector3 bias = Vector3.zero;

            if (deltaTime > 0)
            {
                DeltaTime = deltaTime;
                DeltaVelocity = linearVelocity - lastVelocity;              
                DeltaVelocity += (accNoise + bias) * (float)DeltaTime;
                LinearAcceleration = DeltaVelocity / (float)DeltaTime;
            }

            AngularVelocity = -artiBody.transform.InverseTransformVector(artiBody.angularVelocity);
            AngularVelocity += gyroNoise;

            Orientation = Quaternion.Inverse(orientationOnReset) * artiBody.transform.rotation;
            Orientation = Quaternion.Euler(Orientation.eulerAngles + oriNoise);
            eulerAngles = Orientation.eulerAngles;          

            lastVelocity = linearVelocity;

            if (withGravity)
            {
                // Find the global gravity in the local frame and add to the computed linear acceleration
                Vector3 localGravity = artiBody.transform.InverseTransformDirection(gravity);
                LinearAcceleration += localGravity;
                DeltaVelocity += localGravity * (float)DeltaTime;
            }

            _linearAcceleration = LinearAcceleration;
            _angularVelocity = AngularVelocity;
            _orientation = Orientation;

            return true;
        }

        /// <summary>
        /// Save the current heading of the robot as the reference orientation for the IMU readings
        /// </summary>
        public void ResetHeading()
        {
            float headingOnReset = artiBody.transform.eulerAngles.y;
            orientationOnReset = Quaternion.Euler(0, headingOnReset, 0);
        }

        public void SetGravity(float gravityConstant)
        {
            gravity = new (0, gravityConstant, 0);
        }

        protected override void OnSensorDestroy()
        {
        }
    }
}

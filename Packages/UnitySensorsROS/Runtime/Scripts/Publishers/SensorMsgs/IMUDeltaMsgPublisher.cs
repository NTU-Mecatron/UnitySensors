using RosMessageTypes.Vectornav;
using UnitySensors.ROS.Serializer.Sensor;
using UnitySensors.Sensor.IMU;
using UnityEngine;

namespace UnitySensors.ROS.Publisher.Sensor
{
    [RequireComponent(typeof(IMUSensor))]
    public class IMUDeltaMsgPublisher : RosMsgPublisher<IMUDeltaMsgSerializer, DeltaGroupMsg>
    {
        void Reset()
        {
            _topicName = "imu/delta_data";
            _frequency = 50.0f;
            _serializer.Source = GetComponent<IMUSensor>();
            _serializer.Header.Source = GetComponent<IMUSensor>();
            _serializer.Header.FrameId = "imu_link";
        }
    }
}

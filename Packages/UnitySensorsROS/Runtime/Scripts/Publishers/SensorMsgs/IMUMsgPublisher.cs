using RosMessageTypes.Sensor;
using UnitySensors.ROS.Serializer.Sensor;
using UnitySensors.Sensor.IMU;
using UnityEngine;

namespace UnitySensors.ROS.Publisher.Sensor
{
    [RequireComponent(typeof(IMUSensor))]
    public class IMUMsgPublisher : RosMsgPublisher<IMUMsgSerializer, ImuMsg>
    {
        void Reset()
        {
            _topicName = "imu/data";
            _frequency = 4.0f;
            _serializer.Source = GetComponent<IMUSensor>();
            _serializer.Header.Source = GetComponent<IMUSensor>();
            _serializer.Header.FrameId = "imu_link";
        }
    }
}

using UnityEngine;
using RosMessageTypes.Tf2;
using UnitySensors.ROS.Serializer.Tf2;
using UnitySensors.Sensor.TF;

namespace UnitySensors.ROS.Publisher.Tf2
{
    [RequireComponent(typeof(TFLink))]
    public class TFMessageMsgPublisher : RosMsgPublisher<TFMessageMsgSerializer, TFMessageMsg>
    {
        void Reset()
        {
            _topicName = "/tf";
            _frequency = 50.0f;
        }
    }
}
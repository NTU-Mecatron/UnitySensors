using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;

using UnitySensors.Attribute;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [System.Serializable]
    public class IMUMsgSerializer : RosMsgSerializer<ImuMsg>
    {
        [SerializeField, Interface(typeof(IImuDataInterface))]
        private Object _source;
        [SerializeField]
        private HeaderSerializer _header;

        public Object Source { get => _source; set => _source = value; }
        public HeaderSerializer Header { get => _header; set => _header = value; }

        private IImuDataInterface _sourceInterface;

        public override void Init()
        {
            base.Init();
            _header.Init();
            _sourceInterface = _source as IImuDataInterface;
        }

        public override ImuMsg Serialize()
        {
            _msg.header = _header.Serialize();
            _msg.linear_acceleration = _sourceInterface.linearAcceleration.To<FLU>();
            _msg.orientation = _sourceInterface.orientation.To<FLU>();
            _msg.angular_velocity = _sourceInterface.angularVelocity.To<FLU>();
            return _msg;
        }
    }
}

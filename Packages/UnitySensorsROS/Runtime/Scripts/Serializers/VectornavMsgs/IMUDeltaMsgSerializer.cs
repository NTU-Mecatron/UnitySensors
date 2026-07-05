using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Vectornav;

using UnitySensors.Attribute;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [System.Serializable]
    public class IMUDeltaMsgSerializer : RosMsgSerializer<DeltaGroupMsg>
    {
        [SerializeField, Interface(typeof(IImuDeltaDataInterface))]
        private Object _source;
        [SerializeField]
        private HeaderSerializer _header;

        public Object Source { get => _source; set => _source = value; }
        public HeaderSerializer Header { get => _header; set => _header = value; }

        private IImuDeltaDataInterface _sourceInterface;

        public override void Init()
        {
            base.Init();
            _header.Init();
            _sourceInterface = _source as IImuDeltaDataInterface;
        }

        public override DeltaGroupMsg Serialize()
        {
            _msg.header = _header.Serialize();
            _msg.deltatheta_dvel = _sourceInterface.DeltaVelocity.To<FLU>();
            _msg.deltatheta_dtime = (float)_sourceInterface.DeltaTime;
            // _msg.deltatheta_dtheta = _sourceInterface.DeltaAngle.To<FLU>(); // Ignored
            return _msg;
        }
    }
}

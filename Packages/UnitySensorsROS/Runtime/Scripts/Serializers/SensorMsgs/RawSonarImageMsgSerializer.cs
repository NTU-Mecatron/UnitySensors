// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using UnityEngine;

using RosMessageTypes.Sensor;

using UnitySensors.ROS.Serializer.Std;
using UnitySensors.Sensor.Sonar;

namespace UnitySensors.ROS.Serializer.Sensor
{
    /// <summary>
    /// Serializes a <see cref="SonarSensor"/>'s per-cycle <see cref="SonarSensor.SonarImage"/>
    /// as a mono8 <c>sensor_msgs/Image</c>: width = NumBeams (bearing), height =
    /// <c>NumRangeBins</c> (range). This is the raw bearing/range grid a real FLS driver
    /// publishes, not the Cartesian "wedge" picture -- that's a downstream projection of
    /// this data (see <see cref="FlsFanImageMsgSerializer"/>).
    ///
    /// Reads <see cref="SonarSensor.SonarImage"/> directly (the sensor's own averaged, noised image).
    /// That ties this message's resolution to the sensor's own <c>NumRangeBins</c>/<c>NumBeams</c>;
    /// there is no longer an independently configurable output resolution.
    /// </summary>
    [System.Serializable]
    public class RawSonarImageMsgSerializer : RosMsgSerializer<ImageMsg>
    {
        [SerializeField]
        private HeaderSerializer _header;

        private SonarSensor _sourceInterface;
        private int _numBeams;
        private int _numRangeBins;

        public HeaderSerializer Header { get => _header; set => _header = value; }

        public void SetSource(SonarSensor source)
        {
            _sourceInterface = source;
        }

        public override void Init()
        {
            base.Init();
            _header.Init();

            _numBeams = _sourceInterface.NumBeams;
            _numRangeBins = _sourceInterface.NumRangeBins;

            _msg.encoding = "mono8";
            _msg.is_bigendian = 0;
            _msg.width = (uint)_numBeams;
            _msg.height = (uint)_numRangeBins;
            _msg.step = (uint)_numBeams;
            _msg.data = new byte[_numBeams * _numRangeBins];
        }

        public override ImageMsg Serialize()
        {
            _msg.header = _header.Serialize();

            var sonarImage = _sourceInterface.SonarImage;
            for (int idx = 0; idx < sonarImage.Length; idx++)
            {
                _msg.data[idx] = (byte)Mathf.Clamp(sonarImage[idx] * 255f, 0f, 255f);
            }

            return _msg;
        }

        public override void OnDestroy() { }
    }
}

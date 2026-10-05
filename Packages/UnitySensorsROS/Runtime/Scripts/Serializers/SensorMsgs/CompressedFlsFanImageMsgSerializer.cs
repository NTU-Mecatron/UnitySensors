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
    /// JPEG-compressed variant of <see cref="FlsFanImageMsgSerializer"/>: produces the same
    /// Cartesian "fan" picture, published as a <c>sensor_msgs/CompressedImage</c>.
    /// The fan warp itself is delegated to the wrapped serializer.
    /// </summary>
    [System.Serializable]
    public class CompressedFlsFanImageMsgSerializer : RosMsgSerializer<CompressedImageMsg>
    {
        [SerializeField]
        private FlsFanImageMsgSerializer _image;
        [SerializeField, Range(1, 100)]
        private int _quality = 75;

        private readonly Mono8JpegEncoder _encoder = new();

        public HeaderSerializer Header { get => _image.Header; set => _image.Header = value; }
        public float Contrast { get => _image.Contrast; set => _image.Contrast = value; }

        public void SetSource(ForwardLookingSonarSensor source)
        {
            _image.SetSource(source);
        }

        public override void Init()
        {
            base.Init();
            _image.Init();
            _msg.format = "jpeg";
        }

        public override CompressedImageMsg Serialize()
        {
            ImageMsg image = _image.Serialize();
            _msg.header = image.header;
            _msg.data = _encoder.Encode(image.data, (int)image.width, (int)image.height, _quality);
            return _msg;
        }

        public override void OnDestroy()
        {
            _image.OnDestroy();
        }
    }
}

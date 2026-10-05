// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using UnityEngine;

using RosMessageTypes.Sensor;

using UnitySensors.ROS.Serializer.Sensor;
using UnitySensors.Sensor.Sonar;

namespace UnitySensors.ROS.Publisher.Sensor
{
    /// <summary>
    /// Publishes a <see cref="ForwardLookingSonarSensor"/>'s Cartesian "fan" picture as a
    /// JPEG <c>sensor_msgs/CompressedImage</c> -- see <see cref="CompressedFlsFanImageMsgSerializer"/>.
    /// </summary>
    public class CompressedFlsFanImageMsgPublisher : RosMsgPublisher<CompressedFlsFanImageMsgSerializer, CompressedImageMsg>
    {
        [SerializeField]
        private ForwardLookingSonarSensor _source;

        protected override void InitializePublisher()
        {
            base.InitializePublisher();

            if (_source == null)
            {
                Debug.LogError("Source is not set in CompressedFlsFanImageMsgPublisher. Please ensure that the '_source' field is assigned in the Unity Editor or via code. Expected type: ForwardLookingSonarSensor.");
                return;
            }
            _serializer.SetSource(_source);
        }
    }
}

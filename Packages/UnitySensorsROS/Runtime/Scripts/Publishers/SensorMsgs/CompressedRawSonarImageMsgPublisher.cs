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
    /// Publishes a <see cref="SonarSensor"/>'s raw bearing x range grid as a JPEG
    /// <c>sensor_msgs/CompressedImage</c> -- see <see cref="CompressedRawSonarImageMsgSerializer"/>.
    /// </summary>
    public class CompressedRawSonarImageMsgPublisher : RosMsgPublisher<CompressedRawSonarImageMsgSerializer, CompressedImageMsg>
    {
        [SerializeField]
        private SonarSensor _source;

        protected override void InitializePublisher()
        {
            base.InitializePublisher();

            if (_source == null)
            {
                Debug.LogError("Source is not set in CompressedRawSonarImageMsgPublisher. Please ensure that the '_source' field is assigned in the Unity Editor or via code. Expected type: SonarSensor.");
                return;
            }
            _serializer.SetSource(_source);
        }
    }
}

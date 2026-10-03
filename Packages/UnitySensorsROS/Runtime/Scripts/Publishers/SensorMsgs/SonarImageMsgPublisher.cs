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
    /// Publishes a <see cref="SonarSensor"/>'s per-cycle hits as a rectangular bearing x
    /// range <c>sensor_msgs/Image</c> (mono8): columns = beam index, rows = range bin,
    /// pixel = strongest return intensity in that beam/range cell. This is the raw image
    /// most FLS ROS drivers publish; a Cartesian "wedge" picture is a downstream
    /// projection of it, not something this publisher produces.
    /// </summary>
    public class SonarImageMsgPublisher : RosMsgPublisher<SonarImageMsgSerializer, ImageMsg>
    {
        [SerializeField]
        private SonarSensor _source;

        protected override void InitializePublisher()
        {
            base.InitializePublisher();

            if (_source == null)
            {
                Debug.LogError("Source is not set in SonarImageMsgPublisher. Please ensure that the '_source' field is assigned in the Unity Editor or via code. Expected type: SonarSensor.");
                return;
            }
            _serializer.SetSource(_source);
        }
    }
}

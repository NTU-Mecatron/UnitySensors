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
    /// Publishes a <see cref="ForwardLookingSonarSensor"/>'s per-cycle hits as the
    /// Cartesian "fan" picture (mono8) operators expect, instead of the raw bearing/range
    /// grid -- see <see cref="SonarFanImageMsgSerializer"/> for the warp details.
    /// </summary>
    public class SonarFanImageMsgPublisher : RosMsgPublisher<SonarFanImageMsgSerializer, ImageMsg>
    {
        [SerializeField]
        private ForwardLookingSonarSensor _source;

        protected override void InitializePublisher()
        {
            base.InitializePublisher();

            if (_source == null)
            {
                Debug.LogError("Source is not set in SonarFanImageMsgPublisher. Please ensure that the '_source' field is assigned in the Unity Editor or via code. Expected type: ForwardLookingSonarSensor.");
                return;
            }
            _serializer.SetSource(_source);
        }
    }
}

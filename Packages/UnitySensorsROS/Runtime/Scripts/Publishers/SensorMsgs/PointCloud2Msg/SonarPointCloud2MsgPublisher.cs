// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using UnityEngine;
using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Sensor.Sonar;

namespace UnitySensors.ROS.Publisher.Sensor
{
    /// <summary>
    /// Publishes a <see cref="SonarSensor"/>'s per-cycle hits as a
    /// <c>sensor_msgs/PointCloud2</c>. Any component implementing
    /// <see cref="IPointCloudInterface{T}"/> of <see cref="PointXYZI"/> is a valid source
    /// (the sonar sensors expose one directly). Points are in the sensor-local frame;
    /// set the serializer's header frame_id accordingly.
    /// </summary>
    public class SonarPointCloud2MsgPublisher : PointCloud2MsgPublisher<PointXYZI>
    {
        [SerializeField, Interface(typeof(IPointCloudInterface<PointXYZI>))]
        private Object _source;

        protected override void InitializePublisher()
        {
            base.InitializePublisher();

            if (_source == null)
            {
                Debug.LogError("Source is not set in SonarPointCloud2MsgPublisher. Please ensure that the '_source' field is assigned in the Unity Editor or via code. Expected type: IPointCloudInterface<PointXYZI>.");
                return;
            }
            _serializer.SetSource(_source as IPointCloudInterface<PointXYZI>);
        }
    }
}

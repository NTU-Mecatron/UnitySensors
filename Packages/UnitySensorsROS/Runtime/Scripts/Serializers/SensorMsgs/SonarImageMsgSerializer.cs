// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using UnityEngine;
using Unity.Collections;
using Unity.Jobs;

using RosMessageTypes.Sensor;

using UnitySensors.ROS.Serializer.Std;
using UnitySensors.Sensor.Sonar;

namespace UnitySensors.ROS.Serializer.Sensor
{
    /// <summary>
    /// Serializes a <see cref="SonarSensor"/>'s per-cycle hits as a mono8
    /// <c>sensor_msgs/Image</c>: width = NumBeams (bearing), height = <see cref="_numRangeBins"/>
    /// (range). This is the raw bearing/range grid a real FLS driver publishes, not the
    /// Cartesian "wedge" picture -- that's a downstream projection of this data.
    /// </summary>
    [System.Serializable]
    public class SonarImageMsgSerializer : RosMsgSerializer<ImageMsg>
    {
        [SerializeField]
        private HeaderSerializer _header;
        [SerializeField, Min(1), Tooltip("Range resolution (image rows). Columns are fixed at the sonar's NumBeams.")]
        private int _numRangeBins = 640;

        private SonarSensor _sourceInterface;
        private int _numBeams;

        private JobHandle _jobHandle;
        private IUpdateSonarImageJob _updateSonarImageJob;
        private NativeArray<byte> _image;

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

            _msg.encoding = "mono8";
            _msg.is_bigendian = 0;
            _msg.width = (uint)_numBeams;
            _msg.height = (uint)_numRangeBins;
            _msg.step = (uint)_numBeams;
            _msg.data = new byte[_numBeams * _numRangeBins];

            _image = new NativeArray<byte>(_numBeams * _numRangeBins, Allocator.Persistent);

            _updateSonarImageJob = new IUpdateSonarImageJob
            {
                Points = _sourceInterface.pointCloud.points,
                NumBeams = _numBeams,
                NumRaysPerBeam = _sourceInterface.NumRaysPerBeam,
                NumRangeBins = _numRangeBins,
                MaxRange = _sourceInterface.MaxRange,
                Image = _image
            };
        }

        public override ImageMsg Serialize()
        {
            _msg.header = _header.Serialize();

            _updateSonarImageJob.Points = _sourceInterface.pointCloud.points;
            _updateSonarImageJob.MaxRange = _sourceInterface.MaxRange;

            _jobHandle = _updateSonarImageJob.Schedule(_numBeams, 8);
            _jobHandle.Complete();

            _image.CopyTo(_msg.data);

            return _msg;
        }

        public override void OnDestroy()
        {
            _jobHandle.Complete();
            if (_image.IsCreated) _image.Dispose();
        }
    }
}

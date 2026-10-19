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
        [SerializeField, Min(0f), Tooltip("Gain applied to raw intensity before quantizing to mono8 (cv2.convertScaleAbs alpha in the reference driver). 1 = no change.")]
        private float _contrast = 1f;

        private SonarSensor _sourceInterface;
        private int _numBeams;
        private int _numRangeBins;

        private JobHandle _jobHandle;
        private IQuantizeSonarImageJob _quantizeJob;
        private NativeArray<byte> _rawImage;

        public HeaderSerializer Header { get => _header; set => _header = value; }
        public float Contrast { get => _contrast; set => _contrast = Mathf.Max(value, 0f); }

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

            _rawImage = new NativeArray<byte>(_numBeams * _numRangeBins, Allocator.Persistent);

            _quantizeJob = new IQuantizeSonarImageJob
            {
                Raw = _sourceInterface.SonarImage,
                Contrast = _contrast,
                Dst = _rawImage
            };
        }

        public override ImageMsg Serialize()
        {
            _msg.header = _header.Serialize();

            // Re-read every cycle rather than relying on the handle captured in Init
            // (same pattern as FlsFanImageMsgSerializer).
            _quantizeJob.Raw = _sourceInterface.SonarImage;
            _quantizeJob.Contrast = _contrast;

            _jobHandle = _quantizeJob.Schedule(_rawImage.Length, 64);
            _jobHandle.Complete();

            _rawImage.CopyTo(_msg.data);

            return _msg;
        }

        public override void OnDestroy()
        {
            _jobHandle.Complete();
            if (_rawImage.IsCreated) _rawImage.Dispose();
        }
    }
}

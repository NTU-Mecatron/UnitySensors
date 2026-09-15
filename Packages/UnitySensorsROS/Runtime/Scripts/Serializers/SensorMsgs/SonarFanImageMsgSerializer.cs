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
    /// Serializes a <see cref="ForwardLookingSonarSensor"/>'s per-cycle hits as the
    /// Cartesian "fan" picture operators expect: a polar-to-Cartesian warp of the raw
    /// bearing/range grid (see <see cref="SonarImageMsgSerializer"/> for that raw format),
    /// replicating sonoptix_sonar's echo_imager.py -- mono8, not the driver's bgr8 +
    /// VIRIDIS colormap. Two deliberate departures from the reference: it uses this
    /// sensor's actual <c>FLSFOVDeg</c> directly instead of the driver's 90/120 degree
    /// range-based guess (we have ground truth it doesn't), and it reads <c>MaxRange</c>
    /// straight off the sensor instead of round-tripping it through embedded image
    /// telemetry (unnecessary here since both ends live in the same process).
    /// </summary>
    [System.Serializable]
    public class SonarFanImageMsgSerializer : RosMsgSerializer<ImageMsg>
    {
        [SerializeField]
        private HeaderSerializer _header;
        [SerializeField, Min(1), Tooltip("Range resolution of the intermediate polar grid before warping.")]
        private int _numRangeBins = 200;
        [SerializeField, Min(0f), Tooltip("Gain applied to raw intensity before warping (cv2.convertScaleAbs alpha in the reference driver). 1 = no change.")]
        private float _contrast = 1f;
        [SerializeField, Range(0, 255), Tooltip("Fill value for pixels outside the fan's wedge shape (the bounding rectangle's corners). 0 = black (matches the reference driver), 128 = gray.")]
        private byte _backgroundValue = 128;

        private ForwardLookingSonarSensor _sourceInterface;
        private int _numBeams;
        private int _dstWidth;
        private int _dstHeight;

        private JobHandle _jobHandle;
        private IUpdateSonarImageJob _updatePolarImageJob;
        private IWarpSonarFanImageJob _warpFanImageJob;
        private NativeArray<byte> _polarImage;
        private NativeArray<byte> _fanImage;

        public HeaderSerializer Header { get => _header; set => _header = value; }

        public void SetSource(ForwardLookingSonarSensor source)
        {
            _sourceInterface = source;
        }

        public override void Init()
        {
            base.Init();
            _header.Init();

            _numBeams = _sourceInterface.NumBeams;
            _dstHeight = _numRangeBins;

            float fovRad = _sourceInterface.FLSFOVDeg * Mathf.Deg2Rad;
            _dstWidth = Mathf.Max(1, Mathf.CeilToInt(2f * _dstHeight * Mathf.Sin(fovRad / 2f)));

            _msg.encoding = "mono8";
            _msg.is_bigendian = 0;
            _msg.width = (uint)_dstWidth;
            _msg.height = (uint)_dstHeight;
            _msg.step = (uint)_dstWidth;
            _msg.data = new byte[_dstWidth * _dstHeight];

            _polarImage = new NativeArray<byte>(_numBeams * _numRangeBins, Allocator.Persistent);
            _fanImage = new NativeArray<byte>(_dstWidth * _dstHeight, Allocator.Persistent);

            _updatePolarImageJob = new IUpdateSonarImageJob
            {
                Points = _sourceInterface.pointCloud.points,
                NumBeams = _numBeams,
                NumRaysPerBeam = _sourceInterface.NumRaysPerBeam,
                NumRangeBins = _numRangeBins,
                MaxRange = _sourceInterface.MaxRange,
                Image = _polarImage
            };

            _warpFanImageJob = new IWarpSonarFanImageJob
            {
                Raw = _polarImage,
                SrcWidth = _numBeams,
                SrcHeight = _numRangeBins,
                FovRad = fovRad,
                Contrast = _contrast,
                BorderValue = _backgroundValue,
                DstWidth = _dstWidth,
                Dst = _fanImage
            };
        }

        public override ImageMsg Serialize()
        {
            _msg.header = _header.Serialize();

            _updatePolarImageJob.Points = _sourceInterface.pointCloud.points;
            _updatePolarImageJob.MaxRange = _sourceInterface.MaxRange;
            _warpFanImageJob.Contrast = _contrast;
            _warpFanImageJob.BorderValue = _backgroundValue;

            JobHandle polarHandle = _updatePolarImageJob.Schedule(_numBeams, 8);
            _jobHandle = _warpFanImageJob.Schedule(_dstWidth * _dstHeight, 64, polarHandle);
            _jobHandle.Complete();

            _fanImage.CopyTo(_msg.data);

            return _msg;
        }

        public override void OnDestroy()
        {
            _jobHandle.Complete();
            if (_polarImage.IsCreated) _polarImage.Dispose();
            if (_fanImage.IsCreated) _fanImage.Dispose();
        }
    }
}

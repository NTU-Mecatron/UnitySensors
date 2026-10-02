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
    /// Serializes a <see cref="ForwardLookingSonarSensor"/>'s per-cycle
    /// <see cref="SonarSensor.SonarImage"/> as the Cartesian "fan" picture operators expect:
    /// a polar-to-Cartesian warp of the sensor's own bearing/range grid (see
    /// <see cref="SonarImageMsgSerializer"/> for that raw format), replicating
    /// sonoptix_sonar's echo_imager.py -- mono8, not the driver's bgr8 + VIRIDIS colormap.
    ///
    /// This reads <see cref="SonarSensor.SonarImage"/> directly rather than re-binning the
    /// point cloud itself, so the fan reflects the sensor's own averaging and noise
    /// (diffusion + image-level) instead of a separate, divergent max-based rebin. That also
    /// means the polar grid's resolution is the sensor's own <c>NumRangeBins</c>/<c>NumBeams</c>,
    /// not an independently configurable resolution -- there is no longer a knob to rebin at
    /// a different resolution downstream without duplicating the sensor's binning logic.
    ///
    /// Two deliberate departures from the reference driver: it uses this sensor's actual
    /// <c>FLSFOVDeg</c> directly instead of the driver's 90/120 degree range-based guess (we
    /// have ground truth it doesn't), and it reads <c>MaxRange</c> straight off the sensor
    /// instead of round-tripping it through embedded image telemetry (unnecessary here since
    /// both ends live in the same process).
    /// </summary>
    [System.Serializable]
    public class SonarFanImageMsgSerializer : RosMsgSerializer<ImageMsg>
    {
        [SerializeField]
        private HeaderSerializer _header;
        [SerializeField, Min(0f), Tooltip("Gain applied to raw intensity before warping (cv2.convertScaleAbs alpha in the reference driver). 1 = no change.")]
        private float _contrast = 1f;
        [SerializeField, Range(0, 255), Tooltip("Fill value for pixels outside the fan's wedge shape (the bounding rectangle's corners). 0 = black (matches the reference driver), 128 = gray.")]
        private byte _backgroundValue = 128;

        private ForwardLookingSonarSensor _sourceInterface;
        private int _numBeams;
        private int _dstWidth;
        private int _dstHeight;

        private JobHandle _jobHandle;
        private IWarpSonarFanImageJob _warpFanImageJob;
        private NativeArray<byte> _fanImage;

        public HeaderSerializer Header { get => _header; set => _header = value; }
        public float Contrast { get => _contrast; set => _contrast = Mathf.Max(value, 0f); }

        public void SetSource(ForwardLookingSonarSensor source)
        {
            _sourceInterface = source;
        }

        public override void Init()
        {
            base.Init();
            _header.Init();

            _numBeams = _sourceInterface.NumBeams;
            _dstHeight = _sourceInterface.NumRangeBins;

            float fovRad = _sourceInterface.FLSFOVDeg * Mathf.Deg2Rad;
            _dstWidth = Mathf.Max(1, Mathf.CeilToInt(2f * _dstHeight * Mathf.Sin(fovRad / 2f)));

            _msg.encoding = "mono8";
            _msg.is_bigendian = 0;
            _msg.width = (uint)_dstWidth;
            _msg.height = (uint)_dstHeight;
            _msg.step = (uint)_dstWidth;
            _msg.data = new byte[_dstWidth * _dstHeight];

            _fanImage = new NativeArray<byte>(_dstWidth * _dstHeight, Allocator.Persistent);

            _warpFanImageJob = new IWarpSonarFanImageJob
            {
                Raw = _sourceInterface.SonarImage,
                SrcWidth = _numBeams,
                SrcHeight = _dstHeight,
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

            // Re-read every cycle rather than relying on the handle captured in Init --
            // same defensive pattern the old point-cloud-based version used.
            _warpFanImageJob.Raw = _sourceInterface.SonarImage;
            _warpFanImageJob.Contrast = _contrast;
            _warpFanImageJob.BorderValue = _backgroundValue;

            _jobHandle = _warpFanImageJob.Schedule(_dstWidth * _dstHeight, 64);
            _jobHandle.Complete();

            _fanImage.CopyTo(_msg.data);

            return _msg;
        }

        public override void OnDestroy()
        {
            _jobHandle.Complete();
            if (_fanImage.IsCreated) _fanImage.Dispose();
        }
    }
}

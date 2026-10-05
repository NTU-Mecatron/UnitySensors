// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace UnitySensors.ROS.Serializer.Sensor
{
    /// <summary>
    /// JPEG-encodes a top-down mono8 buffer (ROS row order). Unity's encoders treat
    /// array row 0 as the bottom of the image, so rows are flipped into a scratch
    /// buffer first; otherwise the published picture comes out upside down.
    /// </summary>
    internal class Mono8JpegEncoder
    {
        private byte[] _flipped;

        public byte[] Encode(byte[] mono8, int width, int height, int quality)
        {
            if (_flipped == null || _flipped.Length != mono8.Length)
                _flipped = new byte[mono8.Length];

            for (int row = 0; row < height; row++)
            {
                System.Buffer.BlockCopy(mono8, row * width, _flipped, (height - 1 - row) * width, width);
            }

            return ImageConversion.EncodeArrayToJPG(_flipped, GraphicsFormat.R8_UNorm, (uint)width, (uint)height, (uint)width, quality);
        }
    }
}

// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [BurstCompile]
    internal struct IWarpFlsFanImageJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> Raw; // SrcWidth x SrcHeight, normalized (not yet 0-255)
        public int SrcWidth;
        public int SrcHeight;
        public float FovRad;
        public float Contrast;
        public byte BorderValue;

        public int DstWidth; // destination height is always SrcHeight (dst_h == h)
        [WriteOnly] public NativeArray<byte> Dst;

        public void Execute(int index)
        {
            int i = index / DstWidth;
            int j = index % DstWidth;

            float dx = j - DstWidth / 2f;
            float dy = SrcHeight - i;

            float radius = math.sqrt(dx * dx + dy * dy);
            float theta = math.atan2(dx, dy);

            float srcCol = (theta / FovRad) * SrcWidth + SrcWidth / 2f;
            float srcRow = radius;

            Dst[index] = SampleBilinear(srcRow, srcCol);
        }

        private byte SampleBilinear(float row, float col)
        {
            int r0 = (int)math.floor(row);
            int c0 = (int)math.floor(col);
            float fr = row - r0;
            float fc = col - c0;

            float v00 = SampleOrBorder(r0, c0);
            float v01 = SampleOrBorder(r0, c0 + 1);
            float v10 = SampleOrBorder(r0 + 1, c0);
            float v11 = SampleOrBorder(r0 + 1, c0 + 1);

            float top = math.lerp(v00, v01, fc);
            float bot = math.lerp(v10, v11, fc);
            return (byte)math.clamp(math.round(math.lerp(top, bot, fr)), 0f, 255f);
        }

        private float SampleOrBorder(int r, int c)
        {
            if (r < 0 || r >= SrcHeight || c < 0 || c >= SrcWidth) return BorderValue;
            // Raw is normalized, not yet a display byte -- the 255 scale-up happens here,
            // alongside Contrast (cv2.convertScaleAbs alpha in the reference driver).
            return math.clamp(Raw[r * SrcWidth + c] * 255f * Contrast, 0f, 255f);
        }
    }
}

// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using System.Collections;
using System.Collections.Generic;

using UnityEngine;

using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using UnitySensors.DataType.Sensor;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Core sonar model: owns the raycast-batch machinery, the raycast-hit buffer and the
    /// beam profile. Concrete modalities provide their (sensor-local) ray geometry via
    /// <see cref="FillLocalDirections"/> and any post-processing via <see cref="OnHitsUpdated"/>.
    ///
    /// Exposes its per-cycle hits as an <see cref="IPointCloudInterface{T}"/> of
    /// <see cref="PointXYZI"/> (sensor-local point + return intensity), so the stock
    /// point-cloud publishers / visualizers work against it unchanged. The cloud is a
    /// fixed <see cref="TotalRayCount"/> in length; missed rays sit at the origin with
    /// zero intensity.
    /// </summary>
    public abstract class SonarSensor : UnitySensor, IPointCloudInterface<PointXYZI>
    {
        [Header("Sonar")]
        [Tooltip("Number of rays cast per beam. Beam = A fan of rays.")]
        public int NumRaysPerBeam = 500;
        [Tooltip("Total opening angle of _each_ beam.")]
        public float BeamBreadthDeg = 90;
        [Tooltip("How many beams(fans) are in this arrangement of sonar.")]
        public int NumBeams = 1;
        [Tooltip("Maximum range of ray cast")]
        public float MaxRange = 100;
        [Tooltip("Reflectivity for any surface without an AcousticSurface component, in [0, 1].")]
        [Range(0f, 1f)]
        public float DefaultReflectivity = 0.5f;
        [Tooltip("Multiplier applied to the [0, 1] return intensity when it is packed into " +
                 "the PointXYZI point cloud. Leave at 1 to keep the physical value; raise it " +
                 "(e.g. 255) if a downstream consumer expects a wider range.")]
        public float PointCloudIntensityScale = 1f;

        // Lowest and highest hit heights, for visualization or other purposes.
        [HideInInspector] public float HitsMinHeight = Mathf.Infinity;
        [HideInInspector] public float HitsMaxHeight = 0f;

        public int TotalRayCount => NumRaysPerBeam * NumBeams;
        public float DegreesPerRayInBeam => BeamBreadthDeg / (NumRaysPerBeam - 1);

        // IPointCloudInterface<PointXYZI>: per-cycle hits as a sensor-local PointXYZI cloud.
        private PointCloud<PointXYZI> _pointCloud;
        public PointCloud<PointXYZI> pointCloud => _pointCloud;
        public int pointsNum => TotalRayCount;

        // Unity job structures for long-term casting of rays.
        private JobHandle _jobHandle;
        private NativeArray<RaycastHit> _raycastHits;
        private NativeArray<RaycastCommand> _raycastCommands;
        private IUpdateRaycastCommandsJob _updateRayCastCommandsJob;
        private IUpdateSonarHitsJob _updateSonarHitsJob;
        private IPackSonarPointCloudJob _packPointCloudJob;

        // Data passed to job system
        private NativeArray<float3> _localPoints;
        private NativeArray<float3> _localDirections;
        private NativeArray<float> _beamProfile;
        private NativeArray<float> _returnIntensities;

        public static (int, int) BeamNumRayNumFromRayIndex(int i, int numRaysPerBeam)
        {
            int rayNum = i % numRaysPerBeam;
            int beamNum = i / numRaysPerBeam;
            return (beamNum, rayNum);
        }

        // Called by UnitySensor.Awake().
        protected override void Init()
        {
            _localDirections = new NativeArray<float3>(TotalRayCount, Allocator.Persistent);
            FillLocalDirections(_localDirections);
            SetupJobs();
            OnInit();
        }

        /// <summary>
        /// Fill <paramref name="directions"/> with this modality's ray directions expressed
        /// in the sensor's LOCAL frame. Called once from <see cref="Init"/>; the pattern is
        /// static for the sensor's lifetime.
        /// </summary>
        protected abstract void FillLocalDirections(NativeArray<float3> directions);

        /// <summary>Per-modality setup, run once after the shared buffers exist.</summary>
        protected virtual void OnInit() { }

        /// <summary>Per-modality post-processing, run each cycle after _rayCastHits is refreshed.</summary>
        protected virtual void OnHitsUpdated() { }

        private void SetupJobs()
        {
            _raycastHits = new NativeArray<RaycastHit>(TotalRayCount, Allocator.Persistent);
            _raycastCommands = new NativeArray<RaycastCommand>(TotalRayCount, Allocator.Persistent);

            _updateRayCastCommandsJob = new IUpdateRaycastCommandsJob
            {
                LocalDirections = _localDirections,
                Origin = transform.position,
                Rotation = transform.rotation,
                MaxRange = MaxRange,
                Commands = _raycastCommands
            };

            _localPoints = new NativeArray<float3>(TotalRayCount, Allocator.Persistent);
            _returnIntensities = new NativeArray<float>(TotalRayCount, Allocator.Persistent);

            _beamProfile = new NativeArray<float>(NumRaysPerBeam, Allocator.Persistent);

            _updateSonarHitsJob = new IUpdateSonarHitsJob
            {
                Results = _raycastHits,
                LocalDirections = _localDirections,
                BeamProfile = _beamProfile,
                DefaultReflectivity = DefaultReflectivity,
                NumRaysPerBeam = NumRaysPerBeam,
                MaxRange = MaxRange,
                WorldToLocalRotation = quaternion.identity,
                LocalPoints = _localPoints,
                ReturnIntensities = _returnIntensities
            };

            _pointCloud = new PointCloud<PointXYZI>
            {
                points = new NativeArray<PointXYZI>(TotalRayCount, Allocator.Persistent)
            };

            _packPointCloudJob = new IPackSonarPointCloudJob
            {
                LocalPoints = _localPoints,
                ReturnIntensities = _returnIntensities,
                IntensityScale = PointCloudIntensityScale,
                Points = _pointCloud.points
            };
        }

        protected override IEnumerator UpdateSensor()
        {
            _updateRayCastCommandsJob.Origin = transform.position;
            _updateRayCastCommandsJob.Rotation = transform.rotation;

            // Pose is fixed for this cycle, so capture the world->local rotation up front and
            // let the whole pipeline run as one dependency chain -- no main-thread sync in
            // the middle for material resolution anymore.
            _updateSonarHitsJob.WorldToLocalRotation = math.inverse((quaternion)transform.rotation);

            // Bind (and seal) the scene reflectivity map on the first tick. Every
            // AcousticSurface has registered from Awake by the time this Start-phase
            // coroutine runs; the map is immutable afterwards, so re-binding is a cheap
            // no-op on later cycles.
            _updateSonarHitsJob.ReflectivityMap = AcousticSurfaceRegistry.GetSealed();

            JobHandle buildRayCastCommandsJobHandle = _updateRayCastCommandsJob.Schedule(TotalRayCount, 10);
            JobHandle raycastJobHandle = RaycastCommand.ScheduleBatch(_raycastCommands, _raycastHits, 20, buildRayCastCommandsJobHandle);
            JobHandle updateSonarHitsJob = _updateSonarHitsJob.Schedule(TotalRayCount, 20, raycastJobHandle);
            _jobHandle = _packPointCloudJob.Schedule(TotalRayCount, 64, updateSonarHitsJob);
            _jobHandle.Complete();

            yield return null;
        }

        protected override void OnSensorDestroy()
        {
            if (_raycastHits.IsCreated)
            {
                _jobHandle.Complete();
                _raycastHits.Dispose();
            }
            if (_raycastCommands.IsCreated) _raycastCommands.Dispose();
            if (_localDirections.IsCreated) _localDirections.Dispose();
            if (_beamProfile.IsCreated) _beamProfile.Dispose();
            if (_localPoints.IsCreated) _localPoints.Dispose();
            if (_returnIntensities.IsCreated) _returnIntensities.Dispose();
            if (_pointCloud != null && _pointCloud.points.IsCreated) _pointCloud.Dispose();
        }
    }
}

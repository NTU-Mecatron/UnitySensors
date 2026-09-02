// Copyright (c) 2026 [Bui Gia Bao]. All rights reserved.
// UnityMDS: A High-Fidelity Multi-Drone, Multi-Domain Simulator for Maritime Robotics.
// Licensed under the Mozilla Public License 2.0 (MPL-2.0).
// See the LICENSE file in the repository root for full boundary and usage terms.

using UnityEngine;

using Unity.Collections;

namespace UnitySensors.Sensor.Sonar
{
    /// <summary>
    /// Tags a collider with a scalar acoustic reflectivity in [0, 1]. Registered into
    /// <see cref="AcousticSurfaceRegistry"/> from Awake so sonar jobs can resolve it
    /// Burst-side from <c>RaycastHit.colliderInstanceID</c>. Colliders without this
    /// component fall back to the sensor's <c>DefaultReflectivity</c>.
    /// </summary>
    [RequireComponent(typeof(Collider))]
    [AddComponentMenu("MDS/Sensor/Sonar/Acoustic Surface")]
    public class AcousticSurface : MonoBehaviour
    {
        [SerializeField, Range(0f, 1f)]
        [Tooltip("Fraction of incident acoustic energy reflected back to the sonar. " +
                 "0 = anechoic, 1 = perfect reflector.")]
        private float _reflectivity = 0.5f;

        public float Reflectivity => _reflectivity;

        private void Awake()
        {
            AcousticSurfaceRegistry.Register(GetComponent<Collider>().GetInstanceID(), _reflectivity);
        }
    }

    /// <summary>
    /// Scene-wide, build-once map of collider instance ID -> acoustic reflectivity, in
    /// native memory so sonar jobs can read it Burst-side without touching managed
    /// Colliders.
    ///
    /// Assumes a static world: every <see cref="AcousticSurface"/> present at scene load
    /// registers from Awake, before any sonar coroutine first ticks in Start. The first
    /// sonar to wire its job calls <see cref="GetSealed"/>, after which the map is
    /// immutable and registrations from additively-loaded or spawned objects are ignored
    /// (their hits use the sensor's DefaultReflectivity). Sealing is what lets every sonar
    /// job hold the map [ReadOnly] with no write ever racing a running job -- so no
    /// deferred-write queue / flush step is needed.
    /// </summary>
    public static class AcousticSurfaceRegistry
    {
        private const int InitialCapacity = 4096;

        private static NativeHashMap<int, float> _map;
        private static bool _sealed;

        /// <summary>
        /// Records a collider's reflectivity. No-op once the map has been sealed; call only
        /// from Awake (scene-load phase).
        /// </summary>
        public static void Register(int colliderInstanceId, float reflectivity)
        {
            EnsureCreated();

            if (_sealed)
            {
                Debug.LogWarning(
                    $"[AcousticSurfaceRegistry] Collider {colliderInstanceId} registered after the " +
                    "map was sealed; its sonar returns will use the sensor's DefaultReflectivity. " +
                    "AcousticSurface components must be present at scene load.");
                return;
            }

            _map[colliderInstanceId] = reflectivity;
        }

        /// <summary>
        /// Returns the map and marks it immutable. Idempotent -- the first call seals it.
        /// Call from a sonar's first update tick (Start phase), never from Awake, so every
        /// AcousticSurface has already registered.
        /// </summary>
        public static NativeHashMap<int, float> GetSealed()
        {
            EnsureCreated();
            _sealed = true;
            return _map;
        }

        public static void Dispose()
        {
            if (_map.IsCreated) _map.Dispose();
            _sealed = false;

            Application.quitting -= Dispose;
#if UNITY_EDITOR
            UnityEditor.AssemblyReloadEvents.beforeAssemblyReload -= Dispose;
            UnityEditor.EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
#endif
        }

        private static void EnsureCreated()
        {
            if (_map.IsCreated) return;

            _map = new NativeHashMap<int, float>(InitialCapacity, Allocator.Persistent);
            _sealed = false;

            // The map is a static NativeContainer that outlives every sensor, so its
            // disposal can't hang off any one sensor's teardown.
            Application.quitting -= Dispose;
            Application.quitting += Dispose;
#if UNITY_EDITOR
            // Application.quitting does not fire on play-mode exit in the editor, and a
            // domain reload drops the managed handle without freeing the native memory --
            // which is exactly what the "Leak Detected : Persistent" check reports. Hook
            // both editor lifecycle points so the map is always released.
            UnityEditor.AssemblyReloadEvents.beforeAssemblyReload -= Dispose;
            UnityEditor.AssemblyReloadEvents.beforeAssemblyReload += Dispose;
            UnityEditor.EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
            UnityEditor.EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
#endif
        }

        // Domain-reload-disabled safety: release any native memory left over from a prior
        // play session before the next one starts registering.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetOnPlay() => Dispose();

#if UNITY_EDITOR
        private static void OnPlayModeStateChanged(UnityEditor.PlayModeStateChange change)
        {
            if (change == UnityEditor.PlayModeStateChange.ExitingPlayMode) Dispose();
        }
#endif
    }
}

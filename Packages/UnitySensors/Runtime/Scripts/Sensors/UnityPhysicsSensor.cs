using UnityEngine;
using UnitySensors.Interface.Std;
using System.Runtime.CompilerServices;
using System;
using System.Collections;
using UnityEngine.Rendering;

[assembly: InternalsVisibleTo("UnitySensorsEditor")]
[assembly: InternalsVisibleTo("UnitySensorsROSEditor")]
namespace UnitySensors.Sensor
{
    public abstract class UnityPhysicsSensor : MonoBehaviour, ITimeInterface
    {
        [SerializeField, Min(0)]
        internal float _frequency = 10.0f;
        private float _time;
        private float _period;
        private bool _hasNewData = false;
        private float _timeSinceLastUpdate = 0f;

        public Action onSensorUpdateComplete;
        public float dt { get => _period; }
        public float time { get => _time; }
        public float frequency
        {
            get => _frequency;
            set
            {
                _frequency = Mathf.Max(value, 0);
                _period = 1.0f / _frequency;
            }
        }

        private void Awake()
        {
            _period = 1.0f / _frequency;

            if (_period < Time.fixedDeltaTime)
            {
                Debug.LogWarning($"[{transform.name}] Sensor update frequency set to {frequency}Hz but Unity updates physics at {1f / Time.fixedDeltaTime}Hz. Setting sensor period to Unity's fixedDeltaTime!");
                _period = Time.fixedDeltaTime;
            }

            Init();
        }

        void FixedUpdate()
        {
            _timeSinceLastUpdate += Time.fixedDeltaTime;
            if (_timeSinceLastUpdate < _period) return;
            _hasNewData = UpdateSensor(_timeSinceLastUpdate);
            _timeSinceLastUpdate = 0f;
        }

        public bool HasNewData()
        {
            return _hasNewData;
        }

        public virtual bool UpdateSensor(double deltaTime)
        {
            Debug.Log("This sensor needs to override UpdateSensor!");
            return false;
        }

        private void OnDestroy()
        {
            AsyncGPUReadback.WaitAllRequests();
            OnSensorDestroy();
        }


        protected abstract void Init();
        protected abstract void OnSensorDestroy();
    }
}

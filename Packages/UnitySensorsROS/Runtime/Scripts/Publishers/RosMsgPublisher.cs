using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnitySensors.ROS.Utils.Namespacing;
using UnitySensors.ROS.Serializer;

namespace UnitySensors.ROS.Publisher
{
    public abstract class RosMsgPublisher : MonoBehaviour
    {
        public abstract float Frequency { get; set; }
        public abstract string TopicName { get; set; }
    }

    public class RosMsgPublisher<T, TT> : RosMsgPublisher where T : RosMsgSerializer<TT> where TT : Message, new()
    {
        [SerializeField, Min(0)]
        protected float _frequency = 10.0f;

        [SerializeField]
        protected string _topicName;

        [SerializeField]
        protected T _serializer;
        private static int _publisher_count = 0;

        private ROSConnection _ros;
        private float _dt;
        private float _frequency_inv;
        private int _publisher_id;

        public T Serializer { get => _serializer; }
        public override string TopicName { get => _topicName; set => _topicName = value; }
        public override float Frequency
        {
            get => _frequency;
            set
            {
                _frequency = Mathf.Max(value, 0);
                _frequency_inv = 1.0f / _frequency;
                InitializePublisherOffset();
            }
        }
        private void Awake()
        {
            InitializePublisher();
        }

        protected virtual void InitializePublisher()
        {
            _frequency_inv = 1.0f / _frequency;

            _publisher_id = _publisher_count;
            _publisher_count++;

            InitializePublisherOffset();
        }

        private void InitializePublisherOffset()
        {
            string publisherType = GetType().Name;
            int typeHash = publisherType.GetHashCode();

            // Combine publisher ID and type to create a more dispersed value
            // Use coprime numbers and operations to increase dispersion
            float seed = (_publisher_id * 16777619 + typeHash) * 0.618033988749895f;

            // Ensure the offset is in [0, 1)
            float normalizedOffset = seed % 1.0f;
            if (normalizedOffset < 0) normalizedOffset += 1.0f; // Ensure non-negative

            _dt = normalizedOffset * _frequency_inv;

            // Debug.Log($"Publisher {GetType().Name} ID:{_publisher_id} initialized with offset {normalizedOffset:F3} ({_dt:F3}s)");
        }

        protected virtual void Start()
        {
            _ros = ROSConnection.GetOrCreateInstance();
            _serializer.Init();
            _topicName = NamespaceUtils.GetResolvedTopicName(_topicName, gameObject);
            _ros.RegisterPublisher<TT>(_topicName);
        }

        // TODO: Use Coroutine for async publishing
        protected virtual void FixedUpdate()
        {
            _dt += Time.fixedDeltaTime;
            if (_dt < _frequency_inv) return;
            _ros.Publish(_topicName, _serializer.Serialize());
            _dt -= _frequency_inv;
        }

        private void OnDestroy()
        {
            _serializer.OnDestroy();
        }
    }
}

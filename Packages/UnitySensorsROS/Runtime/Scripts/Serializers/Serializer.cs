using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace UnitySensors.ROS.Serializer
{
    [System.Serializable]
    public abstract class RosMsgSerializer<T> where T : Message, new()
    {
        [SerializeField]
        protected T _msg;
        public T msg { get => _msg; }

        public virtual void Init() { _msg = new T(); }
        // TODO: Use Coroutine for async serialization
        public abstract T Serialize();
        public virtual void OnDestroy() { }
    }
}

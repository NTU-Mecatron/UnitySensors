using RosMessageTypes.Diagnostic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnitySensors.ROS.Utils.Namespacing;

namespace UnitySensors.Sensor.Diagnostics
{
    /// <summary>
    /// This class is used by other DiagnosticUpdater to publish their diagnostic_array msg.
    /// The impl is very different from other publishers because:
    /// 1. Compared to TF2 publisher, it is not aggregating individual array msgs into a longer array; each Updater needs to publish its own array.
    /// 2. Compared to other sensor publishers, we want only one diagnostic publisher to exist. Hence, other Updater can call PublishDiagnosticArrayMsg exactly when they need.
    /// While not following pattern, it allows current developer to make it happen much faster.
    /// Note: This class should be added to the base game object (the vehicle itself); each vehicle should only have one of this.
    /// </summary>
    public class DiagnosticPublisher : MonoBehaviour
    {
        ROSConnection _ros;

        /// <summary>
        /// This is a fixed topic name. It will be namespaced to the vehicle name if available.
        /// </summary>
        string _topicName = "diagnostics";

        void Start()
        {
            _topicName = NamespaceUtils.GetResolvedTopicName(_topicName, this.gameObject);

            _ros = ROSConnection.GetOrCreateInstance();
            _ros.RegisterPublisher<DiagnosticArrayMsg>(_topicName);
        }

        public void PublishDiagnosticArrayMsg(DiagnosticArrayMsg msg)
        {
            _ros.Publish(_topicName, msg);
        }
    }

}

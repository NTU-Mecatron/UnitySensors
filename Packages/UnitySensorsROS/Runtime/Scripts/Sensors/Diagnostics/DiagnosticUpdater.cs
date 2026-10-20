using NUnit.Framework;
using RosMessageTypes.Diagnostic;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.Sensor.Diagnostics
{
    public enum DiagnosticLevel
    {
        OK = 0,
        WARN = 1,
        ERROR = 2,
        STALE = 3
    }

    /// <summary>
    /// Each diagnostic contains a scenario that could happen for a diagnostic name.
    /// Eg. for a sensor temperature, a diagnostic can be either OK, too high or too low.
    /// </summary>
    [System.Serializable]
    public struct Diagnostic
    {
        [Tooltip("Current status.")]
        public DiagnosticLevel level;

        [Tooltip("A description of the status. Eg. 'DVL frequency is too low' or 'INS temperature is too high'.")]
        public string message;

        [Tooltip("Whether to include this diagnostic.")]
        public bool activate;
    }

    /// <summary>
    /// A diagnostic status contains multiple probable diagnostics for a specific test/component. Each diagnostic can be activated or deactivated based on the 'activate' field.
    /// Only one diagnostic can be active at a time for a specific diagnostic status (eg. a sensor cannot be both too hot and too cold).
    /// </summary>
    [System.Serializable]
    public struct DiagnosticStatus
    {
        [Tooltip("A description of the technical aspect of the component reporting (eg. temperature, data_valid, frequency)." +
            "\n\nNote: do not prepend the sensor name; this is done by the Status Name Prefix. ")]
        public string name;

        [Tooltip("In the Editor, each status contains multiple probable diagnostics; each diagnostic contains a scenario that could happen for a status. " +
            "\nOnly one diagnostic can be active at a time for a specific diagnostic status (eg. a sensor cannot be both too hot and too cold).")]
        public List<Diagnostic> possibleDiagnostics;
    }

    /// <summary>
    /// This class interacts heavily with the Editor, updating DiagnosticArrayMsg for a particular sensor based on what is set in the inspector.
    /// Each sensor or actuator should have this component to report Diagnostics.
    /// </summary>
    [System.Serializable]
    public class DiagnosticUpdater : UnitySensor
    {
        [Tooltip("Hardware ID for all the diagnostic statuses in the diagnostic array.")]
        [SerializeField] string _statusHardwareId;

        [Tooltip("This will be prepended to all the status names in the diag statuses. Should be device name.")]
        [SerializeField] string _statusNamePrefix;

        [Tooltip("Header for the diagnostic array message.")]
        [SerializeField] HeaderSerializer _header;

        [Tooltip("List of diagnostic statuses to publish by a hardware id. " +
            "Each status is about a different aspect of the same hardware (temperature, frequency, data accuracy, etc.)." +
            "\n\nThe list of statuses and diagnostics should be set once before play; during runtime, only activate/deactivate is needed. " +
            "Changing the list during runtime will not take effect.")]
        [SerializeField] List<DiagnosticStatus> _diagStatuses;

        DiagnosticArrayMsg _diagArrayMsg = new();
        List<DiagnosticStatusMsg> _diagStatusMsgs = new();

        /// <summary>
        /// The central publisher per vehicle.
        /// </summary>
        DiagnosticPublisher _pub;    

        /// <summary>
        /// Loop through the diagnostic statuses and update the diagnostic array message based on the activated diagnostic for each status.
        /// If a status has two active diagnostics, the first one that is looped through will be published. 
        /// Nevertheless, only one can be active at a time, enforced through editor script (not in the scope of this script).
        /// </summary>
        protected override IEnumerator UpdateSensor() 
        {
            foreach (var diagStatus in _diagStatuses)
            {
                Diagnostic activeDiagnostic = default;
                bool foundActive = false;

                foreach (var diagnostic in diagStatus.possibleDiagnostics)
                {
                    if (diagnostic.activate)
                    {
                        activeDiagnostic = diagnostic;
                        foundActive = true;
                        break;
                    }
                }

                if (foundActive)
                {
                    var statusMsg = new DiagnosticStatusMsg(
                        level: (sbyte)activeDiagnostic.level,
                        name: _statusNamePrefix + ": " + diagStatus.name,
                        message: activeDiagnostic.message,
                        hardware_id: _statusHardwareId,
                        values: new KeyValueMsg[0]
                    );

                    _diagStatusMsgs.Add(statusMsg);
                }
            }

            _diagArrayMsg.header = _header.Serialize();
            _diagArrayMsg.status = _diagStatusMsgs.ToArray();
            _pub.PublishDiagnosticArrayMsg(_diagArrayMsg);

            _diagStatusMsgs.Clear();
            yield return null;
        }

        protected override void Init()
        {
            _pub = GetComponentInParent<DiagnosticPublisher>();
            _header.Init();
            Assert.NotNull(_pub, "The root game object (the vehicle) must have one DiagnosticPublisher." );
            Assert.NotZero(_diagStatuses.Count, "The number of diagnostic statuses must be greater than zero.");
            Assert.IsNotNull(_header.FrameId, "The FrameId of the header on Diagnostic Updater must be set.");
        }

        protected override void OnSensorDestroy() { }

        private void Reset()
        {
            Frequency = 1.0f;
        }
    }

}

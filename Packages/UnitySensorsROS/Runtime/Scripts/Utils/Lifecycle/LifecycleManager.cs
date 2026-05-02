using UnityEngine;
using System.Collections.Generic;
using RosMessageTypes.Lifecycle;
using Unity.Robotics.ROSTCPConnector;
using UnitySensors.ROS.Utils.Namespacing;

namespace UnitySensors.ROS.Utils.Lifecycle
{
    public enum LifecycleState
    {
        Unconfigured,
        Inactive,
        Active,
        Finalized,
        Configuring,
        CleaningUp,
        ShuttingDown,
        Activating,
        Deactivating,
        ErrorProcessing
    }

    public class LifecycleManager : MonoBehaviour
    {
        ROSConnection _ros;

        [Tooltip("Name of the lifecycle node. This will be used as the namespace for the lifecycle services. Cannot be null.")]
        [SerializeField] string _nodeName;
        [Tooltip("List of components whose enabled state will be managed based on the lifecycle state. Assign components that should be active when the lifecycle state is Active, and inactive otherwise.")]
        [SerializeField] List<Behaviour> _referenceComponents;
        [Tooltip("Initial lifecycle state of the node. May change during runtime due to service call.")]
        [SerializeField] LifecycleState _currentLifecycleState = LifecycleState.Inactive;

        public string NodeName { get => _nodeName; set => _nodeName = value; }
        public LifecycleState CurrentLifecycleState { get => _currentLifecycleState; set => _currentLifecycleState = value; }

        void Start()
        {
            Debug.Assert(!string.IsNullOrEmpty(_nodeName), $"Node name for LifecycleManager on {this.gameObject.name} is not set. Please assign a node name in the inspector.");
            _nodeName = NamespaceUtils.GetResolvedTopicName(_nodeName, this.gameObject);

            _ros = ROSConnection.GetOrCreateInstance();
            _ros.ImplementService<ChangeStateRequest, ChangeStateResponse>($"{_nodeName}/change_state", ChangeStateCb);
            _ros.ImplementService<GetStateRequest, GetStateResponse>($"{_nodeName}/get_state", GetStateCb);

            Debug.Assert(_referenceComponents != null && _referenceComponents.Count > 0,
                $"Reference components list on {this.gameObject.name} is empty. Please assign at least one component to reference in the inspector.");

            // Set initial state of components based on the current lifecycle state
            bool activate = _currentLifecycleState == LifecycleState.Active;
            ChangeStateForComponents(activate);
        }

        ChangeStateResponse ChangeStateCb(ChangeStateRequest req)
        {
            Debug.Log($"Received request to change state to {req.transition.id}");

            byte id = req.transition.id;
            bool configure = id == TransitionMsg.TRANSITION_CONFIGURE && _currentLifecycleState == LifecycleState.Unconfigured;
            bool cleanup = id == TransitionMsg.TRANSITION_CLEANUP && _currentLifecycleState == LifecycleState.Inactive;
            bool activate = id == TransitionMsg.TRANSITION_ACTIVATE && _currentLifecycleState == LifecycleState.Inactive;
            bool deactivate = id == TransitionMsg.TRANSITION_DEACTIVATE && _currentLifecycleState == LifecycleState.Active;
            bool success = configure || cleanup || activate || deactivate;

            if (configure)
            {
                ChangeStateForComponents(false);
                _currentLifecycleState = LifecycleState.Inactive;
                this.gameObject.SetActive(true);
            }
            else if (cleanup)
            {
                ChangeStateForComponents(false);
                _currentLifecycleState = LifecycleState.Unconfigured;
                this.gameObject.SetActive(false);
            }          
            else if (activate)
            {
                ChangeStateForComponents(true);
                _currentLifecycleState = LifecycleState.Active;
            }
            else if (deactivate)
            {
                ChangeStateForComponents(false);
                _currentLifecycleState = LifecycleState.Inactive;
            }      

            return new ChangeStateResponse() { success = success };
        }

        GetStateResponse GetStateCb(GetStateRequest req)
        {
            Debug.Log("Received request to get current state");
            byte id;
            string label;
            switch (_currentLifecycleState)
            {
                case LifecycleState.Unconfigured:
                    id = StateMsg.PRIMARY_STATE_UNCONFIGURED;
                    label = "unconfigured";
                    break;
                case LifecycleState.Inactive:
                    id = StateMsg.PRIMARY_STATE_INACTIVE;
                    label = "inactive";
                    break;
                case LifecycleState.Active:
                    id = StateMsg.PRIMARY_STATE_ACTIVE;
                    label = "active";
                    break;
                case LifecycleState.Finalized:
                    id = StateMsg.PRIMARY_STATE_FINALIZED;
                    label = "finalized";
                    break;
                case LifecycleState.Configuring:
                    id = StateMsg.TRANSITION_STATE_CONFIGURING;
                    label = "configuring";
                    break;
                case LifecycleState.CleaningUp:
                    id = StateMsg.TRANSITION_STATE_CLEANINGUP;
                    label = "cleaning_up";
                    break;
                case LifecycleState.ShuttingDown:
                    id = StateMsg.TRANSITION_STATE_SHUTTINGDOWN;
                    label = "shutting_down";
                    break;
                case LifecycleState.Activating:
                    id = StateMsg.TRANSITION_STATE_ACTIVATING;
                    label = "activating";
                    break;
                case LifecycleState.Deactivating:
                    id = StateMsg.TRANSITION_STATE_DEACTIVATING;
                    label = "deactivating";
                    break;
                case LifecycleState.ErrorProcessing:
                    id = StateMsg.TRANSITION_STATE_ERRORPROCESSING;
                    label = "error_processing";
                    break;
                default:
                    id = StateMsg.PRIMARY_STATE_UNKNOWN;
                    label = "unknown";
                    break;
            }
            
            return new GetStateResponse()
            {
                current_state = new StateMsg()
                {
                    id = id,
                    label = label
                }
            };
        }

        void ChangeStateForComponents(bool activate)
        {
            foreach (var comp in _referenceComponents)
            {
                if (comp != null)
                    comp.enabled = activate;
            }
        }
    }
}

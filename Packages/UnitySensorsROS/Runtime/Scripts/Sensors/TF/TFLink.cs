using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace UnitySensors.Sensor.TF
{
    [System.Serializable]
    public struct TFData
    {
        public string frame_id_parent;
        public string frame_id_child;
        public Vector3 position;
        public Quaternion rotation;
        public CoordinateSpaceSelection position_frame_convention;
        public CoordinateSpaceSelection rotation_frame_convention;
    };

    /// <summary>
    /// A class that acts as both a TF Link and a source of TF data for the TfMessageMsgSerializer.
    /// As a TF Link, it holds information about its frame id, transform, and child links.
    /// When TfMessagePublisher is present and uses this TFLink as its source, you can dictate whether it should recursively gather TF data from child links.
    /// This functionality is useful for global TF publishers where you only want to publish the base_link and exclude all child links.
    /// </summary>
    public class TFLink : UnitySensor
    {

        [SerializeField, Tooltip("Frame Id of this game object. Could be world, map, base_link, sensor_link." +
            "\n\nDo not prepend the name of the robot to the link (aka do not write robot/base_link).")]
        string _frame_id;

        [SerializeField, Tooltip("List of child links. This list will be auto-populated during runtime if not assigned in inspector. " +
            "Or it will use the items that you have assigned and do not auto-search for child links.")]
        TFLink[] _children;

        //// Only appear in inspector if TfMessageMsgPublisher is present
        //[SerializeField, Tooltip("If true, when this TFLink is used as a source for TfMessagePublisher, " +
        //    "it will recursively gather TF data from its children. " +
        //    "If false, it will only provide its own TF data without considering children." +
        //    "\n\nIf this is true, _useNamespacedChildIds should be false. And vice versa.")]
        //bool _recurseFindChildLinks = true;
        //// Only appear in inspector if TfMessageMsgPublisher is present
        //[SerializeField, Tooltip("If true, when this TFLink is used as a source for TfMessagePublisher, " +
        //    "it will prepend the name of the robot to the frame ids. " +
        //    "If false, it will use the frame ids as is.")]
        //bool _useNamespacedChildIds = false;

        [Tooltip("Cached transform of this TFLink.")]
        Transform _transform;

        [Tooltip("Cache the name of the gameObject containing base_link.")]
        string _base_link_prefix = "";

        public string FrameId { get { return _frame_id; } set { _frame_id = value; } }

        protected override void Init()
        {
            _transform = this.transform;
            _frame_id = _frame_id.ToLower().Replace(" ", "_");

            // Recursively look for parent TFLink with frame id "base_link" to cache
            string baseLinkName = FindBaseLinkGameObjectName(_transform);
            if (!string.IsNullOrEmpty(baseLinkName))
            {
                _base_link_prefix = baseLinkName + "/"; // End with slash (eg. robot/)
            }

            // Automatically find all direct children TFLink components if null
            if (_children == null || _children.Length == 0)
            {
                List<TFLink> children = new ();
                FindDirectChildrenTFLinks(transform, children);
                _children = children.ToArray();
            }        

            // Remove null or inactive children
            _children = _children.Where(child => child != null && child.gameObject.activeInHierarchy).ToArray();
        }

        bool IsBaseLink(string frameId) => frameId.Contains("base_link");

        /// <summary>
        /// Recursively searches up the parent hierarchy for a TFLink with frame_id "base_link"
        /// and returns the name of its GameObject, or empty string if not found.
        /// </summary>
        string FindBaseLinkGameObjectName(Transform current)
        {
            if (current == null)
                return "";

            if (current.TryGetComponent<TFLink>(out var tfLink))
            {
                if (IsBaseLink(tfLink.FrameId))
                {
                    return current.gameObject.name.ToLower().Replace(" ", "_");
                }
            }

            return FindBaseLinkGameObjectName(current.parent);
        }

        /// <summary>
        /// Recursive method to find direct children TFLink components and append it to a list
        /// </summary>
        void FindDirectChildrenTFLinks(Transform parent, List<TFLink> children)
        {
            foreach (Transform child in parent)
            {
                if (child.TryGetComponent<TFLink>(out var childTFLink))
                {
                    // If a TFLink is found, add it to the list and stop searching further into this child
                    children.Add(childTFLink);
                }
                else
                {
                    // If no TFLink is found, continue searching recursively into the child's children
                    FindDirectChildrenTFLinks(child, children);
                }
            }
        }

        /// <summary>
        /// Public method for the TfMessageMsgSerializer to get all TF data from this link and optionally its children.
        /// Only the top level TFLink call this. Whatever code inside this method is only run once, not recursively.
        /// </summary>
        public TFData[] GetTFData(bool recurseFindChildLinks, bool useBaseLinkNameAsPrefix, string suffix)
        {
            List<TFData> tfData = new();

            // Warn if map/world frame is not at origin with no rotation
            if (_frame_id == "map" || _frame_id == "world")
            {
                if (_transform.position != Vector3.zero || _transform.rotation != Quaternion.identity)
                {
                    Debug.LogWarning($"TFLink: The '{_frame_id}' frame is expected to be at the origin with no rotation. " +
                        $"However, the current position is {_transform.position} and rotation is {_transform.rotation.eulerAngles}. " +
                        $"Please ensure that the '{_frame_id}' frame is correctly set up at the world origin.");
                }
            }

            // If this method is called on the base_link (aka the TF publisher is on this link), add map->odom and odom->base_link static frames
            if (IsBaseLink(_frame_id))
            {
                AddStaticFrames(tfData, useBaseLinkNameAsPrefix, suffix);
            }

            // Correctly set the frame id for this link
            string prefix = (useBaseLinkNameAsPrefix) ? _base_link_prefix : "";
            string frame_id = (_frame_id == "map" || _frame_id == "world") ? _frame_id : prefix + _frame_id + suffix;

            // Get TF data from all children
            foreach (TFLink child in _children)
            {
                tfData.AddRange(child.GetTFData(frame_id, _transform, recurseFindChildLinks, useBaseLinkNameAsPrefix, suffix));
            }

            return tfData.ToArray();
        }

        /// <summary>
        /// Method usually called by the child link. Hence must be public method.
        /// </summary>
        public TFData[] GetTFData(string parentFrameId, Transform parentTransform, bool recurseFindChildLinks, bool useBaseLinkNameAsPrefix, string suffix)
        {
            List<TFData> tfData = new();

            // Get this link's TF data relative to the parent
            string prefix = (useBaseLinkNameAsPrefix) ? _base_link_prefix : "";
            string frame_id = prefix + _frame_id + suffix;
            Vector3 relativePos = parentTransform.InverseTransformPoint(_transform.position);
            Quaternion relativeRot = Quaternion.Inverse(parentTransform.rotation) * _transform.rotation;

            // Use ENU for map/world/odom frames, FLU for body frames and save this setting for the TFMessageMsgSerializer to perform the actual conversion
            CoordinateSpaceSelection positionFrame = (parentFrameId == "map" || parentFrameId == "world" || parentFrameId == "odom") 
                                                ? CoordinateSpaceSelection.ENU : CoordinateSpaceSelection.FLU;
            // ENU frame will introduct a 90deg offset (if Z-axis is North) while FLU keep things the same
            CoordinateSpaceSelection rotationFrame = (IsBaseLink(_frame_id)) 
                                                ? CoordinateSpaceSelection.ENU : CoordinateSpaceSelection.FLU;

            // Add this link's TF data
            TFData tfData_self = new ()
            {
                frame_id_parent = parentFrameId,
                frame_id_child = frame_id,
                position = relativePos,
                rotation = relativeRot,
                position_frame_convention = positionFrame,
                rotation_frame_convention = rotationFrame
            };
            tfData.Add(tfData_self);

            // Recursively get TF data from children if specified, using only the top-level parent setting
            if (recurseFindChildLinks)
            {
                foreach (TFLink child in _children)
                {
                    tfData.AddRange(child.GetTFData(frame_id, this._transform, recurseFindChildLinks, useBaseLinkNameAsPrefix, suffix));
                }
            }
            
            return tfData.ToArray();
        }

        /// <summary>
        /// Method to add static frames: map->odom and odom->base_link
        /// </summary>
        void AddStaticFrames(List<TFData> tfData, bool useBaseLinkNameAsPrefix, string suffix)
        {
            if (!IsBaseLink(_frame_id))
            {
                Debug.LogWarning("TFLink: AddStaticFrames called on a non-base_link TFLink. This should not happen.");
                return;
            }

            string prefix = (useBaseLinkNameAsPrefix) ? _base_link_prefix : "";
            string odomFrameId = prefix + "odom" + suffix;
            string baseLinkFrameId = prefix + _frame_id + suffix;

            TFData mapToOdom = new()
            {
                frame_id_parent = "map",
                frame_id_child = odomFrameId,
                position = Vector3.zero,
                rotation = Quaternion.identity,
                position_frame_convention = CoordinateSpaceSelection.ENU,
                rotation_frame_convention = CoordinateSpaceSelection.FLU
            };
            tfData.Add(mapToOdom);

            TFData odomToBaseLink = new()
            {
                frame_id_parent = odomFrameId,
                frame_id_child = baseLinkFrameId,
                position = _transform.position,
                rotation = _transform.rotation,
                position_frame_convention = CoordinateSpaceSelection.ENU,
                rotation_frame_convention = CoordinateSpaceSelection.ENU
            };
            tfData.Add(odomToBaseLink);
        }

        protected override IEnumerator UpdateSensor() { yield return null; }
        protected override void OnSensorDestroy() {}
    }
}
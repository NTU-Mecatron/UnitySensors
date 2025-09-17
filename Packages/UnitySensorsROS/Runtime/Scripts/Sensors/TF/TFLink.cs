using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnitySensors.ROS.Publisher.Tf2;

namespace UnitySensors.Sensor.TF
{
    [System.Serializable]
    public struct TFData
    {
        public string frame_id_parent;
        public string frame_id_child;
        public Vector3 position;
        public Quaternion rotation;
    };

    public class TFLink : UnitySensor
    {
        [SerializeField]
        private string _frame_id;
        [SerializeField]
        private bool _isTFRoot = false;
        [SerializeField]
        private TFLink[] _children;

        private Transform _transform;

        private readonly List<string> _map_frame_ids = new List<string> { "map", "world" };

        public string FrameId { get => _frame_id; }
        public bool IsTFRoot { get => _isTFRoot; }
        private bool IsMapTF() { return GetComponent<TFMessageMsgPublisher>() != null; }

        protected override void Init()
        {
            _transform = this.transform;

            // Prepend the TFRoot parent's name to the _frame_id
            if (!IsMapTF())
            {
                Transform parent = _transform;
                while (parent != null)
                {
                    TFLink parent_tf = parent.GetComponent<TFLink>();
                    if (parent_tf != null && parent_tf.IsTFRoot)
                    {
                        // Prepend the TFRoot parent name to the _frame_id
                        _frame_id = $"{parent.name.Replace(" ", "_").Replace("-", "_")}/{_frame_id}";
                        break;
                    }
                    parent = parent.parent;
                }
            }
            
        }

        private void Reset()
        {
            // Set _isTFRoot to true if the nearest parent with TFLink is a map frame
            _isTFRoot = transform.parent == null || _map_frame_ids.Contains(transform.parent.GetComponentInParent<TFLink>().FrameId); 

            if (!IsMapTF())
            {
                if (_isTFRoot)
                {
                    _frame_id = "base_link";
                }
                else
                {
                    // Automatically generate the frame_id based on the GameObject's name
                    _frame_id = $"{gameObject.name.ToLower()}_link";
                }
            }
            else _frame_id = "map";

            // Automatically find all direct children TFLink components
            List<TFLink> children = new List<TFLink>();
            FindDirectChildrenTFLinks(transform, children);
            _children = children.ToArray();
        }

        // Recursive method to find direct children TFLink components
        private void FindDirectChildrenTFLinks(Transform parent, List<TFLink> children)
        {
            foreach (Transform child in parent)
            {
                TFLink childTFLink = child.GetComponent<TFLink>();
                if (childTFLink != null)
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


        protected override IEnumerator UpdateSensor()
        {
            yield return null;
        }

        public TFData[] GetTFData()
        {
            List<TFData> tfData = new List<TFData>();

            Matrix4x4 worldToLocalMatrix = _transform.worldToLocalMatrix;
            Quaternion worldToLocalQuaternion = Quaternion.Inverse(_transform.rotation);

            foreach (TFLink child in _children)
            {
                tfData.AddRange(child.GetTFData(_frame_id, worldToLocalMatrix, worldToLocalQuaternion));
            }

            return tfData.ToArray();
        }

        public TFData[] GetTFData(string frame_id_parent, Matrix4x4 worldToLocalMatrix, Quaternion worldToLocalQuaternion)
        {
            List<TFData> tfData = new List<TFData>();

            TFData tfData_self;
            tfData_self.frame_id_parent = frame_id_parent;
            tfData_self.frame_id_child = _frame_id;
            tfData_self.position = (Vector3)(worldToLocalMatrix * new Vector4(_transform.position.x, _transform.position.y, _transform.position.z, 1.0f));
            Vector3 localScale = _transform.localScale;
            Vector3 lossyScale = _transform.lossyScale;
            Vector3 scaleVector = new Vector3()
            {
                x = localScale.x != 0 ? lossyScale.x / localScale.x : 0,
                y = localScale.y != 0 ? lossyScale.y / localScale.y : 0,
                z = localScale.z != 0 ? lossyScale.z / localScale.z : 0
            };
            tfData_self.position.Scale(scaleVector);
            tfData_self.rotation = worldToLocalQuaternion * _transform.rotation;
            tfData.Add(tfData_self);

            worldToLocalMatrix = _transform.worldToLocalMatrix;
            worldToLocalQuaternion = Quaternion.Inverse(_transform.rotation);

            foreach (TFLink child in _children)
            {
                tfData.AddRange(child.GetTFData(_frame_id, worldToLocalMatrix, worldToLocalQuaternion));
            }

            return tfData.ToArray();
        }

        protected override void OnSensorDestroy()
        {
        }
    }
}
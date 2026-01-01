using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnitySensors.ROS.Serializer.Std;
using UnitySensors.ROS.Utils.Time;
using UnitySensors.Sensor.TF;

namespace UnitySensors.ROS.Serializer.Tf2
{

    [System.Serializable]
    public class TFMessageMsgSerializer : RosMsgSerializer <TFMessageMsg>
    {
        [SerializeField]
        TFLink _source;

        [SerializeField, Tooltip("If true, it will recursively gather TF data from its children. " +
            "If false, it will only provide TF data of the most immediate children.")]
        bool _recurseFindChildLinks = true;

        [Tooltip("If true, it will prepend the name of the gameObject containing base_link to the frame_id. " +
            "For example, base_link -> robot/base_link and imu_link -> robot/imu_link. " +
            "\n\nTick true only when there are multiple base_link as children (aka a map publisher). ")]
        public bool useBaseLinkNameAsPrefix;

        [Tooltip("Add a suffix to all link name if not null. For example, base_link -> base_link_gt.")]
        public string suffix = "_gt";

        // Does not need to be exposed in inspector because frame_id is not needed and header time is automatically found
        [SerializeField, HideInInspector]
        HeaderSerializer _header;

        public TFLink Source { get => _source; set => _source = value; }

        public override void Init()
        {
            base.Init();
            _header.Init();
        }

        public override TFMessageMsg Serialize()
        {
            HeaderMsg headerMsg = _header.Serialize();
            List<TransformStampedMsg> transforms = new List<TransformStampedMsg>();

            TFData[] tfData = _source.GetTFData(_recurseFindChildLinks, useBaseLinkNameAsPrefix, suffix);
            foreach (TFData data in tfData)
            {
                TransformStampedMsg transform = new TransformStampedMsg();
                transform.header = new();
                transform.header.stamp = headerMsg.stamp;
#if ROS2
#else
                transform.header.seq = headerMsg.seq;
#endif
                transform.header.frame_id = data.frame_id_parent;
                transform.child_frame_id = data.frame_id_child;

                // Convert position and rotation according to their frame conventions
                if (data.position_frame_convention == CoordinateSpaceSelection.ENU)
                    transform.transform.translation = data.position.To<ENU>();
                else if (data.position_frame_convention == CoordinateSpaceSelection.FLU)
                    transform.transform.translation = data.position.To<FLU>();
                else 
                    Debug.LogError($"Unsupported TFFrameConvention {data.position_frame_convention} for position");

                if (data.rotation_frame_convention == CoordinateSpaceSelection.ENU)
                    transform.transform.rotation = data.rotation.To<ENU>();
                else if (data.rotation_frame_convention == CoordinateSpaceSelection.FLU)
                    transform.transform.rotation = data.rotation.To<FLU>();
                else
                    Debug.LogError($"Unsupported TFFrameConvention {data.rotation_frame_convention} for rotation");

                transforms.Add(transform);
            }
            _msg.transforms = transforms.ToArray();
            return _msg;
        }
    }
}
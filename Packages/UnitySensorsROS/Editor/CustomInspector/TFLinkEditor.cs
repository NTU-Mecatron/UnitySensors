using UnityEditor;
using UnitySensors.Sensor.TF;
using UnitySensors.ROS.Publisher.Tf2;

namespace UnitySensors.ROS.Editor
{
    //[CustomEditor(typeof(TFLink))]
    //public class TFLinkEditor : Editor
    //{
    //    SerializedProperty _frame_id;
    //    SerializedProperty _children;

    //    void OnEnable()
    //    {
    //        _frame_id = serializedObject.FindProperty("_frame_id");
    //        _children = serializedObject.FindProperty("_children");
    //    }

    //    public override void OnInspectorGUI()
    //    {
    //        serializedObject.Update();

    //        TFLink tfLink = (TFLink)target;
    //        bool hasTFMessagePublisher = tfLink.GetComponent<TFMessageMsgPublisher>() != null;

    //        // Link settings header
    //        EditorGUILayout.LabelField("Link settings", EditorStyles.boldLabel);
    //        EditorGUILayout.PropertyField(_frame_id);
    //        EditorGUILayout.PropertyField(_children);

    //        serializedObject.ApplyModifiedProperties();
    //    }
    //}
}
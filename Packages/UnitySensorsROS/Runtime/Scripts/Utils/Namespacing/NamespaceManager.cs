using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnitySensors.ROS.Utils.Namespacing
{
    public class NamespaceManager : MonoBehaviour
    {
        [Tooltip("The namespace for this GameObject and its children. Should start with a '/' if it's an absolute namespace and you want to ignore the namespace in the parents..")]
        [SerializeField] private string _currentNamespace = "";

        public string CurrentNamespace => _currentNamespace;
    }

    public static class NamespaceUtils
    {
        public static string GetResolvedTopicName(string topicName, GameObject currentObject)
        {
            if (string.IsNullOrEmpty(topicName))
            {
                Debug.LogWarning("Topic name is empty.");
                return null;
            }

            if (topicName.EndsWith("/"))
            {
                Debug.Log("Topic name should not end with a slash. Automatically remove it now.");
                return topicName.TrimEnd('/');
            }

            // If the topic name is already absolute, return it as is
            if (topicName.StartsWith("/"))
            {
                return topicName;
            }

            // Recursively look up the parent hierarchy for NamespaceManager
            string resolvedNamespace = "";
            Transform parent = currentObject.transform.parent;
            while (parent != null)
            {
                NamespaceManager parentNamespaceManager = parent.GetComponent<NamespaceManager>();
                if (parentNamespaceManager != null)
                {
                    resolvedNamespace = $"{parentNamespaceManager.CurrentNamespace}/{resolvedNamespace}".Replace("//", "/");
                }

                if (resolvedNamespace.StartsWith("/"))
                {
                    break; // Found an absolute namespace, stop searching
                }
                parent = parent.parent;
            }

            // Combine the resolved namespace with the topic name
            return $"{resolvedNamespace}/{topicName}".Replace("//", "/");
        }
    }
}

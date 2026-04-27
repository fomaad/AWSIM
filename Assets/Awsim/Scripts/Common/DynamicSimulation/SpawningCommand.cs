using Awsim.Common.TraceObjects;
using UnityEngine;

namespace Awsim.Common.DynamicCommand
{
    [System.Serializable]
    public class SpawningCommand
    {
        public float timestamp;
        public string name;
        public string body_style;
        public Vector3Object position;
        public QuaternionObject orientation;

        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}
using UnityEngine;

namespace Awsim.Common.DynamicCommand
{
    [System.Serializable]
    public class RemoveObjectCommand
    {
        public float timestamp;
        
        // if target leave unspecified, despawn all NPCs existing in the simulation
        public string target;

        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}
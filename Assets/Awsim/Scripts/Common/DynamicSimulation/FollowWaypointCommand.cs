using Awsim.Common.AWSIM_Script;
using Awsim.Common.TraceObjects;
using UnityEngine;

namespace Awsim.Common.DynamicCommand
{
    [System.Serializable]
    public class FollowWaypointCommand
    {
        public float timestamp;
        public string target;
        public Vector3Object[] waypoints;
        public float speed = NPCConfig.DUMMY_SPEED;
        public float acceleration = NPCConfig.DUMMY_ACCELERATION;
        public float deceleration = NPCConfig.DUMMY_DECELERATION;
        public bool is_speed_defined, is_acceleration_defined, is_deceleration_defined;
        public override string ToString()
        {
            return JsonUtility.ToJson(this);
        }
    }
}
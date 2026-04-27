using System;

namespace Awsim.Common.TraceObjects
{
    [Serializable]
    public class PoseObject
    {
        public Vector3Object position;
        public QuaternionObject quaternion;

        public bool Equals(PoseObject other)
        {
            return position.Equals(other.position) && quaternion.Equals(other.quaternion);
        }

        public static PoseObject FromRosPose(geometry_msgs.msg.Pose input)
        {
            return new PoseObject()
            {
                position = new Vector3Object(input.Position.X, input.Position.Y, input.Position.Z),
                quaternion = new QuaternionObject(input.Orientation.X, input.Orientation.Y,
                    input.Orientation.Z, input.Orientation.W)
            };
        }
    }
}
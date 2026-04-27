using System;

namespace Awsim.Common.TraceObjects
{
    public class AccelerationObject
    {
        public Vector3Object linear;
        public Vector3Object angular;

        public bool Equals(AccelerationObject other)
        {
            return linear.Equals(other.linear) && angular.Equals(other.angular);
        }
    }
}
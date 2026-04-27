using System;

namespace Awsim.Common.TraceObjects
{
    public class BoundingBoxObject
    {
        public double x;
        public double y;
        public double width;
        public double height;

        public bool Equals(BoundingBoxObject other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && width.Equals(other.width) && height.Equals(other.height);
        }
    }
}
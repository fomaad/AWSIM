namespace Awsim.Common.TraceObjects
{
    public class EgoDetailObject
    {
        public Vector3Object center;
        public Vector3Object extents;

        public double RootToFront()
        {
            return center.z + extents.z;
        }
    }
}
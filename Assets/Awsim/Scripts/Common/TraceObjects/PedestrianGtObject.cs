namespace Awsim.Common.TraceObjects
{
    public class PedestrianGtObject
    {
        public string name;
        public Pose2Object pose;
        public float speed;
        public BoundingBoxObject bounding_box; //2d bounding box on camera view
    }
}
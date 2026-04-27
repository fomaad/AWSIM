namespace Awsim.Common.TraceObjects
{
    public class PredictPathObject
    {
        public double confidence;
        // [sec] time step for each path step
        public double time_step;
        public Pose2Object[] path;
    }
}
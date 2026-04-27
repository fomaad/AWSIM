using System.Collections.Generic;

namespace Awsim.Common.TraceObjects
{
    public class StateObject
    {
        public double timeStamp;
        public EgoGroundTruthObject groundtruth_ego;
        public NPCGroundTruthObject[] groundtruth_NPCs;
        public PedestrianGtObject[] groundtruth_pedestrians;
        
        public List<PerceptionObject> perception_objects;
        public List<BBPerceptionObject> boundingbox_perception_objects;
        
        public PlanTrajectory plan_trajectory;
    }
}
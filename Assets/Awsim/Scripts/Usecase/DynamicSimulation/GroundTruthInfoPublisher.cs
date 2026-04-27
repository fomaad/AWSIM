using System.Linq;
using ROS2;
using UnityEngine;
using aw_monitor.msg;
using Awsim.Common;
using Awsim.Common.TraceObjects;
using Awsim.Entity;

namespace Awsim.Usecase.DynamicSimulation
{
    public class GroundTruthInfoPublisher
    {
        protected readonly AccelVehicle _egoVehicle;
        protected readonly Camera _sensorCamera;
        protected readonly float _maxDistanceVisibleOnCamera;
        
        
        // inner use
        QosSettings qosSettings = new QosSettings(
            ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            10);

        readonly string _gtKinematicTopic = "/simulation/gt/kinematic";
        private GroundtruthKinematic _gtKinematicMsg;
        readonly IPublisher<GroundtruthKinematic> _gtKinematicPublisher;

        readonly string _gtSizeTopic = "/simulation/gt/size";
        private GroundtruthSize _gtSizeMsg;

        readonly IPublisher<GroundtruthSize> _gtSizePublisher;
        // cached
        private aw_monitor.msg.VehicleSize _egoSize;

        readonly string _metadataTopic = "/awsim/sim_metadata";
        readonly IPublisher<std_msgs.msg.String> _metadataPublisher;
        private std_msgs.msg.String _metadata = new (){Data = "{}"};
        
        readonly IPublisher<aw_monitor.msg.ExecutionState> _executionStatePublisher;
        private aw_monitor.msg.ExecutionState _executionStateMsg;
        
        public GroundTruthInfoPublisher(GameObject egoCarGO)
        {
            _egoVehicle = egoCarGO.GetComponent<AccelVehicle>();
            _egoSize = StatusExtraction.GetEgoSize(egoCarGO, true);
            
            // Debug.Log($"Center: {_egoSize.Center.X} {_egoSize.Center.Y} {_egoSize.Center.Z}, " +
            //           $"Size: {_egoSize.Size.X} {_egoSize.Size.Y} {_egoSize.Size.Z}");
            
            var cameraobj = GameObject.FindGameObjectsWithTag("ObjectDetectionCamera").FirstOrDefault();
            if (cameraobj == null)
            {
                Debug.LogError("[DynamicSim] Could not find object detection camera.");
                _sensorCamera = egoCarGO.GetComponentInChildren<Camera>();
            }
            else
            {
                _sensorCamera = cameraobj.GetComponentInChildren<Camera>();
                var cameraRos2Publisher = cameraobj.GetComponent<CameraRos2Publisher>();
                cameraRos2Publisher.Initialize();
            }

            _maxDistanceVisibleOnCamera = 200;
            
            _gtKinematicPublisher = AwsimRos2Node.CreatePublisher<GroundtruthKinematic>(_gtKinematicTopic, qosSettings.GetQosProfile());
            _gtSizePublisher = AwsimRos2Node.CreatePublisher<GroundtruthSize>(_gtSizeTopic, qosSettings.GetQosProfile());
            _metadataPublisher = AwsimRos2Node.CreatePublisher<std_msgs.msg.String>(_metadataTopic, qosSettings.GetQosProfile());
            _executionStatePublisher = AwsimRos2Node.CreatePublisher<aw_monitor.msg.ExecutionState>("/simulation/gt/execution_state", qosSettings.GetQosProfile());

            AwsimRos2Node.CreateService<aw_monitor.srv.GroundtruthKinematic_Request, aw_monitor.srv.GroundtruthKinematic_Response>(
                "/simulation/gt_srv/kinematic",
                HandleGtKinematicRequest);
            AwsimRos2Node.CreateService<aw_monitor.srv.GroundtruthSize_Request, aw_monitor.srv.GroundtruthSize_Response>(
                "/simulation/gt_srv/size",
                HandleGtSizeRequest);
            
            ExecutionStateTracker.Initialize();
            
            AwsimRos2Node.CreateService<aw_monitor.srv.ExecutionState_Request, aw_monitor.srv.ExecutionState_Response>(
                "/simulation/gt_srv/execution_state",
                HandleExecutionStateRequest);
        }
        
        private aw_monitor.srv.GroundtruthKinematic_Response HandleGtKinematicRequest(aw_monitor.srv.GroundtruthKinematic_Request msg)
        {
            if (_gtKinematicMsg == null)
                _gtKinematicMsg = new GroundtruthKinematic();
            return new aw_monitor.srv.GroundtruthKinematic_Response()
            {
                Stamp = _gtKinematicMsg.Stamp,
                Groundtruth_ego = _gtKinematicMsg.Groundtruth_ego,
                Groundtruth_vehicles = _gtKinematicMsg.Groundtruth_vehicles,
                Groundtruth_pedestrians = _gtKinematicMsg.Groundtruth_pedestrians,
            };
        }
        private aw_monitor.srv.GroundtruthSize_Response HandleGtSizeRequest(aw_monitor.srv.GroundtruthSize_Request msg)
        {
            if (_gtSizeMsg == null)
                _gtSizeMsg = new GroundtruthSize();
            return new aw_monitor.srv.GroundtruthSize_Response()
            {
                Vehicle_sizes = _gtSizeMsg.Vehicle_sizes,
                Camera_screen_height = _gtSizeMsg.Camera_screen_height,
                Camera_screen_width = _gtSizeMsg.Camera_screen_width,
            };
        }
        

        private aw_monitor.srv.ExecutionState_Response HandleExecutionStateRequest(
            aw_monitor.srv.ExecutionState_Request msg)
        {
            if (_executionStateMsg == null)
                _executionStateMsg = new aw_monitor.msg.ExecutionState();
            return new aw_monitor.srv.ExecutionState_Response()
            {
                Stamp = _executionStateMsg.Stamp,
                Motion_state = _executionStateMsg.Motion_state,
                Routing_state = _executionStateMsg.Routing_state,
                Operation_state = _executionStateMsg.Operation_state,
                Is_autonomous_mode_available = _executionStateMsg.Is_autonomous_mode_available,
            };
        }

        public void OnFixedUpdate()
        {
            Publish();
        }

        private void Publish()
        {
            PublishKinematic();
            PublishGtSize();
            PublishExecutionState();
        }

        private void PublishKinematic()
        {
            _gtKinematicMsg = ExtractKinematics();
            _gtKinematicPublisher.Publish(_gtKinematicMsg);
            _metadataPublisher.Publish(_metadata);
        }

        public void SetMetadataAndPublish(string data)
        {
            _metadata = new std_msgs.msg.String() { Data = data };
        }

        private GroundtruthKinematic ExtractKinematics()
        {
            var egoInfo = StatusExtraction.ExtractEgoKinematic(_egoVehicle, useROSCoord:true);
                
            var npcVehicles = CustomSimulationManager.AllNPCVehicles;
            var npcInfo = StatusExtraction.ExtractNPCKinematics(npcVehicles, useROSCoord:true);
            var bboxes = StatusExtraction.Extract2DVehicleBoundingBoxes(
                npcVehicles, _sensorCamera, _egoVehicle, _maxDistanceVisibleOnCamera);
            for (int i = 0; i < npcInfo.Length; i++)
            {
                if (bboxes[i] != null)
                    npcInfo[i].bounding_box = bboxes[i];
            }

            // pedestrians
            var npcPedestrians = CustomSimulationManager.AllPedestrians;
            var pedesInfo = StatusExtraction.ExtractPedestrians(npcPedestrians, useROSCoord:true);
            var bboxes2 = StatusExtraction.Extract2DPedestrianBoundingBoxes(
                npcPedestrians, _sensorCamera, _egoVehicle, _maxDistanceVisibleOnCamera);
            for (int i = 0; i < pedesInfo.Length; i++)
            {
                if (bboxes2[i] != null)
                    pedesInfo[i].bounding_box = bboxes2[i];
            }

            _gtKinematicMsg = new GroundtruthKinematic()
            {
                Groundtruth_ego = ToROSPoseTwistAccel(egoInfo),
                // Groundtruth_vehicles = ToROSGtInfo(npcInfo),
                // groundtruth_pedestrian = pedesInfo,
                Stamp = AwsimRos2Node.GetCurrentRosTime()
            };
            _gtKinematicMsg.Groundtruth_vehicles = new GroundtruthNPCVehicle[npcInfo.Length];
            _gtKinematicMsg.Groundtruth_pedestrians = new GroundtruthNPCPedestrian[pedesInfo.Length];
            for (int i = 0; i < npcInfo.Length; i++)
            {
                _gtKinematicMsg.Groundtruth_vehicles[i] = ToROSVehGtInfo(npcInfo[i]);
            }

            for (int i = 0; i < pedesInfo.Length; i++)
            {
                _gtKinematicMsg.Groundtruth_pedestrians[i] = ToROSPedesGtInfo(pedesInfo[i]);
            }
            return _gtKinematicMsg;
        }

        private void PublishGtSize()
        {
            // NPCs details
            var npcs = CustomSimulationManager.AllNPCVehicles;
            var vehicleSizes = new aw_monitor.msg.VehicleSize[npcs.Count + 1];
            vehicleSizes[0] = _egoSize;
            for (int i = 0; i < npcs.Count; i++)
                vehicleSizes[i + 1] = StatusExtraction.GetNPCVehicleSize(npcs[i], true);

            _gtSizeMsg = new GroundtruthSize()
            {
                Vehicle_sizes = vehicleSizes,
                Camera_screen_height = _sensorCamera.pixelHeight,
                Camera_screen_width = _sensorCamera.pixelWidth,
                Other_note = "",
            };
            _gtSizePublisher.Publish(_gtSizeMsg);
        }

        private void PublishExecutionState()
        {
            _executionStateMsg = new aw_monitor.msg.ExecutionState()
            {
                Stamp = AwsimRos2Node.GetCurrentRosTime(),
                Motion_state = ExecutionStateTracker.MotionState,
                Routing_state = ExecutionStateTracker.RoutingState,
                Operation_state = ExecutionStateTracker.OperationState,
                Is_autonomous_mode_available = ExecutionStateTracker.IsAutonomousModeAvailable,
            };
            _executionStatePublisher.Publish(_executionStateMsg);
        }
        
        // public functions
        public static PoseTwistAccel ToROSPoseTwistAccel(EgoGroundTruthObject egoGroundTruthObj)
        {
            return new PoseTwistAccel()
            {
                Pose = ToROSCustomPose(egoGroundTruthObj.pose),
                Twist = ToROSTwist(egoGroundTruthObj.twist),
                Accel = ToROSAccel(egoGroundTruthObj.acceleration)
            };
        }

        public static GroundtruthNPCVehicle ToROSVehGtInfo(NPCGroundTruthObject npcGroundTruthObj)
        {
            var result = new GroundtruthNPCVehicle()
            {
                Name = npcGroundTruthObj.name,
                Pose = ToROSCustomPose(npcGroundTruthObj.pose),
                Twist = ToROSTwist(npcGroundTruthObj.twist),
                Accel = npcGroundTruthObj.acceleration,
            };
            if (npcGroundTruthObj.bounding_box != null)
                result.Bounding_box = ToROSBoundingBox(npcGroundTruthObj.bounding_box);
            return result;
        }
        
        public static GroundtruthNPCPedestrian ToROSPedesGtInfo(PedestrianGtObject pedesGtObj)
        {
            var result = new GroundtruthNPCPedestrian()
            {
                Name = pedesGtObj.name,
                Pose = ToROSCustomPose(pedesGtObj.pose),
                Speed = pedesGtObj.speed,
            };
            if (pedesGtObj.bounding_box != null)
                result.Bounding_box = ToROSBoundingBox(pedesGtObj.bounding_box);
            return result;
        }

        public static CustomPose ToROSCustomPose(Pose2Object input)
        {
            return new CustomPose()
            {
                Position = ToROSVector3(input.position),
                Rotation = ToROSVector3(input.rotation),
            };
        }

        public static geometry_msgs.msg.Twist ToROSTwist(TwistObject input)
        {
            return new geometry_msgs.msg.Twist()
            {
                Linear = ToROSVector3(input.linear),
                Angular = ToROSVector3(input.angular),
            };
        }

        public static geometry_msgs.msg.Accel ToROSAccel(AccelerationObject input)
        {
            return new geometry_msgs.msg.Accel()
            {
                Linear = ToROSVector3(input.linear),
                Angular = ToROSVector3(input.angular),
            };
        }

        public static BoundingBox ToROSBoundingBox(BoundingBoxObject input)
        {
            return new BoundingBox()
            {
                X = input.x,
                Y = input.y,
                Width = input.width,
                Height = input.height,
            };
        }

        public static geometry_msgs.msg.Vector3 ToROSVector3(Vector3Object input)
        {
            return new geometry_msgs.msg.Vector3()
            {
                X = input.x, Y = input.y, Z = input.z
            };
        }
    }
}
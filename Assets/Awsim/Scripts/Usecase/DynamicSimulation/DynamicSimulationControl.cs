using System;
using System.Collections.Generic;
using System.Linq;
using aw_monitor.srv;
using Awsim.Common;
using Awsim.Common.AWSIM_Script;
using Awsim.Common.DynamicCommand;
using Awsim.Entity;
using Awsim.Usecase.TrafficSimulation;
using RGLUnityPlugin;
using ROS2;
using UnityEngine;
using ResponseStatus = aw_monitor.msg.ResponseStatus;

namespace Awsim.Usecase.DynamicSimulation
{
    public class DynamicSimulationControl : MonoBehaviour
    {
        #region SerializeFields

        [SerializeField]
        private GameObject egoCar;
        
        [SerializeField]
        private TrafficSimNpcVehicle npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan;
        
        [SerializeField]
        private GameObject casualPedestrian, elegantPedestrian;
        
        [SerializeField]
        private LayerMask obstacleLayerMask, groundLayerMask;

        #endregion

        #region consts (e.g., topic and service names)
        public const string TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN = "/dynamic_control/vehicle/spawn";
        public const string TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE = "/dynamic_control/vehicle/follow_lane";

        public const string TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS =
            "/dynamic_control/vehicle/follow_waypoints";
        public const string TOPIC_DYNAMIC_CONTROL_SET_TARGET_SPEED = "/dynamic_control/vehicle/target_speed";

        public const string TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING = "/dynamic_control/npc/remove"; // applied to both vehicles and pedestrians
        public const string TOPIC_DYNAMIC_CONTROL_AWSIM_SCRIPT = "/dynamic_control/script/awsim_script";
        public const string TOPIC_EGO_ESTIMATED_KINEMATICS = "/api/vehicle/kinematics";
        
        public const string TOPIC_DYNAMIC_CONTROL_PEDESTRIAN_SPAWN = "/dynamic_control/pedestrian/spawn";
        public const string TOPIC_DYNAMIC_CONTROL_PEDESTRIAN_FOLLOW_WAYPOINTS =
            "/dynamic_control/pedestrian/follow_waypoints";

        // service to check whether the spawning, follow lane, etc. actions sent before 
        // were successfully applied without any errors
        public const string SRV_DYNAMIC_CONTROL_VEHCILE_SPAWN = TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN + "_srv";

        public const string SRV_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE =
            TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE + "_srv";

        public const string SRV_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS =
            TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS + "_srv";
        
        public const string SRV_DYNAMIC_CONTROL_SET_TARGET_SPEED =
            TOPIC_DYNAMIC_CONTROL_SET_TARGET_SPEED + "_srv";
        
        public const string SRV_DYNAMIC_CONTROL_VEHICLE_REMOVING =
            TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING + "_srv";
        
        public const string SRV_DYNAMIC_CONTROL_PEDESTRIAN_SPAWN = TOPIC_DYNAMIC_CONTROL_PEDESTRIAN_SPAWN + "_srv";
        public const string SRV_DYNAMIC_CONTROL_PEDESTRIAN_FOLLOW_WAYPOINTS =
            TOPIC_DYNAMIC_CONTROL_PEDESTRIAN_FOLLOW_WAYPOINTS + "_srv";
        
        public const string SRV_DYNAMIC_CONTROL_AWSIM_SCRIPT =
            TOPIC_DYNAMIC_CONTROL_AWSIM_SCRIPT + "_srv";
        public const string SRV_DYNAMIC_CONTROL_MAP_NETWORK =
            "/dynamic_control/map/network";
        
        public const string LOCALIZATION_INITIALIZATION_SRV = "/api/localization/initialize";
        
        public const string TOPIC_INITIAL_POSE = "/initialpose";
        
        #endregion

        #region private internal fields

        // queues of publisher messages sent from client (e.g., AWSIM-Script and Scenic)
        // Note that we cannot handle the requested action inside ROS subscription callbacks.
        // This is because the implementation needs to use some Unity functions that are only accessible from the main thread.
        // whereas, the callbacks are fired in the background thread (ROS spinning).
        private Queue<std_msgs.msg.String> _spawnReqQueue = new();
        private Queue<std_msgs.msg.String> _followLaneReqQueue = new();
        private Queue<std_msgs.msg.String> _followWaypointsReqQueue = new();
        private Queue<std_msgs.msg.String> _setTargetSpeedReqQueue = new();
        private Queue<std_msgs.msg.String> _removeReqQueue = new();
        private Queue<std_msgs.msg.String> _awsimScriptReqQueue = new();
        private Queue<std_msgs.msg.String> _pedestrianSpawnReqQueue = new();
        private Queue<std_msgs.msg.String> _pedestrianFollowWaypointReqQueue = new();
        
        private Queue<geometry_msgs.msg.PoseWithCovarianceStamped> _poseReqQueue = new();

        // saving the map (requests |-> responses), where
        // requests are (in form of json string) published msg from clients for making actions (e.g., spawning)
        // responses are (instance of DynamicControl_Response) are the processed results.
        // Note that it is impossible to implement a service server instead, i.e.,
        // blocking until finishing processing actions.
        // This is because the action implementation must be done in the main thread.
        Dictionary<string, DynamicControl_Response> _spawnReqResDict = new();
        Dictionary<string, DynamicControl_Response> _followLaneReqResDict = new();
        Dictionary<string, DynamicControl_Response> _followWaypointsReqResDict = new();
        Dictionary<string, DynamicControl_Response> _setTargetSpeedReqResDict = new();
        Dictionary<string, DynamicControl_Response> _removeReqResDict = new();
        Dictionary<string, DynamicControl_Response> _awsimScriptReqResDict = new();
        Dictionary<string, DynamicControl_Response> _pedestrianSpawnReqResDict = new();
        Dictionary<string, DynamicControl_Response> _pedestrianFollowWaypointReqResDict = new();

        private Dictionary<String, DynamicControl_Response> _egoPoseReqResDict = new();
        
        private MapNetworkWrapper _mapNetworkWrapper;
        
        private GroundTruthInfoPublisher _groundTruthInfoPublisher;
        
        #endregion

        public void Awake()
        {
            var lidarSensors = egoCar.GetComponentsInChildren<LidarSensor>();
            bool noiseEnabled = LoadCliArg();
            foreach (var sensor in lidarSensors)
            {
                sensor.OnAwake(noiseEnabled);
            }
        }

        public void Initialize()
        {
            CustomSimulationManager.Initialize(this.gameObject, egoCar,
                npcTaxi, npcHatchback, npcSmallCar, npcTruck, npcVan,
                casualPedestrian, elegantPedestrian, obstacleLayerMask, groundLayerMask);
            InitializeROS();
            InitializeSimulationPublisher();
            
            _mapNetworkWrapper = new MapNetworkWrapper(CustomSimulationManager.GetAllTrafficLanes());
        }
        
        private void InitializeSimulationPublisher()
        {
            _groundTruthInfoPublisher = new GroundTruthInfoPublisher(egoCar);
            Debug.Log("[AWAnalysis] Initialized ground truth kinematic publisher.");
        }
        
        public void InitializeROS()
        {
            var qos = new QosSettings
            (
                ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                1
            ).GetQosProfile();

            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHCILE_SPAWN,
                msg => { _spawnReqQueue.Enqueue(msg); },
                qos);
            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE,
                msg => { _followLaneReqQueue.Enqueue(msg); },
                qos);
            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS,
                msg => { _followWaypointsReqQueue.Enqueue(msg); },
                qos);
            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_SET_TARGET_SPEED,
                msg => { _setTargetSpeedReqQueue.Enqueue(msg); },
                qos);
            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_VEHICLE_REMOVING,
                msg => { _removeReqQueue.Enqueue(msg); },
                qos);
            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_AWSIM_SCRIPT,
                msg => { _awsimScriptReqQueue.Enqueue(msg); },
                qos);
            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_PEDESTRIAN_SPAWN,
                msg => { _pedestrianSpawnReqQueue.Enqueue(msg); },
                qos);
            AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                TOPIC_DYNAMIC_CONTROL_PEDESTRIAN_FOLLOW_WAYPOINTS,
                msg => { _pedestrianFollowWaypointReqQueue.Enqueue(msg); },
                qos);

            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHCILE_SPAWN,
                msg => _spawnReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));

            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHCILE_FOLLOW_LANE,
                msg =>
                    _followLaneReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));

            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHICLE_FOLLOW_WAYPOINTS,
                msg =>
                    _followWaypointsReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));
            
            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_SET_TARGET_SPEED,
                msg =>
                    _setTargetSpeedReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));
            
            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_VEHICLE_REMOVING,
                msg =>
                    _removeReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));

            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_AWSIM_SCRIPT,
                msg =>
                    _awsimScriptReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));
            
            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_MAP_NETWORK,
                HandleMapNetworkReq);
            
            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_PEDESTRIAN_SPAWN,
                msg =>
                    _pedestrianSpawnReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));
            AwsimRos2Node.CreateService<DynamicControl_Request, DynamicControl_Response>(
                SRV_DYNAMIC_CONTROL_PEDESTRIAN_FOLLOW_WAYPOINTS,
                msg =>
                    _pedestrianFollowWaypointReqResDict.GetValueOrDefault(msg.Json_request, UNPROCESSED_REQ()));
            
            AwsimRos2Node.CreateSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(
                    TOPIC_INITIAL_POSE, 
                msg => {_poseReqQueue.Enqueue(msg); }, 
                new QosSettings
                (
                    ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                    DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                    HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                    5
                ).GetQosProfile());
        }
        
        private DynamicControl_Response UNPROCESSED_REQ()
        {
            return new DynamicControl_Response()
            {
                Status = new ResponseStatus()
                {
                    Code = 2,
                    Success = false,
                    Message = "Unprocessed for invalid request."
                }
            };
        }
        private DynamicControl_Response INVALID_REQ(Exception e)
        {
            return new DynamicControl_Response()
            {
                Status = new ResponseStatus()
                {
                    Code = 3,
                    Success = false,
                    Message = e.Message
                }
            };
        }
        private static ResponseStatus SuccessResponseStatus(string message="Success")
        {
            return new ResponseStatus
            {
                Code = 0,
                Message = message,
                Success = true
            };
        }

        public void OnUpdate()
        {
            CustomSimulationManager.Instance.OnUpdate();

            while (_spawnReqQueue.Count > 0)
            {
                var req = _spawnReqQueue.Dequeue();
                DynamicControl_Response response = null;

                try
                {
                    var command = JsonUtility.FromJson<SpawningCommand>(req.Data);
                    Debug.Log($"Received SPAWN command: {command}");
                    response = HandleSpawnAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _spawnReqResDict[req.Data] = response;
                }
            }
            while (_followLaneReqQueue.Count > 0)
            {
                var req = _followLaneReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<FollowLaneCommand>(req.Data);
                    Debug.Log($"Received FOLLOW LANE command: {command}");
                    response = HandleFollowLaneAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _followLaneReqResDict[req.Data] = response;
                }
            }
            while (_followWaypointsReqQueue.Count > 0)
            {
                var req = _followWaypointsReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<FollowWaypointCommand>(req.Data);
                    Debug.Log($"Received FOLLOW WAYPOINTS command: {command}");
                    response = HandleFollowWaypointsAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _followWaypointsReqResDict[req.Data] = response;
                }
            }
            while (_setTargetSpeedReqQueue.Count > 0)
            {
                var req = _setTargetSpeedReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<SetTargetSpeedCommand>(req.Data);
                    Debug.Log($"Received SET SPEED command: {command}");
                    response = HandleSetTargetSpeedAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _setTargetSpeedReqResDict[req.Data] = response;
                }
            }

            while (_removeReqQueue.Count > 0)
            {
                var req = _removeReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<RemoveObjectCommand>(req.Data);
                    Debug.Log($"Recevied REMOVE ACTOR command: {command}");
                    response = HandleRemoveAction(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _removeReqResDict[req.Data] = response;
                }
            }
            
            while (_pedestrianSpawnReqQueue.Count > 0)
            {
                var req = _pedestrianSpawnReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<SpawningCommand>(req.Data);
                    Debug.Log($"Received pedestrian SPAWN command: {command}");
                    response = HandlePedesSpawn(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _pedestrianSpawnReqResDict[req.Data] = response;
                }
            }

            while (_pedestrianFollowWaypointReqQueue.Count > 0)
            {
                var req = _pedestrianFollowWaypointReqQueue.Dequeue();
                DynamicControl_Response response = null;
                try
                {
                    var command = JsonUtility.FromJson<FollowWaypointCommand>(req.Data);
                    Debug.Log($"Received pedestrian FOLLOW WAYPOINTS command: {command}");
                    response = HandlePedesFollowWaypoints(command);
                }
                catch (ArgumentException e)
                {
                    response = INVALID_REQ(e);
                }
                finally
                {
                    _pedestrianFollowWaypointReqResDict[req.Data] = response;
                }
            }
        }

        public void OnFixedUpdate()
        {
            _timeNow = Time.fixedTime;

            while (_poseReqQueue.Count > 0)
            {
                var req = _poseReqQueue.Dequeue();
                var response = HandleEgoPoseReq(req);
                _egoPoseReqResDict[req.ToString()] = response;
            }
            
            CustomSimulationManager.Instance.OnFixedUpdate();
            _groundTruthInfoPublisher.OnFixedUpdate();
        }

        #region Handlers for dynamic actions

        private DynamicControl_Response HandleEgoPoseReq(geometry_msgs.msg.PoseWithCovarianceStamped msg)
        {
            Vector3 subscribed_pos = new Vector3(
                (float)msg.Pose.Pose.Position.X,
                (float)msg.Pose.Pose.Position.Y,
                (float)msg.Pose.Pose.Position.Z);

            var position = Ros2Utility.Ros2ToUnityPosition(subscribed_pos - MgrsPosition.Instance.Mgrs.Position);
            var rotation = Ros2Utility.Ros2ToUnityRotation(msg.Pose.Pose.Orientation);

            Scene.AutowareSimulationDemo.EgoVehicle egoVehicle = egoCar.GetComponent<Scene.AutowareSimulationDemo.EgoVehicle>();
            Vector3 rayOrigin = new Vector3(position.x, 1000.0f, position.z);
            Vector3 rayDirection = Vector3.down;

            if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, Mathf.Infinity))
            {
                egoCar.transform.position = new Vector3(position.x, hit.point.y + 1.33f, position.z);
                egoCar.transform.transform.rotation = rotation;

                Debug.Log($"Reset Ego position: ({position.x}, {hit.point.y + 1.33f}, {position.z})");
                return new DynamicControl_Response
                {
                    Status = SuccessResponseStatus()
                };
            }

            string log = "No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.";
            Debug.Log(log);
            return new DynamicControl_Response
            {
                Status = new ResponseStatus
                {
                    Code = 1,
                    Message = log,
                    Success = false
                }
            };
        }
        
        private DynamicControl_Response HandleSpawnAction(SpawningCommand command)
        {
            var position = Ros2Utility.RosMGRSToUnityPosition(command.position);
            var lane = DynamicSimUtils.LaneAtPosition(position, out int waypointId, out float laneOffset,
                tolerance: 0.5f);
            if (lane == null)
            {
                Debug.LogError($"Cannot find lane for the spawning position {position}");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"Cannot find lane for the spawning position {position}.",
                        Success = false
                    }
                };
            }
            
            // if an actor exists with the same name, remove it first
            var targetNPC = CustomSimulationManager.AllNPCVehicles.Find(npc => npc.NpcVehicle.ScriptName == command.name);
            if (targetNPC != null)
            {
                Debug.LogError($"An NPC with the name {command.name} already exists. Removing it.");
                if (!RemoveSingleNPC(targetNPC))
                    Debug.LogError($"Could not remove the NPC {command.name}");
            }
            
            var spawnPosition = new LaneOffsetPosition(lane.name, laneOffset);
            
            // construct configuration
            NPCCar npc = new NPCCar(ScenarioParser.ParseVehicleType(command.body_style), spawnPosition);
            npc.Name = command.name;
            npc.SpawnDelayOption = NPCDelayTime.DelayMove(float.MaxValue);
            CustomSimulationManager.SpawnNPCAndDelayMovement(npc);
            Debug.Log(
                $"[AWAnalysis] spawned NPC {command.name} at position {position}, lane {lane.name}, offset {laneOffset}");

            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus()
            };
        }
        
        private DynamicControl_Response HandleFollowLaneAction(FollowLaneCommand command)
        {
            var targetNPC = CustomSimulationManager.AllNPCVehicles.Find(npc => npc.NpcVehicle.ScriptName == command.target);
            if (targetNPC == null)
            {
                Debug.LogError($"[AWAnalysis] Target NPC {command.target} not found.");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"NPC {command.target} not found.",
                        Success = false
                    }
                };
            }
            
            CustomSimulationManager.ResetMotionProfileForNPC(ref targetNPC,
                command.speed, command.acceleration, command.deceleration,
                command.is_speed_defined, command.is_acceleration_defined, command.is_deceleration_defined);
            
            if (CustomSimulationManager.DoesExistInDelayMoveNPCs(targetNPC))
            {
                // if the NPC was spawned but its movement is delayed.
                // Let it move
                CustomSimulationManager.RemoveDelayFromNPC(targetNPC);
            }
            else
            {
                // if the NPC is already moving
                try
                {
                    var trafficLane = DynamicSimUtils.ParseLane(command.lane);
                    CustomSimulationManager.ResetNPCRoute(targetNPC, trafficLane);
                }
                catch (InvalidScriptException exception)
                {
                    return new DynamicControl_Response
                    {
                        Status = new ResponseStatus
                        {
                            Code = 1,
                            Message = exception.Message,
                            Success = false
                        }
                    };
                }
            }

            Debug.Log($"[AWAnalysis] Sent follow lane command to NPC {command.target}");
            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus()
            };
        }
        
        private DynamicControl_Response HandleFollowWaypointsAction(FollowWaypointCommand command)
        {
            var targetNPC = CustomSimulationManager.AllNPCVehicles.Find(npc => npc.NpcVehicle.ScriptName == command.target);
            if (targetNPC == null)
            {
                Debug.LogError($"[AWAnalysis] Target NPC {command.target} not found.");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"NPC {command.target} not found.",
                        Success = false
                    }
                };
            }

            List<Vector3> waypoints = new List<Vector3>();
            foreach (var point in command.waypoints)
            {
                waypoints.Add(Ros2Utility.RosMGRSToUnityPosition(point));
            } 
            PublishMetadata("waypoints", waypoints);
            
            // TODO: handle case waypoint.z = 0

            // construct a virtual traffic lane
            TrafficLane virtualLane;
            // find the lane on which the last waypoint located
            var lane = DynamicSimUtils.LaneAtPosition(waypoints.Last(), out int waypointId, out float laneOffset,
                tolerance: 0.5f);

            if (lane == null)
            {
                Debug.LogWarning("Cannot find next lane for the given waypoints");
                virtualLane = this.gameObject.AddComponent<TrafficLane>();
                virtualLane.name = "VirtualLane";
                virtualLane.UpdateWaypoints(waypoints.ToArray());
                virtualLane.SetSpeedLimit(60);
            }
            else
            {
                Debug.Log($"Found lane {lane.name} as the next lane of the given waypoints");
                var allExistingNames = GetAllChildNames();

                virtualLane = Instantiate(lane, this.transform);
                if (allExistingNames.Contains(virtualLane.name))
                {
                    virtualLane.name += Guid.NewGuid();
                }
                
                // configure waypoints for the virtual lane.
                List<Vector3> virtualLaneWaypoints = new List<Vector3>();
                // The current position of NPC should be inserted as the first waypoint
                // if (CustomSimUtils.DistanceIgnoreYAxis(targetNPC.Position, waypoints[0]) > 1)
                //     virtualLaneWaypoints.Add(targetNPC.Position);

                // add specified waypoints
                virtualLaneWaypoints.AddRange(waypoints);
                // add $lane's waypoints after the last specified waypoint (in order to connect the next lane(s) of $lane) 
                for (int i = waypointId + 1; i < lane.Waypoints.Length; i++)
                    virtualLaneWaypoints.Add(lane.Waypoints[i]);
                virtualLane.UpdateWaypoints(virtualLaneWaypoints.ToArray());

                // reset virtual lane's previous 
                virtualLane.ResetPrevLanes(new List<TrafficLane>());
                virtualLane.ResetNextLanes(lane.NextLanes);
            }
            
            CustomSimulationManager.ResetMotionProfileForNPC(ref targetNPC,
                command.speed, command.acceleration, command.deceleration,
                command.is_speed_defined, command.is_acceleration_defined, command.is_deceleration_defined,
                followCustomWaypoints: 1);
            if (CustomSimulationManager.DoesExistInDelayMoveNPCs(targetNPC))
            {
                // if the NPC was spawned but its movement is delayed.
                // Config the virtual lane as the NPC route (without goal), and let it move
                CustomSimulationManager.ResetLanePositionForNPC(targetNPC, virtualLane);
                CustomSimulationManager.RemoveDelayFromNPC(targetNPC);
            }
            else
            {
                // if the NPC is already moving
                CustomSimulationManager.ResetNPCRoute(targetNPC, virtualLane);
            }

            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus()
            };
        }
        
        private DynamicControl_Response HandleSetTargetSpeedAction(SetTargetSpeedCommand command)
        {
            var targetNPC = CustomSimulationManager.AllNPCVehicles.Find(npc => npc.NpcVehicle.ScriptName == command.target);
            if (targetNPC == null)
            {
                Debug.LogError($"[AWAnalysis] Target NPC {command.target} not found.");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"NPC {command.target} not found.",
                        Success = false
                    }
                };
            }
            
            CustomSimulationManager.ResetMotionProfileForNPC(ref targetNPC,
                command.speed, command.acceleration, command.deceleration,
                true, command.is_acceleration_defined, command.is_deceleration_defined);
            
            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus()
            };
        }

        private DynamicControl_Response HandleRemoveAction(RemoveObjectCommand command)
        {
            List<String> unsuccessfulTargets = new List<String>();
            List<String> unsuccessfulPedestrians = new List<String>();
            
            if (string.IsNullOrEmpty(command.target))
            {
                // remove all NPCs
                int noNPCs = CustomSimulationManager.AllNPCVehicles.Count;
                for (int i = noNPCs - 1; i >= 0; i--)
                {
                    var npc = CustomSimulationManager.AllNPCVehicles[i];
                    if (!RemoveSingleNPC(npc))
                        unsuccessfulTargets.Add(npc.NpcVehicle.ScriptName);
                }

                int noPedes = CustomSimulationManager.AllPedestrians.Count;
                for (int i = noPedes - 1; i >= 0; i--)
                {
                    var pedes = CustomSimulationManager.AllPedestrians[i];
                    if (!RemoveSinglePedestrian(pedes.Item1))
                        unsuccessfulPedestrians.Add(pedes.Item1.Name);
                }
            }
            else
            {
                var targetNPC = CustomSimulationManager.AllNPCVehicles.Find(npc => npc.NpcVehicle.ScriptName == command.target);
                var targetPedes = CustomSimulationManager.AllPedestrians.Find(entry => entry.Item1.Name == command.target);
                if (targetNPC == null && targetPedes == null)
                {
                    Debug.LogError($"[AWAnalysis] Target NPC {command.target} not found.");
                    return new DynamicControl_Response
                    {
                        Status = new ResponseStatus
                        {
                            Code = 1,
                            Message = $"NPC {command.target} not found.",
                            Success = false
                        }
                    };
                }
                if (targetNPC != null)
                    if (!RemoveSingleNPC(targetNPC))
                        unsuccessfulTargets.Add(command.target);
                if (targetPedes != null)
                    if (!RemoveSinglePedestrian(targetPedes.Item1))
                        unsuccessfulPedestrians.Add(command.target);
            }
            
            if (unsuccessfulTargets.Count > 0)
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"Could not despawn NPC(s) {string.Join(", ", unsuccessfulTargets)}.",
                        Success = false
                    }
                };

            var ok = RemoveVirtualLanes();
            if (!ok)
                return new DynamicControl_Response()
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"Could remove spawned virtual traffic lanes",
                        Success = false
                    }
                };
            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus()
            };
        }
        
        private DynamicControl_Response HandlePedesSpawn(SpawningCommand command)
        {
            // if an actor exists with the same name, remove it first
            var existPedes = CustomSimulationManager.AllPedestrians.Find(entry => entry.Item1.Name == command.name);
            if (existPedes != null)
            {
                Debug.LogError($"A pedestrian with the name {command.name} already exists. Removing it.");
                if (!RemoveSinglePedestrian(existPedes.Item1))
                    Debug.LogError($"Could not remove the pedestrian {command.name}");
            }
            
            NPCPedes npcPedes = new NPCPedes(command.name, ScenarioParser.ParseHumanType(command.body_style), null);
            npcPedes.Config.Delay = NPCDelayTime.DelayMove(float.MaxValue);
            npcPedes.LastPosition = Ros2Utility.RosMGRSToUnityPosition(command.position);
            npcPedes.LastRotation = Ros2Utility.RosToUnityRotation(command.orientation);
            CustomSimulationManager.SpawnPedestrianAndDelayMovement(npcPedes, this.gameObject);
            Debug.Log($"[AWAnalysis] spawned PEDESTRIAN {command.name}.");

            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus()
            };
        }

        private DynamicControl_Response HandlePedesFollowWaypoints(FollowWaypointCommand command)
        {
            var targetPedesEntry = CustomSimulationManager.AllPedestrians.Find(
                entry => entry.Item1.Name == command.target);
            if (targetPedesEntry == null)
            {
                Debug.LogError($"[AWAnalysis] Target Pedestrian {command.target} not found.");
                return new DynamicControl_Response
                {
                    Status = new ResponseStatus
                    {
                        Code = 1,
                        Message = $"Pedestrian {command.target} not found.",
                        Success = false
                    }
                };
            }
            
            var targetPedes = targetPedesEntry.Item1;
            List<Vector3> waypoints = new List<Vector3>();
            foreach (var point in command.waypoints)
            {
                waypoints.Add(Ros2Utility.RosMGRSToUnityPosition(point));
            }
            
            CustomSimulationManager.ResetPedestrianProfile(ref targetPedes, waypoints, command.speed, command.is_speed_defined);
            CustomSimulationManager.RemoveDelayFromPedestrian(targetPedes);
            
            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus()
            };
        }

        private bool RemoveSinglePedestrian(NPCPedes target)
        {
            return CustomSimulationManager.DespawnPedestrian(target);
        }

        private bool RemoveVirtualLanes()
        {
            var lanes = GetComponentsInChildren<TrafficLane>();
            try
            {
                foreach (var l in lanes)
                {
                    Destroy(l.gameObject);
                    Debug.Log($"Removed lane {l.name}");
                }
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                return false;
            }
            return true;
        }
        
        private DynamicControl_Response HandleMapNetworkReq(DynamicControl_Request command)
        {
            if (_mapNetworkWrapper == null)
                return UNPROCESSED_REQ();
            string jsonStr = JsonUtility.ToJson(_mapNetworkWrapper);
            return new DynamicControl_Response
            {
                Status = SuccessResponseStatus(jsonStr)
            };
        }
        
        #endregion

        #region Internal functions

        private bool RemoveSingleNPC(TrafficSimNpcVehicle target)
        {
            return CustomSimulationManager.DespawnNPC(target);
        }
        
        private void PublishMetadata(string key, List<Vector3> unityPoints)
        {
            var jsonStrArray = unityPoints.Select(p => JsonUtility.ToJson(Ros2Utility.UnityToRosMGRS(p)));
            string jsonStr = string.Join(", ", jsonStrArray);
            jsonStr = "[" + jsonStr + "]";
            _groundTruthInfoPublisher.SetMetadataAndPublish("{\"" + key + "\": " + jsonStr + "}");
        }
        
        /// <summary>
        /// mainly to enable/disable the noise from LiDAR sensor data
        /// </summary>
        /// <returns>true if noise is enabled; false otherwise</returns>
        private bool LoadCliArg()
        {
            var cmdArgs = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < cmdArgs.Length; i++)
            {
                if (cmdArgs[i] == "-noise" && cmdArgs.Length > i + 1)
                {
                    if (cmdArgs[i + 1].ToLower() == "false")
                        return false;
                }
            }
            return true;
        }

        #endregion

        private static float _timeNow;
        public static float GetFixedTime()
        {
            return _timeNow;
        }
        
        private List<String> GetAllChildNames()
        {
            var childNames = new List<String>();
            for (int i = 0; i < this.transform.childCount; i++)
            {
                childNames.Add(this.transform.GetChild(i).name);
            }
            return childNames;
        }
    }
}
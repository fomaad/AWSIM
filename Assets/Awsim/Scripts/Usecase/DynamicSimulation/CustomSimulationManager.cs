using System;
using System.Collections.Generic;
using System.Linq;
using Awsim.Common.AWSIM_Script;
using Awsim.Common.DynamicCommand;
using Awsim.Entity;
using Awsim.Usecase.TrafficSimulation;
using UnityEngine;

namespace Awsim.Usecase.DynamicSimulation
{
    public class CustomSimulationManager
    {
        private static CustomSimulationManager manager;
        private GameObject _egoCar;
        private TrafficSimNpcVehicle _npcTaxi, _npcHatchback, _npcSmallCar, _npcTruck, _npcVan;
        private GameObject _casualPedestrian, _elegantPedestrian;
        private LayerMask _obstacleLayerMask;
        private LayerMask _groundLayerMask;
        private GameObject _parentGameObject;

        private NpcVehicleSimulator _npcVehicleSimulator;
        private NpcVehicleSpawner _npcVehicleSpawner;
        private TrafficLane[] _allTrafficLanes;
        
        // 2nd item: waypoint index
        private Dictionary<TrafficSimNpcVehicle, Tuple<int, NPCCar>> _delayingMoveNPCs;
        private List<NPCPedes> _delayingMovePedestrians;

        // all NPCs spawned
        private List<TrafficSimNpcVehicle> _npcs;
        private List<Tuple<NPCPedes, Pedestrian>> _pedestrians;

        public static void Initialize(GameObject parentGameObject, GameObject egoVehicle,
            TrafficSimNpcVehicle taxi, TrafficSimNpcVehicle hatchback,
            TrafficSimNpcVehicle smallCar, TrafficSimNpcVehicle truck, TrafficSimNpcVehicle van,
            GameObject casualPedestrian, GameObject elegantPedestrian,
            LayerMask obstacleLM, LayerMask groundLM)
        {
            Instance._npcs = new List<TrafficSimNpcVehicle>();
            Instance._pedestrians = new List<Tuple<NPCPedes, Pedestrian>>();
            Instance._delayingMoveNPCs = new Dictionary<TrafficSimNpcVehicle, Tuple<int, NPCCar>>();
            Instance._delayingMovePedestrians = new List<NPCPedes>();
            
            Instance._parentGameObject = parentGameObject;
            Instance._egoCar = egoVehicle;
            
            Instance._npcTaxi = taxi;
            Instance._npcHatchback = hatchback;
            Instance._npcSmallCar  = smallCar;
            Instance._npcTruck = truck;
            Instance._npcVan = van;
            Instance._casualPedestrian = casualPedestrian;
            Instance._elegantPedestrian = elegantPedestrian;
            Instance._obstacleLayerMask  = obstacleLM;
            Instance._groundLayerMask = groundLM;
            
            Instance._npcVehicleSpawner = new NpcVehicleSpawner(Instance._parentGameObject);
            
            NpcVehicleConfig vehicleConfig = NpcVehicleConfig.Default();
            vehicleConfig.Acceleration = 3.0f;
            vehicleConfig.Deceleration = 3.0f;
            Instance._npcVehicleSimulator = new NpcVehicleSimulator(vehicleConfig, 
                Instance._obstacleLayerMask, Instance._groundLayerMask, 10, Instance._egoCar.transform);
            
        }

        #region getters

        public static List<TrafficSimNpcVehicle> AllNPCVehicles => Instance._npcs;
        public static List<Tuple<NPCPedes, Pedestrian>> AllPedestrians => Instance._pedestrians;

        public static CustomSimulationManager Instance => manager ??= new CustomSimulationManager();

        public static TrafficLane[] GetAllTrafficLanes()
        {
            if (Instance._allTrafficLanes == null)
            {
                var trafficLanesParent = GameObject.Find("TrafficLanes");
                if (trafficLanesParent != null)
                    Instance._allTrafficLanes = trafficLanesParent.GetComponentsInChildren<TrafficLane>();
                else 
                    Debug.LogError("TrafficLanes not found");
            }

            return Instance._allTrafficLanes;
        }
        #endregion

        public void OnFixedUpdate()
        {
            UpdateDelayingNPCs();
            _npcVehicleSimulator.StepOnce(Time.fixedDeltaTime);
            
            foreach (var npc in _npcs)
            {
                npc.OnFixedUpdate();
            }

            // update pedestrian poses
            UpdatePedestrians();
        }

        public void OnUpdate()
        {
            foreach (var entry in _pedestrians)
            {
                entry.Item2.OnUpdate();
            }
        }

        /// <summary>
        /// Mainly perform 2 tasks:
        /// 1. Scan `delayingMoveNPCs`, i.e., the list of NPCs were spawned but not yet moved,
        ///    to make them move if they are ready to move.
        /// 2. Scan `delayingSpawnNPCs`, i.e., the list of NPCs waiting to be spawned,
        ///    to spawn them if they are ready to be spawned.
        /// </summary>
        private void UpdateDelayingNPCs()
        {
            List<TrafficSimNpcVehicle> removeAfter = new List<TrafficSimNpcVehicle>();
            foreach (var entry in _delayingMoveNPCs)
            {
                TrafficSimNpcVehicle npcVehicle = entry.Key;
                NPCCar npcCar = entry.Value.Item2;
                int waypointIndex = entry.Value.Item1;
                INPCSpawnDelay idelay = npcCar.SpawnDelayOption;

                if (idelay is NPCDelayTime delayTime)
                {
                    if (delayTime.DelayType == DelayKind.FROM_BEGINNING && Time.fixedTime >= delayTime.DelayAmount)
                    {
                        if (npcCar.Goal == null)
                        {
                            _npcVehicleSimulator.Register(npcVehicle, 
                                DynamicSimUtils.ParseLane(npcCar.InitialPosition.GetLane()), 
                                waypointIndex, 
                                npcCar.Config);
                        }
                        // else // npc has a goal
                        // {
                        //     _npcVehicleSimulator.Register(npcVehicle, waypointIndex,
                        //         npcCar.Goal,
                        //         npcCar.Config,
                        //         npcCar.VehicleType);
                        // }
                        removeAfter.Add(npcVehicle);
                    }
                }
            }
            foreach (var npc in removeAfter)
                _delayingMoveNPCs.Remove(npc);
        }
        
        private void UpdatePedestrians()
        {
            foreach (var entry in _pedestrians)
            {
                NPCPedes npcPedes = entry.Item1;
                var delayTime = npcPedes.Config?.Delay;
                if (delayTime == null ||
                    (delayTime.DelayType == DelayKind.FROM_BEGINNING &&
                     Time.fixedTime >= delayTime.DelayAmount))
                {
                    var newPosition = npcPedes.LastPosition +
                                      npcPedes.LastRotation * Vector3.forward * (npcPedes.Config.Speed * Time.fixedDeltaTime);
                    if (DynamicSimUtils.SignDistance(newPosition, npcPedes.CurrentWaypoint, npcPedes.LastRotation) > -0.02f)
                    {
                        npcPedes.LastPosition = newPosition;
                        // entry.Item2.SetPosition(npcPedes.LastPosition);
                    }
                    // update the waypoint
                    else
                    {
                        // if moving forward, and this is the last waypoint
                        if (!npcPedes.Backward && npcPedes.CurrentWaypointIndex == npcPedes.Waypoints.Count - 1)
                        {
                            // loop: turn backward
                            if (npcPedes.Config != null && npcPedes.Config.Loop)
                            {
                                npcPedes.Backward = true;
                                npcPedes.LastPosition = npcPedes.CurrentWaypoint;
                                npcPedes.CurrentWaypointIndex--;
                                npcPedes.LastRotation = Quaternion.LookRotation(npcPedes.CurrentWaypoint - npcPedes.LastPosition);
                                // entry.Item2.SetRotation(npcPedes.LastRotation);
                            }
                            // reached the goal, do nothing
                            else
                            {
                                
                            }
                        }
                        // if moving backward, and this is the first waypoint
                        else if (npcPedes.Backward && npcPedes.CurrentWaypointIndex == 0)
                        {
                            if (npcPedes.Config != null && npcPedes.Config.Loop)
                            {
                                npcPedes.Backward = false;
                                npcPedes.LastPosition = npcPedes.CurrentWaypoint;
                                npcPedes.CurrentWaypointIndex++;
                                npcPedes.LastRotation = Quaternion.LookRotation(npcPedes.CurrentWaypoint - npcPedes.LastPosition);
                                // entry.Item2.SetRotation(npcPedes.LastRotation);
                            }

                        }
                        // update rotation to match new waypoint
                        else
                        {
                            npcPedes.LastPosition = npcPedes.CurrentWaypoint;
                            if (npcPedes.Backward)
                                npcPedes.CurrentWaypointIndex--;
                            else
                                npcPedes.CurrentWaypointIndex++;
                            npcPedes.LastRotation = Quaternion.LookRotation(npcPedes.CurrentWaypoint - npcPedes.LastPosition);
                            // entry.Item2.SetRotation(npcPedes.LastRotation);
                        }
                    }
                    
                    entry.Item2.PoseInput = new Pose(npcPedes.LastPosition, npcPedes.LastRotation);
                    entry.Item2.OnFixedUpdate();
                }
            }
        }
        

        #region public functionalities
        
        // spawn an NPC and delay `delay.ActivateDelay` seconds to make it move
        public static TrafficSimNpcVehicle SpawnNPCAndDelayMovement(NPCCar npcCar)
        {
            if (npcCar.SpawnDelayOption == null || npcCar.SpawnDelayOption.ActionDelayed != DelayedAction.MOVING)
                throw new Exception("[DynamicSim]: Invalid NPCSpawnDelay parameter.");

            // spawn NPC
            TrafficSimNpcVehicle npc = SpawnNPC(npcCar.VehicleType, npcCar.InitialPosition, out int waypointIndex, npcCar.Name);
            if (npcCar.Config != null)
                npc.NpcVehicle.CustomConfig = npcCar.Config;

            Instance._delayingMoveNPCs.Add(npc,
                Tuple.Create(waypointIndex, npcCar));
            return npc;
        }
        
        // despawn NPC immediately
        public static bool DespawnNPC(TrafficSimNpcVehicle vehicle)
        {
            var internalState = Instance._npcVehicleSimulator.VehicleStates.FirstOrDefault(state =>
                state.Vehicle == vehicle);
            if (internalState != null)
                internalState.ShouldDespawn = true;
            UnityEngine.Object.DestroyImmediate(vehicle.gameObject);
            Instance._npcs.Remove(vehicle);
            return true;
        }
        
        public static void SpawnPedestrianAndDelayMovement(NPCPedes npcPedes, GameObject parent)
        {
            GameObject pedesGameObj = UnityEngine.Object.Instantiate(Instance.GetNPCPrefab(npcPedes.PedType),
                npcPedes.LastPosition,
                npcPedes.LastRotation);
            pedesGameObj.transform.parent = parent.transform;
            
            Pedestrian pedestrian = pedesGameObj.GetComponent<Pedestrian>();
            AllPedestrians.Add(Tuple.Create(npcPedes,pedestrian));
        }
                
        // despawn NPC pedestrian
        public static bool DespawnPedestrian(NPCPedes pedestrian)
        {
            var internalState = Instance._pedestrians.Find(entry => entry.Item1.Name == pedestrian.Name);
            if (internalState != null)
            {
                UnityEngine.Object.Destroy(internalState.Item2.gameObject);
                Instance._pedestrians.Remove(internalState);
                return true;
                // Debug.LogError($"[AWAnalysis] Could not find internal state of {vehicle}.");
            }
            return false;
        }
        
        #endregion
        

        #region Internal processing

        // spawn an NPC (static, no movement)
        private static TrafficSimNpcVehicle SpawnNPC(VehicleType vehicleType, IPosition spawnPosition, out int waypointIndex,
            string name = "")
        {
            // EnsureNonNullInstance(Manager());
            // calculate position
            TrafficLane spawnLane = DynamicSimUtils.ParseLane(spawnPosition.GetLane());
            Vector3 position = DynamicSimUtils.CalculatePosition(
                spawnLane, spawnPosition.GetOffset(), out waypointIndex);
            NpcVehicleSpawnPoint spawnPoint = new NpcVehicleSpawnPoint(spawnLane, position, waypointIndex);

            // spawn NPC
            TrafficSimNpcVehicle npcWrapper = Instance._npcVehicleSpawner.Spawn(Instance.GetNPCPrefab(vehicleType),
                SpawnIdGenerator.Generate(), spawnPoint, Quaternion.LookRotation(spawnPoint.Forward));
            if (name != "")
                npcWrapper.NpcVehicle.ScriptName = name;
            Instance._npcs.Add(npcWrapper);
            return npcWrapper;
        }
        
        
        private TrafficSimNpcVehicle GetNPCPrefab(VehicleType vehicleType)
        {
            switch (vehicleType)
            {
                case VehicleType.TAXI:
                    return _npcTaxi;
                case VehicleType.HATCHBACK:
                    return _npcHatchback;
                case VehicleType.SMALL_CAR:
                    return _npcSmallCar;
                case VehicleType.TRUCK:
                    return _npcTruck;
                case VehicleType.VAN:
                    return _npcVan;
                default:
                    Debug.LogWarning("[DynamicSim] Cannot detect the vehicle type `" + vehicleType + "`. " +
                                     "Use `taxi` as the default.");
                    return _npcTaxi;
            }
        }
        
        private GameObject GetNPCPrefab(PedesType pedestrianType)
        {
            switch (pedestrianType)
            {
                case PedesType.CASUAL:
                    return _casualPedestrian;
                case PedesType.ELEGANT:
                    return _elegantPedestrian;
                default:
                    Debug.LogWarning("[NPCSim] Cannot parse the pedestrian type `" + pedestrianType + "`. " +
                                     "Use `casual` pedestrian as the default.");
                    return _casualPedestrian;
            }
        }

        #endregion

        #region Dynamically Intervene actions

        /// <summary>
        /// 
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="targetSpeed"></param>
        /// <param name="acceleration"></param>
        /// <param name="deceleration"></param>
        /// <param name="isSpeedDefined"></param>
        /// <param name="isAccelerationDefined"></param>
        /// <param name="isDecelerationDefined"></param>
        /// <param name="followCustomWaypoints"> flag, 1 -> true, 0 -> false, -1 > skip </param>
        public static void ResetMotionProfileForNPC(ref TrafficSimNpcVehicle vehicle,
            float targetSpeed, float acceleration, float deceleration,
            bool isSpeedDefined, bool isAccelerationDefined, bool isDecelerationDefined,
            int followCustomWaypoints=-1)
        {
            foreach (var internalState in Instance._npcVehicleSimulator.VehicleStates)
            {
                if (internalState.Vehicle == vehicle)
                {
                    internalState.CustomConfig = ResetMotionProfileForNPCConfig(vehicle.NpcVehicle.CustomConfig,
                        targetSpeed, acceleration, deceleration,
                        isSpeedDefined, isAccelerationDefined, isDecelerationDefined, followCustomWaypoints);
                    break;
                }
            }

            vehicle.NpcVehicle.CustomConfig = ResetMotionProfileForNPCConfig(vehicle.NpcVehicle.CustomConfig,
                targetSpeed, acceleration, deceleration,
                isSpeedDefined, isAccelerationDefined, isDecelerationDefined, followCustomWaypoints);
            
            if (Instance._delayingMoveNPCs.ContainsKey(vehicle))
            {
                var waypointId = Instance._delayingMoveNPCs[vehicle].Item1;
                var npcCar = Instance._delayingMoveNPCs[vehicle].Item2;
                npcCar.Config = ResetMotionProfileForNPCConfig(npcCar.Config,
                    targetSpeed, acceleration, deceleration,
                    isSpeedDefined, isAccelerationDefined, isDecelerationDefined, followCustomWaypoints);
                Instance._delayingMoveNPCs[vehicle] = new Tuple<int, NPCCar>(waypointId, npcCar);
            }

        }
        
        public static NPCConfig ResetMotionProfileForNPCConfig(NPCConfig config,
            float targetSpeed, float acceleration, float deceleration,
            bool isSpeedDefined, bool isAccelerationDefined, bool isDecelerationDefined,
            int followCustomWaypoints=-1)
        {
            config ??= new NPCConfig();
            if (isSpeedDefined)
                config.TargetSpeed = targetSpeed;
            if (isAccelerationDefined)
            {
                config.Acceleration = acceleration;
                config.AggressiveDrive = true;
            }
            if (isDecelerationDefined)
                config.Deceleration = deceleration;
            if (followCustomWaypoints != -1)
                config.FollowCustomWaypoints = followCustomWaypoints == 1;
            return config;
        }
        
        
        /// <summary>
        /// reset delay to 0 for the $vehicle, i.e., making it move immediately
        /// </summary>
        /// <param name="vehicle"></param>
        public static void RemoveDelayFromNPC(TrafficSimNpcVehicle vehicle, float delay=0f)
        {
            var waypointId= Instance._delayingMoveNPCs[vehicle].Item1;
            var npcCar = Instance._delayingMoveNPCs[vehicle].Item2;
            // npcCar.SpawnDelayOption = NPCDelayTime.DelayMoveUntilEgoEngaged(delay);
            npcCar.SpawnDelayOption = NPCDelayTime.DelayMove(delay);
            Instance._delayingMoveNPCs[vehicle] = new Tuple<int, NPCCar>(waypointId, npcCar);
        }

        /// <summary>
        /// reset the lane of the $vehicle.
        /// Example use case: to reset the vehicle with a new virtual lane (e.g., constructed from a sequence of desired waypoints) 
        /// </summary>
        /// <param name="vehicle">must currently on $newLane</param>
        /// <param name="newLane"></param>
        public static void ResetLanePositionForNPC(TrafficSimNpcVehicle vehicle, TrafficLane newLane)
        {
            var npcCar = Instance._delayingMoveNPCs[vehicle].Item2;
            npcCar.InitialPosition = new LaneOffsetPosition(newLane.name, 0);
            Instance._delayingMoveNPCs[vehicle] = new Tuple<int, NPCCar>(1, npcCar);
        }
        
        public static bool DoesExistInDelayMoveNPCs(TrafficSimNpcVehicle vehicle)
        {
            return Instance._delayingMoveNPCs.ContainsKey(vehicle);
        }
        
        /// <summary>
        /// reset the current following lane for the given NPC
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="route"></param>
        public static void ResetNPCRoute(TrafficSimNpcVehicle vehicle, TrafficLane route)
        {
            var internalState = Instance._npcVehicleSimulator.VehicleStates.FirstOrDefault(state =>
                state.Vehicle == vehicle);
            if (internalState == null)
            {
                Debug.LogError($"Cannot find the NPC {vehicle.name}");
                return;
            }
            internalState.FollowingLanes = new List<TrafficLane>{route};
            internalState.WaypointIndex = 0;
        }
        
        public static void ResetPedestrianProfile(ref NPCPedes pedestrian, List<Vector3> waypoints, float speed,
            bool isSpeedDefined)
        {
            pedestrian.Waypoints = waypoints;
            if (isSpeedDefined)
                pedestrian.Config.Speed = speed;
            pedestrian.Config.Loop = true; // TODO: enable config from client
        }
        
        public static void RemoveDelayFromPedestrian(NPCPedes pedestrian, float delay=0f)
        {
            pedestrian.Config.Delay = NPCDelayTime.DelayMove(delay);
        }
        
        #endregion

    }
}
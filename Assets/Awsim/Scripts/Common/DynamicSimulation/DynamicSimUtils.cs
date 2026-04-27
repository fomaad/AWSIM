using System;
using System.Collections.Generic;
using System.Linq;
using Awsim.Common.AWSIM_Script;
using Awsim.Usecase.DynamicSimulation;
using Awsim.Usecase.TrafficSimulation;
using UnityEngine;

namespace Awsim.Common.DynamicCommand
{
    public class DynamicSimUtils
    {
        public const string TRAFFIC_LANE_STR = "trafficlane";

        public static TrafficLane LaneAtPosition(Vector3 position, out int waypointId, 
            out float laneOffset, float tolerance = 0.1f)
        {
            return LaneAtPosition(position, CustomSimulationManager.GetAllTrafficLanes(), 
                                    out waypointId, out laneOffset, tolerance);
        }

        public static TrafficLane LaneAtPosition(Vector3 position, TrafficLane[] allTrafficLanes,
            out int waypointId, out float laneOffset, float tolerance = 0.1f)
        {
            bool checkElevation = true;
            if (Mathf.Approximately(position.z,0))
            {
                Debug.LogWarning($"[AWAnalysis] The elevation is likely not provided for point {position}." +
                                 $"The interpreted point in map might not be the expected one.");
                checkElevation = false;
            }
            List<TrafficLane> candidates = new();
            List<int> candidateWpIDs = new();
            List<float> candidateLaneOffsets = new();
            foreach (var lane in allTrafficLanes)
            {
                if (IsPointOnCenterLane(position, lane, out int wpID, out float offset, tolerance, checkElevation))
                {
                    candidates.Add(lane);
                    candidateWpIDs.Add(wpID);
                    candidateLaneOffsets.Add(offset);
                }
            }
            if (candidates.Count == 0)
            {
                waypointId = -1;
                laneOffset = -1;
                return null;
            }

            if (candidates.Count > 1)
            {
                int resultId = 0;
                float maxZ = candidates[0].Waypoints[0].z;
                string laneName = candidates[0].name;
                for (int i = 1; i < candidates.Count; i++)
                {
                    laneName += ", " + candidates[i].name;
                    if (candidates[i].Waypoints[0].z > maxZ)
                    {
                        maxZ = candidates[i].Waypoints[0].z;
                        resultId = i;
                    }
                }
                Debug.LogWarning($"[AWAnalysis] Found {candidates.Count} ({laneName}) " +
                                 $"possible traffic lanes for position {position}." +
                                 $"By default, the highest-elevation lane was selected.");
                waypointId = candidateWpIDs[resultId];
                laneOffset = candidateLaneOffsets[resultId];
                return candidates[resultId];
            }

            waypointId = candidateWpIDs[0];
            laneOffset = candidateLaneOffsets[0];
            return candidates[0];
        }
        
        /// <summary>
        /// check if a given point is on the lane
        /// </summary>
        /// <param name="position"></param>
        /// <param name="lane"></param>
        /// <param name="waypointId">the waypoint index of the starting point of the segment on which the point is located.</param>
        /// <param name="laneOffset">the lane offset of the point from the lane starting point</param>
        /// <param name="tolerance"></param>
        /// <returns> True if the point is on the lane center line, False otherwise.</returns>
        public static bool IsPointOnCenterLane(Vector3 position, TrafficLane lane, 
            out int waypointId, out float laneOffset, float tolerance = 0.1f, bool checkElevation = true)
        {
            Vector2 pos2D = new Vector2(position.x, position.z);
            for(int i = 0; i < lane.Waypoints.Length - 1; i++)
            {
                Vector2 start = new Vector2(lane.Waypoints[i].x, lane.Waypoints[i].z);
                Vector2 end = new Vector2(lane.Waypoints[i + 1].x, lane.Waypoints[i + 1].z);
                if (DistancePointToLineSegment(pos2D, start, end) <= tolerance)
                {
                    if (checkElevation)
                    {
                        if (Math.Abs(position.y - lane.Waypoints[i].y) < 3f)
                        {
                            waypointId = i;
                            laneOffset = lane.DistanceUpToWaypoint(i) + Vector3.Distance(lane.Waypoints[i], position);
                            return true;
                        }
                    }
                    else
                    {
                        waypointId = i;
                        laneOffset = lane.DistanceUpToWaypoint(i) + Vector3.Distance(lane.Waypoints[i], position);
                        return true;
                    }
                }
            }
            waypointId = -1;
            laneOffset = -1;
            return false;
        }

        /// <summary>
        /// Calculates the shortest distance from a point to a line segment in 2D.
        /// If the projection falls outside the segment, return infinity
        /// </summary>
        /// <param name="point">The point to measure the distance from (Vector2).</param>
        /// <param name="segmentStart">The starting point of the line segment (Vector2).</param>
        /// <param name="segmentEnd">The ending point of the line segment (Vector2).</param>
        /// <returns>The shortest distance from the point to the line segment.</returns>
        public static float DistancePointToLineSegment(Vector2 point, Vector2 segmentStart, Vector2 segmentEnd)
        {
            // Vector from segmentStart to point
            Vector2 AP = point - segmentStart;

            // Vector representing the line segment
            Vector2 AB = segmentEnd - segmentStart;

            // Calculate the squared length of the segment (for normalization and avoiding sqrt early)
            float lengthSq = AB.sqrMagnitude;

            // If the segment has zero length, it's a point. Return distance to that point.
            if (lengthSq == 0.0f)
            {
                return Vector2.Distance(point, segmentStart);
            }

            // Calculate the projection parameter (t)
            // t = (AP dot AB) / |AB|^2
            float t = Vector2.Dot(AP, AB) / lengthSq;

            if (t < 0.0f || t > 1.0f)
                return float.MaxValue;

            // Calculate the closest point on the segment
            Vector2 closestPoint = segmentStart + t * AB;

            // Return the distance from the original point to the closest point on the segment
            return Vector2.Distance(point, closestPoint);
        }
        
        /// <summary>
        /// return the distance between two points, ignore the Y component
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <returns></returns>
        public static float DistanceIgnoreYAxis(Vector3 point1, Vector3 point2)
        {
            return MagnitudeIgnoreYAxis(point1 - point2);
        }

        public static float MagnitudeIgnoreYAxis(Vector3 point1)
        {
            point1.y = 0f;
            return Vector3.Magnitude(point1);
        }
        
        // parse traffic name from a given name
        public static TrafficLane ParseLane(string laneName)
        {
            if (CustomSimulationManager.GetAllTrafficLanes() != null)
            {
                var candidate = CustomSimulationManager.GetAllTrafficLanes().FirstOrDefault(l => l.name == laneName);
                if (candidate != null)
                    return candidate;
            }
            GameObject obj = GameObject.Find(laneName);
            if (obj == null)
                throw new InvalidScriptException("[DynamicSim] Cannot find traffic lane with name: " + laneName);
            return obj.GetComponent<TrafficLane>();
        }
        
        // given "TrafficLane.231", parse to get 231
        public static int ParseLaneIndex(string laneName)
        {
            laneName = laneName.ToLower();
            if (laneName.StartsWith(TRAFFIC_LANE_STR))
            {
                laneName = laneName.Substring(TRAFFIC_LANE_STR.Length);
                if (laneName.StartsWith("."))
                    laneName = laneName.Substring(1);
                if (Int32.TryParse(laneName, out int index))
                {
                    return index;
                }
            }
            return -1;
        }

        /// <summary>
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="distance"></param>
        /// <param name="waypointIndex"></param>
        /// <returns>vector3 representing the point on lane $lane, far $distance m from the starting point of $lane</returns>
        public static Vector3 CalculatePosition(TrafficLane lane, float distance, out int waypointIndex)
        {
            float remainDistance = distance;
            for (int j = 0; j < lane.Waypoints.Length - 1; j++)
            {
                Vector3 startPoint = lane.Waypoints[j];
                Vector3 endPoint = lane.Waypoints[j + 1];
                if (DistanceIgnoreYAxis(startPoint, endPoint) < remainDistance)
                {
                    remainDistance -= DistanceIgnoreYAxis(startPoint, endPoint);
                    continue;
                }
                else
                {
                    // if (remainDistance == 0)
                    // {
                    //     waypointIndex = j;
                    //     return startPoint;
                    // }
                    // else
                    {
                        Vector3 temp = (endPoint - startPoint).normalized;
                        waypointIndex = j + 1;
                        return startPoint + (temp * remainDistance);
                    }
                }
            }
            Debug.LogWarning("The given distance " + distance + " is larger than the total lane length." +
                             " The end point of the lane is used.");
            waypointIndex = lane.Waypoints.Length - 1;
            return lane.Waypoints[waypointIndex];
        }
        
        public static float SignDistance(Vector3 root, Vector3 position, Quaternion direction)
        {
            var distance = Vector3.Distance(position, root);
            return Vector3.Dot(position - root, direction * Vector3.forward) > 0 ? distance : -distance;
        }
    }
}
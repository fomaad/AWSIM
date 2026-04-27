using Awsim.Common.AWSIM_Script;
using UnityEngine;

namespace Awsim.Usecase.DynamicSimulation
{
    public class ScenarioParser
    {
        public static VehicleType ParseVehicleType(string vehicleType)
        {
            switch (vehicleType.ToLower())
            {
                case "taxi":
                    return VehicleType.TAXI;
                case "hatchback":
                    return VehicleType.HATCHBACK;
                case "small-car":
                case "smallcar":
                    return VehicleType.SMALL_CAR;
                case "truck":
                    return VehicleType.TRUCK;
                case "van":
                    return VehicleType.VAN;
                default:
                    throw new InvalidScriptException("Cannot parse the vehicle type: " + vehicleType);
            }
        }
        
        public static PedesType ParseHumanType(string humanTypeStr)
        {
            switch (humanTypeStr.ToLower())
            {
                case "casual":
                    return PedesType.CASUAL;
                case "elegant":
                    return PedesType.ELEGANT;
                default:
                    Debug.LogError("Cannot parse the pedestrian type: " + humanTypeStr +
                                   ". Use the casual human as default.");
                    return PedesType.CASUAL;
            }
        }
    }
}
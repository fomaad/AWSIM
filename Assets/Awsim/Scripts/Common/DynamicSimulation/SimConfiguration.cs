namespace Awsim.Common.DynamicCommand
{
    [System.Serializable]
    public class SimConfiguration
    {
        public bool applyAngularGaussianNoise = true;
        public bool applyDistanceGaussianNoise = true;
        public bool applyVelocityDistortion = true;
        
        public float angularNoiseStDev = 1; // Degrees
        public float angularNoiseMean = 0; // Degrees

        public float distanceNoiseStDevBase = 0.05f; // Meters
        public float distanceNoiseStDevRisePerMeter = 0.01f; // Meters
        public float distanceNoiseMean = 0; // Meters
        
    }
}
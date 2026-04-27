namespace Awsim.Common.AWSIM_Script
{
    public interface INPCSpawnDelay
    {
        public DelayedAction ActionDelayed { get; set; }
    }
    
    public enum DelayedAction
    {
        SPAWNING, // spawning NPC is delayed
        MOVING    // spawn NPC as it is, but delay its movement
    }
}
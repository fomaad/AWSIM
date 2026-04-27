using System.Collections.Generic;

namespace Awsim.Common.TraceObjects
{
    public class TraceObject : TraceObjectWithoutState
    {
        public List<StateObject> states;
    }
}
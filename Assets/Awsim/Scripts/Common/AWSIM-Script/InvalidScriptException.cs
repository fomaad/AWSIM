using System;

namespace Awsim.Common.AWSIM_Script
{
    public class InvalidScriptException : Exception
    {
        public InvalidScriptException()
        {
        }
        public InvalidScriptException(string message)
            : base(message)
        {
        }
    }
}
using System.Text;
using System.Text.Json;
using System.Diagnostics;

namespace JakaAPI
{
    public partial class JakaRobot
    {
        public delegate void DebugInformation(string message);
        private event DebugInformation? FunctionFeedback;

        //private readonly int _commandDelay = 50;
        public readonly int _commandDelay = 2;

        private string _lastSendingResponse = string.Empty;

        // Currently unused
        private string _lastListeningResponse = string.Empty;

        public void DebugSubscribe(DebugInformation debug)
        {
            FunctionFeedback += debug;
        }

        public void DebugUnSubscribe(DebugInformation debug)
        {
            FunctionFeedback -= debug;
        }

        protected void OnPostCommand()
        {
            _lastSendingResponse = ReadSendingResponse();

            Thread.Sleep(_commandDelay);
            WaitComplete();
            ReadSendingResponse();
            Thread.Sleep(_commandDelay);

            var outdata = JsonSerializer.Deserialize<Dictionary<string,string>>(_lastSendingResponse);//convert output into dictionary of string-string
            char errCode = outdata["errorCode"][0];//get value of errorCode

            if (errCode != '0')
            {
                Console.WriteLine(_lastSendingResponse);
            }
                

            try
            {
                FunctionFeedback?.Invoke(_lastSendingResponse);
            }
            catch(Exception ex) 
            { 
                Console.WriteLine(ex.ToString());
            }

                    
        }

        private string ReadSendingResponse()
        {
            byte[] responseBytes = new byte[2048];
            int numBytesReceived = _socketSending.Receive(responseBytes);
            return Encoding.ASCII.GetString(responseBytes, 0, numBytesReceived);
        }

        public string ReadListeningResponse()
        {
            byte[] responseBytes = new byte[2048];
            int numBytesReceived = _socketListening.Receive(responseBytes);
            return Encoding.ASCII.GetString(responseBytes, 0, numBytesReceived);
        }

        private void ExactDelay(int millisec)
        {
            Stopwatch stopWatch = new Stopwatch();
            long frequency = Stopwatch.Frequency;
            long tickstowait = (millisec * 1000L) / frequency;
            stopWatch.Start();
            while (stopWatch.ElapsedTicks < tickstowait)
            {
                ; // do nothing
            }
            stopWatch.Stop();
        }

        public string GetLastSendingResponse() => _lastSendingResponse;

        public string GetLastListeningResponse() => _lastListeningResponse;
    }
}

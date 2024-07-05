using JakaAPI.Types;
using JakaAPI.Types.Math;
using JakaAPI;
using PainterArm.Calibration;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace PainterArm
{
    /// <summary>
    /// Jaka Robot based implementation of painting robot
    /// </summary>
    public class JakaPainter : JakaRobot
    {
        private CoordinateSystem2D? _canvasCoordinateSystem;
        private double _currentX, _currentY, _currentHeight;

        private LocationDictionary _brushesLocations;
        private CartesianPosition _dryerLocation;

        private const double _brushLength = 157.5;
        private const double _needleLength = 157.5;

        private const double _readyheight = 10.5;

        private CartesianPosition _initialPosition = new(-294, 276, 122, 180, 0, 0);//position in the middle of canvas, 13 mm above it
        private JointsPosition _initialJointPosition = new(121.291, 62.202, -100.304, 128.102, 90.000, 31.291);


        public int CurrentBrush { get; private set; }

        private readonly Dictionary<int, int> _brushesDI = new()
            {
                { 0, 9 }, // Change to actual DIs
                { 1, 10 },
                { 2, 11 },
                { 3, 12 },
                { 4, 13 },
                { 5, 14 },
            };

        /// <summary>
        /// Indicates whether the grip of the robot is being in grap state
        /// </summary>
        private bool _grip;

        public AbstractCalibrationBehavior CanvasCalibrationBehavior, BrushesCalibrationBehavior, DryerCalibrationBehavior;

        public JakaPainter(string domain, int portSending = 10001, int portListening = 10000)
            : base(domain, portSending, portListening)
        {
            _brushesLocations = new LocationDictionary();
            _grip = true;
            _currentX = 0;
            _currentY = 0;
            _currentHeight = 0;

            CurrentBrush = -1;
            SetDOState(0, 0, _grip);
            CanvasCalibrationBehavior = new NeedleManualThreePointCalibration(this, _needleLength);
            DryerCalibrationBehavior = new ManualOnePointCalibration(this);
            BrushesCalibrationBehavior = new ManualOnePointCalibration(this);
        }

        /// <summary>
        /// Canvas calibration based on existing <see cref="CoordinateSystem2D"/>
        /// </summary>
        /// <param name="cs">Existing coordinate system to be used as canvas</param>
        public void CalibrateCanvas(CoordinateSystem2D cs)
        {
            _canvasCoordinateSystem = cs;
            UpdateRobotPosition();
        }

        /// <summary>
        /// Canvas calibration based on existing <see cref="CoordinateSystem2D"/>
        /// </summary>
        public void UpdateRobotPosition()
        {
            //get current points
            /*Point p = GetRobotData().ArmCartesianPosition.Point;
            Point pcanvas = _canvasCoordinateSystem!.WorldPointToCanvasPoint(p.X, p.Y, p.Z);
            _currentX = pcanvas.X;
            _currentY = pcanvas.Y;
            _currentHeight = pcanvas.Z;*/
        }



        /// <summary>
        /// Brushes calibration
        /// </summary>
        public void CalibrateBrushes(LocationDictionary locations) => _brushesLocations = locations;

        /// <summary>
        /// Dryer calibration
        /// </summary>
        public void CalibrateDryer(CartesianPosition location) => _dryerLocation = location;

        /// <summary>
        /// Draw line with canvas 2D coordinates
        /// </summary>
        /// <param name="x">X-axis coordinates in millimeters</param>
        /// <param name="y">Y-axis coordinates in millimeters</param>
        /// /// <param name="deltaz">Z-axis offset in millimeters, negative or positive</param>
        public void DrawLine(double x, double y, double deltaz = 0)
        {
            //replacing MoveLinear(new CartesianPosition(point3d, _canvasCoordinateSystem.RPYParameters), 100, 25, MovementType.Absolute);
            double t = 4e-3; //repeating time
            double v = 20; //robot speed

            double d = v * t; //distance covered during repeating time
            double D = Math.Sqrt((x - _currentX) * (x - _currentX) + (y - _currentY)*(y - _currentY) + deltaz* deltaz);
            int n = (int)Math.Floor(D/d);
            double dx = (x - _currentX) / n;
            double dy = (y - _currentY) / n;
            double dz = deltaz / n;

            double Z0 = _currentHeight;


            ServoMove(1); //enter servo mode
            Point newpoint;
            for (int i = 0; i< n; i++)
            {
                _currentX += dx;
                _currentY += dy;
                _currentHeight += dz;
                newpoint = _canvasCoordinateSystem!.CanvasPointToWorldPoint(_currentX, _currentY, _currentHeight); //new current point
                CartesianMoveControl(new CartesianPosition(newpoint, _canvasCoordinateSystem.RPYParameters), MovementType.Absolute);
                Thread.Sleep((int)(t * 1000));
            }

            _currentX = x;
            _currentY = y;
            _currentHeight = Z0 + deltaz;
            newpoint = _canvasCoordinateSystem!.CanvasPointToWorldPoint(_currentX, _currentY, _currentHeight); //go to final current point
            CartesianMoveControl(new CartesianPosition(newpoint, _canvasCoordinateSystem.RPYParameters), MovementType.Absolute);

            ServoMove(0); //exit servo mode
        }

        /// <summary>
        /// Brush cartesian move in 3D coordinates
        /// </summary>
        /// <param name="coordinates"> X, Y, dZ coordinates in millimeters</param>
        public void BrushMove(double[] coordinates)
        {
            int scalef = 40; //scaling factor
            int ctr = 1;

            //go to initial position for correct program execution
            //MoveLinear(_initialPosition, 20, 20, MovementType.Absolute);
            //JointMove(_initialJointPosition, 5, 10, MovementType.Absolute);
            //Thread.Sleep(1500);
            //UpdateRobotPosition();

            double newheight = _currentHeight;
            int N = coordinates.Length;
            int npts = (int)Math.Floor(N / 3.0); //number of points in the stroke
            ModifyVariables(5500, "npts", npts);
            //point j
            for (int j = 0; j < N; j += 3)
            {
                double x = coordinates[j];
                double y = coordinates[j + 1];
                double deltaz = coordinates[j + 2];
                //Point newpoint = _canvasCoordinateSystem!.CanvasPointToWorldPoint(x, y, newheight + deltaz); //new current point

                /*int dx = (int)(scalef * newpoint.X);
                int dy = (int)(scalef * newpoint.Y);
                int dz = (int)(scalef * newpoint.Z);*/

                int dx = (int)(x);
                int dy = (int)(y);
                int dz = (int)(deltaz);

                ModifyVariables(5501 + j, string.Format("x{0}", ctr), dx);
                ModifyVariables(5502 + j, string.Format("y{0}", ctr), dy);
                ModifyVariables(5503 + j, string.Format("z{0}", ctr), dz);

                ctr++;
                newheight += deltaz;
            }


            //LoadProgram("jakastroke.ngc");
            LoadProgram("stroke.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}",GetLoadedProgramName());
            PlayProgram();
            //UpdateRobotPosition();
            while (GetProgramStatus() != "idle")
                Thread.Sleep(100);
        }

        /// <summary>
        /// Brush cartesian move using bezier in 3D coordinates
        /// </summary>
        /// <param name="coordinates"> X, Y, dZ coordinates in millimeters</param>
        public void BezierMove(double[] coordinates)
        {
            int scalef = 40; //scaling factor
            int ctr = 1;

            double newheight = _currentHeight;
            int N = coordinates.Length;
            int npts = (int)Math.Floor(N / 3.0); //number of points in the stroke
            ModifyVariables(5500, "npts", npts);
            //point j
            for (int j = 0; j < N; j += 3)
            {
                double x = coordinates[j];
                double y = coordinates[j + 1];
                double deltaz = coordinates[j + 2];

                int dx = (int)(x);
                int dy = (int)(y);
                int dz = (int)(deltaz);

                ModifyVariables(5501 + j, string.Format("x{0}", ctr), dx);
                ModifyVariables(5502 + j, string.Format("y{0}", ctr), dy);
                ModifyVariables(5503 + j, string.Format("z{0}", ctr), dz);

                ctr++;
                newheight += deltaz;
            }

            LoadProgram("strokebezier.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            //UpdateRobotPosition();
            while (GetProgramStatus() != "idle")
                Thread.Sleep(100);
        }

        /// <summary>
        /// Brush cartesian move in 3D coordinates
        /// </summary>
        /// <param name="coordinates"> X, Y, dZ coordinates in millimeters</param>
        public void TakePaint(double[] coordinates)
        {
            int scalef = 40; //scaling factor
            int dx = (int)(scalef * coordinates[0]);
            int dy = (int)(scalef * coordinates[1]);

            //go to initial position above canvas for correct program execution
            //MoveLinear(_initialPosition, 250, 100, MovementType.Absolute);
            //UpdateRobotPosition();

            //write position of a can with paint, shifts in 1/40 mm using both axes in a LCS, centered with bottom left can
            ModifyVariables(5519, "dxpaint", dx);
            ModifyVariables(5520, "dypaint", dy);
            //each can stands 1800 units from another (45 mm)

            //load a program and play

            LoadProgram("takepaint.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            while (GetProgramStatus() != "idle")
                Thread.Sleep(100);

            //a brush should return to the initial position above canvas
        }


        /// <summary>
        /// Move robot away and take a photo
        /// </summary>
        public void TakeaPhoto()
        {
            LoadProgram("takeaphoto.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            while (GetProgramStatus() != "idle")
                Thread.Sleep(100);
        }

        public void PlaceBrush()
        {
            LoadProgram("placebrush.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            while (GetProgramStatus() != "idle")
                Thread.Sleep(100);
        }

        public void PlaceWasher()
        {
            LoadProgram("placewasher.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            Thread.Sleep(1500); //17 sec for the command
            while (GetProgramStatus() != "idle")
            {
                Console.WriteLine("Program is running!");
                Thread.Sleep(1000);
            }
        }

        public void TakeWasher()
        {
            LoadProgram("takewasher.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            Thread.Sleep(500); //15 sec for the command
            while (GetProgramStatus() != "idle")
            {
                Console.WriteLine("Program is running!");
                Thread.Sleep(1000);
            }
        }

        public void PlaceDrier()
        {
            LoadProgram("placedryer.ngc");
            Thread.Sleep(10);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            while (GetProgramStatus() != "idle")
            {
                Console.WriteLine("Program is running!");
                Thread.Sleep(1000);
            }
        }

        public void TakeDrier()
        {
            LoadProgram("takedryer.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();
            Thread.Sleep(1500); //17 sec for the command
            while (GetProgramStatus() != "idle")
            {
                Console.WriteLine("Program is running!");
                Thread.Sleep(1000);
            }
        }

        public async void Delay(double[] timetowait)
        {
            //Thread.Sleep((int)timetowait[0]*1000);

            IPEndPoint ipEndPoint = new(IPAddress.Parse("192.168.1.102"), 1500);
            Socket listener = new Socket(ipEndPoint.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
            listener.Bind(ipEndPoint);
            listener.Listen(100);

            LoadProgram("socket1.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram();

            //Thread.Sleep(1000);

            byte[] messageBytes = BitConverter.GetBytes(1);

            var handler = await listener.AcceptAsync();
            await handler.SendAsync(messageBytes, 0);

           

            var buffer = new byte[1_024];
            var received = await handler.ReceiveAsync(buffer, SocketFlags.None);

            var response = Encoding.UTF8.GetString(buffer, 0, received);

            Console.WriteLine($"Socket client received acknowledgment: \"{response}\"");

            

            listener.Close();
        }

        /// <summary>
        /// Brush cartesian move in 3D coordinates with socket communication
        /// </summary>
        /// <param name="coordinates"> X, Y, dZ coordinates in millimeters</param>
        public void BrushSocket(double[] coordinates)
        {
            int N = coordinates.Length;
            int npts = (int)Math.Floor(N / 3.0); //number of points in the stroke

            const int datalength = 43;
            //int[] coords = new int[datalength];
            float[] coords = new float[datalength];

            coords[0] = npts;
            for (int j = 0; j < N; j += 3)
            {
                double x = coordinates[j];
                double y = coordinates[j + 1];
                double z = coordinates[j + 2];

                float dx = (float)(x);
                float dy = (float)(y);
                float dz = (float)(z);

                coords[j + 1] = dx;
                coords[j + 2] = dy;
                coords[j + 3] = dz;
            }


            IPEndPoint ipEndPoint = new(IPAddress.Parse("192.168.1.102"), 1500);
            Socket listener = new Socket(ipEndPoint.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
            listener.Bind(ipEndPoint);
            listener.Listen(1);


            byte[] messageBytes;
            String s = "[";
            for (int j = 0; j < datalength; j++)
            {
                s += Convert.ToString(coords[j]);
                if (j < datalength - 1)
                    s += ",";
            }
            s +="]";

            LoadProgram("socket1.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram(false);

            //var handler = await listener.AcceptAsync();
            var handler = listener.Accept();

            var buffer = new byte[256];
            //var received = await handler.ReceiveAsync(buffer, SocketFlags.None);
            var received = handler.Receive(buffer, SocketFlags.None);
            var response = Encoding.UTF8.GetString(buffer, 0, received);
            Console.WriteLine($"Socket client received acknowledgment: \"{response}\"");


            messageBytes = System.Text.Encoding.ASCII.GetBytes(s);
            //messageBytes = Encoding.ASCII.GetBytes("<N><1>");
            //await handler.SendAsync(messageBytes, 0);
            //messageBytes = BitConverter.GetBytes(1);

            handler.Send(messageBytes, 0);

            //received = await handler.ReceiveAsync(buffer, SocketFlags.None);
            received = handler.Receive(buffer, SocketFlags.None);
            response = Encoding.UTF8.GetString(buffer, 0, received);
            Console.WriteLine($"Socket client received acknowledgment: \"{response}\"");

            listener.Close();

            OnPostCommand();
            while (GetProgramStatus() != "idle")
                Thread.Sleep(100);
        }


        /// <summary>
        /// Brush cartesian move in 3D coordinates with socket communication
        /// </summary>
        /// <param name="coordinates"> X, Y, dZ coordinates in millimeters</param>
        public void BrushSocket2(double[] coordinates)
        {
            //no more than 42 points in the stroke
            //determine amounts of data to be sent
            int N = coordinates.Length;
            int npts = (int)Math.Floor(N / 3.0); //number of points in the stroke

            const int datalength = 42;
            float[] coords = new float[datalength];
            (int ncycles, int remcycles) = Math.DivRem(N, datalength);

            //start socket communication
            IPEndPoint ipEndPoint = new(IPAddress.Parse("192.168.1.102"), 1500);
            Socket listener = new Socket(ipEndPoint.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
            listener.Bind(ipEndPoint);
            listener.Listen(1);

            //load program on robot controller
            LoadProgram("socket3.ngc");
            Thread.Sleep(100);
            Console.WriteLine("Program is loaded: {0}", GetLoadedProgramName());
            PlayProgram(false);

            var handler = listener.Accept();

            //receive message about receiving data
            var buffer = new byte[256];
            var received = handler.Receive(buffer, SocketFlags.None);
            var response = Encoding.UTF8.GetString(buffer, 0, received);
            Console.WriteLine($"Socket client received acknowledgment: \"{response}\"");

            //send message with npts
            byte[] messageBytes;
            String str = "<N><" + Convert.ToString(N) + ">";
            messageBytes = System.Text.Encoding.ASCII.GetBytes(str);
            handler.Send(messageBytes, 0);
            Console.WriteLine(str);

            //received = await handler.ReceiveAsync(buffer, SocketFlags.None);
            received = handler.Receive(buffer, SocketFlags.None);
            response = Encoding.UTF8.GetString(buffer, 0, received);
            Console.WriteLine($"Socket client received acknowledgment: \"{response}\"");
            Thread.Sleep(100);


            for (int i = 0; i < ncycles + 1; i++)
            {
                int niter = datalength;
                if (i == ncycles)
                    niter = remcycles;

                for (int j = 0; j < niter; j += 3)
                {
                    double x = coordinates[datalength * i + j];
                    double y = coordinates[datalength * i + j + 1];
                    double z = coordinates[datalength * i + j + 2];

                    float dx = (float)(x);
                    float dy = (float)(y);
                    float dz = (float)(z);

                    coords[j] = dx;
                    coords[j + 1] = dy;
                    coords[j + 2] = dz;
                }
                
                String s = "[";
                for (int j = 0; j < datalength; j++)
                {
                    s += Convert.ToString(coords[j]);
                    if (j < datalength - 1)
                        s += ",";
                }
                s += "]";

                //received = await handler.ReceiveAsync(buffer, SocketFlags.None);
                received = handler.Receive(buffer, SocketFlags.None);
                response = Encoding.UTF8.GetString(buffer, 0, received);
                Console.WriteLine($"Socket client received acknowledgment: \"{response}\"");

                messageBytes = System.Text.Encoding.ASCII.GetBytes(s);
                handler.Send(messageBytes, 0);

                received = handler.Receive(buffer, SocketFlags.None);
                response = Encoding.UTF8.GetString(buffer, 0, received);
                Console.WriteLine($"Socket client received acknowledgment: \"{response}\"");

                //Thread.Sleep(100);
            }
            //stop communication
            listener.Close();

            //let the robot perform the task
            OnPostCommand();
            while (GetProgramStatus() != "idle")
                Thread.Sleep(100);
        }


        // Raw method, will be implemented soon
        public void MixWater()
        {
            Console.WriteLine("Water vortex start...");
            Thread.Sleep(1000);
            Console.WriteLine("Water vortex end...");
        }

        /// <summary>
        /// Move the brush perpendicular to the canvas
        /// </summary>
        /// <param name="height">Z-axis offset</param>
        /// <param name="movementType">Brush movement type, absolute or relative</param>
        public void BrushOrthogonalMove(double height, MovementType movementType)
        {
            _currentHeight = (movementType == MovementType.Relative) ? _currentHeight + height : height;
            Point point3d = _canvasCoordinateSystem!.CanvasPointToWorldPoint(_currentX, _currentY, _currentHeight);

            MoveLinear(new CartesianPosition(point3d, _canvasCoordinateSystem.RPYParameters), 100, 100, MovementType.Absolute);
        }

        /// <summary>
        /// Returns current held brush to the stand
        /// </summary>
        public void ReturnCurrentBrush()
        {
            CartesianPosition brushPosition = _brushesLocations[CurrentBrush];
            Point brushPoint = brushPosition.Point;
            Point upperPoint = new(brushPoint.X, brushPoint.Y, brushPoint.Z + _brushLength);
            RPYRotation orthogonalRPY = brushPosition.Rpymatrix;

            // Move to position above the brush
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            // Move to the brush on stand
            MoveLinear(new CartesianPosition(brushPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            GripOff();

            // Move to position above the stand again
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            CurrentBrush = -1;
        }

        /// <summary>
        /// Returns current held brush to the stand
        /// </summary>
        public void PickNewBrush(int num)
        {
            CurrentBrush = num;
            CartesianPosition brushPosition = _brushesLocations[num];
            Point brushPoint = brushPosition.Point;
            Point upperPoint = new Point(brushPoint.X, brushPoint.Y, brushPoint.Z + _brushLength);
            RPYRotation orthogonalRPY = brushPosition.Rpymatrix;

            // Move to position above the brush
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            GripOff();

            // Move to the brush on stand
            MoveLinear(new CartesianPosition(brushPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            GripOn();

            // Move to position above the stand again
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);
        }

        // Dunk current brush it the palette color
        public void DunkBrushInColor(CartesianPosition colorPosition)
        {
            Point colorPoint = colorPosition.Point;
            Point upperPoint = new Point(colorPoint.X, colorPoint.Y, colorPoint.Z + _brushLength);
            RPYRotation orthogonalRPY = colorPosition.Rpymatrix;

            // Move to position above the palete
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            // Move to color on palette
            MoveLinear(new CartesianPosition(colorPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            // Move to position above the palete again
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);
        }

        public void DryCurrentBrush()
        {
            Point dryerPoint = _dryerLocation.Point;
            Point upperPoint = new Point(dryerPoint.X, dryerPoint.Y, dryerPoint.Z + _brushLength);
            RPYRotation orthogonalRPY = _dryerLocation.Rpymatrix;

            // Move to position above the dryer
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            // Move to dryer
            MoveLinear(new CartesianPosition(dryerPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            int rotationCount = 3;
            for (int i = 0; i < rotationCount; i++)
            {
                double c = Math.Pow(-1, i);
                JointMove(new JointsPosition(0, 0, 0, 0, 0, c * 30), 100, 100, MovementType.Relative);
            }

            // Move to position above the dryer again
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);
        }

        /// <summary>
        /// Sets the state of the grip to <b>ON</b>
        /// </summary>
        public void GripOn()
        {
            _grip = true;
            SetDOState(0, 0, _grip);
        }

        /// <summary>
        /// Sets the state of the grip to <b>OFF</b>
        /// </summary>
        public void GripOff()
        {
            _grip = false;
            SetDOState(0, 0, _grip);
        }

        /// <summary>
        /// Toggles the state of the grip
        /// </summary>
        [Obsolete("Not recommended to use, consider using explicit methods GripOn and GripOff")]
        public void ToggleGrip()
        {
            _grip = !_grip;
            SetDOState(0, 0, _grip);
        }

        /// <summary>
        /// Getting brush slot state based on Hall sensor:<br/>
        /// - <i>High voltage</i> means <i>no magnetic field</i>, a.k.a. no brush in the slot<br/>
        /// - <i>Low voltage</i> means <i>magnetic field presence</i>, a.k.a. brush is in the slot 
        /// </summary>
        /// <param name="brushNum"></param>
        /// <returns><see cref="BrushSlotState.EMPTY"/> if input contains high voltage signal, <see cref="BrushSlotState.OCCUPIED"/> otherwise</returns>
        public BrushSlotState GetBrushState(int brushNum)
        {
            bool[] states = GetDIStatus();
            return states[_brushesDI[brushNum]] ? BrushSlotState.EMPTY : BrushSlotState.OCCUPIED;
        }

        /// <summary>
        /// Enabled/disables brush holder checker by applying state to specific pin
        /// </summary>
        /// <param name="enable">If true, checker will be enabled, else it will be disabled</param>
        public void SetBrushHolderCheckingState(bool enable) => SetDOState(0, 9, enable);

        /// <summary>
        /// Draw line with canvas 2D coordinates
        /// </summary>
        /// <param name="x">X-axis offset in millimeters <i>(or special units like 25 micron?)</i></param>
        /// <param name="y">Y-axis offset in millimeters</param>
        public void MoveHorizontal(double x, double y)
        {
            Point point3d = _canvasCoordinateSystem!.CanvasPointToWorldPoint(x, y, _currentHeight); //new point

            MoveLinear(new CartesianPosition(point3d, _canvasCoordinateSystem.RPYParameters), 100, 100, MovementType.Absolute);

            _currentX = x;
            _currentY = y;
        }


    }
}

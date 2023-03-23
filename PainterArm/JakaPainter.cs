﻿using JakaAPI.Types;
using JakaAPI.Types.Math;
using JakaAPI;
using PainterArm.Calibration;
using System;

namespace PainterArm
{
    /// <summary>
    /// Jaka Robot based implementation of painting robot
    /// </summary>
    public class JakaPainter : JakaRobot
    {
        private CoordinateSystem2D? _canvasCoordinateSystem;
        private double _currentX, _currentY, _currentHeight;

        private Dictionary<int, CartesianPosition> _brushesLocations;
        private CartesianPosition _dryerLocation;
        private int _brushLength;
        public int CurrentBrush { get; private set; }

        /// <summary>
        /// Indicates whether the grip of the robot is being in grap state
        /// </summary>
        private bool _grip;

        public AbstractCalibrationBehavior CanvasCalibrationBehavior;

        public ref CoordinateSystem2D? GetCanvasCoordinateSystemReference() => ref _canvasCoordinateSystem;

        public JakaPainter(string domain, int portSending = 10001, int portListening = 10000)
            : base(domain, portSending, portListening)
        {
            _brushesLocations = new Dictionary<int, CartesianPosition>();
            _grip = false;
            _currentX = 0;
            _currentY = 0;
            _currentHeight = 0;
            _brushLength = 100;

            CurrentBrush = -1;
            SetDOState(0, 0, _grip);
            CanvasCalibrationBehavior = new ManualCalibration(this);
        }

        /// <summary>
        /// Canvas calibration based on existing <see cref="CoordinateSystem2D"/>
        /// </summary>
        /// <param name="cs">Existing coordinate system to be used as canvas</param>
        public void SetCalibrationSurface(CoordinateSystem2D cs)
        {
            _canvasCoordinateSystem = cs;
            Console.WriteLine($"Calibrated coordinates:\n{_canvasCoordinateSystem}");
        }

        /// <summary>
        /// Brushes calibration by a point
        /// </summary>
        public void CalibrateBrushes()
        {
            Console.WriteLine("(1) Add new brush location\n(0) End calibration");
            //Dictionary<int, CartesianPosition> brushesLocations = new();

            while (true)
            {
                int option = Int32.Parse(Console.ReadLine());
                switch (option)
                {
                    case 1:
                        _brushesLocations.Add(_brushesLocations.Count, GetRobotData().ArmCartesianPosition);
                        break;
                    case 0:
                        return /*brushesLocations*/;
                }
            }
        }

        /// <summary>
        /// Dryer calibration by a point
        /// </summary>
        public void CalibrateDryer()
        {
            Console.WriteLine("(1) Add dryer location\n(0) End calibration");
            while (true)
            {
                int option = Int32.Parse(Console.ReadLine());
                switch (option)
                {
                    case 1:
                        _dryerLocation = GetRobotData().ArmCartesianPosition;
                        break;
                    case 0:
                        return;
                }
            }
        }

        /// <summary>
        /// Draw line with canvas 2D coordinates
        /// </summary>
        /// <param name="x">X-axis offset in millimeters <i>(or special units like 25 micron?)</i></param>
        /// <param name="y">Y-axis offset in millimeters</param>
        public void DrawLine(double x, double y)
        {
            Point point3d = _canvasCoordinateSystem!.CanvasPointToWorldPoint(_currentX = x, _currentY = y, _currentHeight);
            Console.WriteLine(point3d);

            MoveLinear(new CartesianPosition(point3d, _canvasCoordinateSystem.RPYParameters), 100, 25, MovementType.Absolute);
        }

        // Create water vortex, not implemented yet
        public void MixWater()
        {
            Console.WriteLine("Water vortex start...");
            Thread.Sleep(1000);
            Console.WriteLine("Water vortex end...");
        }

        public void BrushOrthogonal(double height)
        {
            Point point3d = _canvasCoordinateSystem!.CanvasPointToWorldPoint(_currentX, _currentY, _currentHeight = height);
            MoveLinear(new CartesianPosition(point3d, _canvasCoordinateSystem.RPYParameters), 100, 25, MovementType.Absolute);
        }

        // Return current brush to stand
        public void ReturnCurrentBrush()
        {   
            CartesianPosition brushPosition = _brushesLocations[CurrentBrush];
            Point brushPoint = brushPosition.Point;
            Point upperPoint = new Point(brushPoint.X, brushPoint.Y, brushPoint.Z + _brushLength);
            RPYMatrix orthogonalRPY = brushPosition.Rpymatrix;

            // Move to position above the brush
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            // Move to the brush on stand
            MoveLinear(new CartesianPosition(brushPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            GripOff();

            // Move to position above the stand again
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            CurrentBrush = -1;
        }

        // Pick new clear brush
        public void PickNewBrush(int num)
        {
            CurrentBrush = num;
            CartesianPosition brushPosition = _brushesLocations[num];
            Point brushPoint = brushPosition.Point;
            Point upperPoint = new Point(brushPoint.X, brushPoint.Y, brushPoint.Z + _brushLength);
            RPYMatrix orthogonalRPY = brushPosition.Rpymatrix;

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
            RPYMatrix orthogonalRPY = colorPosition.Rpymatrix;

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
            RPYMatrix orthogonalRPY = _dryerLocation.Rpymatrix;

            // Move to position above the dryer
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            // Move to dryer
            MoveLinear(new CartesianPosition(dryerPoint, orthogonalRPY), 100, 25, MovementType.Absolute);

            int rotationCount = 3;
            for (int i = 0; i < rotationCount; i++)
            {
                //double c = (i % 2) * 2 - 1;
                double c = Math.Pow(-1, i);
                JointMove(new JointsPosition(0, 0, 0, 0, 0, c * 30), 100, 100, MovementType.Relative);
            }

            // Move to position above the dryer again
            MoveLinear(new CartesianPosition(upperPoint, orthogonalRPY), 100, 25, MovementType.Absolute);
        }

        // Grip methods
        public void GripOn()
        {
            _grip = true;
            SetDOState(0, 0, _grip);
        }

        public void GripOff()
        {
            _grip = false;
            SetDOState(0, 0, _grip);
        }

        [Obsolete("Not recommended to use, consider using explicit methods GripOn and GripOff")]
        public void ToggleGrip()
        {
            _grip = !_grip;
            SetDOState(0, 0, _grip);
        }
    }
}

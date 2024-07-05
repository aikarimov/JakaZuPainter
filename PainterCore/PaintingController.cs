using PainterArm;
using PainterCore.Configuration;
using JakaAPI.Types;
using System.Net;
using System.Net.Sockets;

namespace PainterCore
{
    public class PaintingController
    {
        public PaintingController()
        {
            //const string ip = "192.168.1.101";
            const string ip = "192.168.1.100";


            _painter = new(ip);
            _palette = new(_painter);
            _mixer = new();
            _logger = new();
        }

        private readonly JakaPainter _painter;
        private readonly Palette _palette;
        private readonly RobotMixerDummy _mixer;
        private readonly Logger _logger;

        private ColorRGB _currentColor = new(0, 0, 0);

        private double _brushLength = 157.5;

        public async void Start()
        {
            _painter.DebugSubscribe(_logger.LogMessage);
           // InitPainter();

            Console.WriteLine("Calibration started.");
            CalibrateAllDevices();
            Console.WriteLine("Calibration ended. Press any key to continue...");
            Console.ReadKey();

            //ParserHPGL commands = new(@"..\..\..\Resources\strokes_lib.plt");
            //ParserHPGL commands = new(@"..\..\..\Resources\strokes_lib_bb.plt");
            //ParserHPGL commands = new(@"..\..\..\Resources\strokes_exp.plt");
            //ParserHPGL commands = new(@"..\..\..\Resources\delay.plt");
            ParserHPGL commands = new(@"..\..\..\Resources\strokes_calligraphy.plt");
            //ParserHPGL commands = new(@"..\..\..\Resources\phonglib.plt");
            //ParserHPGL commands = new(@"..\..\..\Resources\Circle.plt");


            //ParserHPGL commands = new(@"..\..\..\Resources\strokes_vincent.plt");
            //ParserHPGL commands = new(@"..\..\..\Resources\strokes_randlib.plt");


            bool statedown = false;//indicated whether the robot is in down or up position
                       

            foreach (CommandHPGL command in commands.GetNextCommand())
            {
                Console.WriteLine($"Executing: {command}");

                switch (command.Code)
                {
                    case CodeHPGL.IN:
                        break;
                    case CodeHPGL.PC:
                        //BrushColor(command.Arguments);
                        break;
                    case CodeHPGL.PW:
                        break;
                    case CodeHPGL.PU:
                        _painter.BrushOrthogonalMove(_brushLength + 10, MovementType.Absolute);
                        MoveHorizontal(command.Arguments);
                        statedown = false;
                        break;
                    case CodeHPGL.PD:
                        if (!statedown)
                            _painter.BrushOrthogonalMove(_brushLength + 0, MovementType.Absolute);
                        DrawLine(command.Arguments);
                        statedown = true;
                        break;
                    case CodeHPGL.BM:
                        Console.Beep();
                        BrushMove(command.Arguments);
                        break;
                    case CodeHPGL.TP:
                        TakePaint(command.Arguments);
                        break;
                    case CodeHPGL.PB:
                        _painter.PlaceBrush();
                        break;
                    case CodeHPGL.BW:
                        _painter.PlaceWasher();
                        break;
                    case CodeHPGL.TW:
                        _painter.TakeWasher();
                        break;
                    case CodeHPGL.BD:
                        _painter.PlaceDrier();
                        break;
                    case CodeHPGL.TD:
                        _painter.TakeDrier();
                        break;
                    case CodeHPGL.BB:
                        Console.Beep();
                        _painter.BezierMove(command.Arguments);
                        break;
                    case CodeHPGL.DL:
                        Console.Beep(100, 500);
                        _painter.Delay(command.Arguments);
                        break;
                    case CodeHPGL.BS:
                        Console.Beep();
                        //_painter.BrushSocket(command.Arguments);
                        _painter.BrushSocket2(command.Arguments);
                        break;
                }
                //Thread.Sleep(500);
            }
            //add sound to indicate the end of program
            Console.Beep(440, 500);


            //DisablePainter();

            Console.WriteLine("\nPress any button to exit the program...");
            Console.ReadKey();
        }

        private void CalibrateAllDevices()
        {
            // Canvas calibration
            ConfigurationManager.CalibrationDialog(out CoordinateSystem2D canvasCoordinateSystem,
                _painter.CanvasCalibrationBehavior,
                @"..\..\..\Configuration\canvas_calibration.json",
                "Canvas calibration");
            _painter.CalibrateCanvas(canvasCoordinateSystem);
            Console.WriteLine($"Calibrated coordinates:\n{canvasCoordinateSystem}\n{canvasCoordinateSystem.RPYParameters}\n");
            _logger.LogMessage($"Calibrated coordinates:\n{canvasCoordinateSystem}\n{canvasCoordinateSystem.RPYParameters}\n");
          

            // Brushes calibration
            ConfigurationManager.CalibrationDialog(out LocationDictionary brushesLocations,
                _painter.BrushesCalibrationBehavior,
                @"..\..\..\Configuration\brushes_calibration.json",
                "Brushes calibration");
            _painter.CalibrateBrushes(brushesLocations);

            // Dryer calibration
            ConfigurationManager.CalibrationDialog(out LocationDictionary dryerLocations,
                _painter.DryerCalibrationBehavior,
                @"..\..\..\Configuration\dryer_calibration.json",
                "Dryer calibration");
            _painter.CalibrateDryer(dryerLocations[dryerLocations.Count - 1]);

            // Palette calibration
            ConfigurationManager.CalibrationDialog(out CoordinateSystem2D paletteCoordinateSystem,
                _palette.CalibrationBehavior,
                @"..\..\..\Configuration\palette_calibration.json",
                "Palette calibration");
            _palette.CalibratePalette(paletteCoordinateSystem);
        }

        private void BrushColor(double[] arguments)
        {
            ColorRGB color = new(arguments[1], arguments[2], arguments[3]);

            if (_palette.IsColorAdded(color))
            {
                if (_palette.GetStrokesLeft(color) == 0)
                {
                    _palette.UpdateColor(color);
                    _mixer.MixColor(_palette.GetColorCoordinates(color), color);
                }
            }
            else
            {
                _palette.AddNewColor(color);
                _mixer.MixColor(_palette.GetColorCoordinates(color), color);
            }

            _palette.SubstractStroke(color);

            if (color != _currentColor)
            {
                if (_painter.CurrentBrush != -1)
                {
                    _painter.ReturnCurrentBrush();
                }

                _painter.MixWater();
                _painter.PickNewBrush(0);
                _painter.DryCurrentBrush();
            }

            _currentColor = color;

            _painter.DunkBrushInColor(_palette.GetColorCoordinates(color));
        }

        private void DrawLine(double[] arguments)
        {
            if (false && _palette.GetStrokesLeft(_currentColor) == 0)
            {
                _palette.UpdateColor(_currentColor);
                _mixer.MixColor(_palette.GetColorCoordinates(_currentColor), _currentColor);
            }

            if (arguments.Length == 2)
            {
                _painter.DrawLine(arguments[0], arguments[1]);
            }
            else
            {
                _painter.DrawLine(arguments[0], arguments[1], arguments[2]);
            }
        }

        private void BrushMove(double[] arguments)
        {
            if (false && _palette.GetStrokesLeft(_currentColor) == 0)
            {
                _palette.UpdateColor(_currentColor);
                _mixer.MixColor(_palette.GetColorCoordinates(_currentColor), _currentColor);
            }

            _painter.BrushMove(arguments);
        }

        private void TakePaint(double[] arguments)
        {
            _painter.TakePaint(arguments);
        }

        private void TakeaPhoto()
        {
            _painter.TakeaPhoto();
        }
            

        private void MoveHorizontal(double[] arguments)
        {
           _painter.MoveHorizontal(arguments[0], arguments[1]);
        }

        private void InitPainter()
        {
            _painter.PowerOn();
            _painter.EnableRobot();
        }

        private void DisablePainter()
        {
            _painter.GripOff();
            _painter.DisableRobot();
            _painter.PowerOff();
        }
    }
}

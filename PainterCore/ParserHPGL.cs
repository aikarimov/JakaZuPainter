using System.Globalization;

namespace PainterCore
{
    public struct CommandHPGL
    {
        public CodeHPGL Code { get; private set; }
        public double[] Arguments { get; private set; }

        public CommandHPGL(CodeHPGL code, double[] arguments)
        {
            Code = code;
            Arguments = arguments;
        }

        public override string ToString()
        {
            return $"{Code}[" + String.Join(",", Arguments.Select(p => p.ToString()).ToArray()) + "]";
        }
    }

    public enum CodeHPGL
    {
        IN, // Initialize, start a plotting job
        PC, // Pen color (r,g,b)
        PW, // Pen width (w)
        PU, // Pen up and move to (x, y)
        PD, // Pen down and move to (x, y)
        BM, // Brush move down to (x, y, deltaz) 
    }

    public class ParserHPGL
    {
        private readonly char _delimiter = ';';
        private string _filePath;
        private const double sf = 0.025; //scale factor, dividing all coordinates by 40 to get mm
        public ParserHPGL(string filePath = @"..\..\..\Resources\strokes.plt")
        {
            _filePath = filePath;
        }

        public IEnumerable<CommandHPGL> GetNextCommand()
        {
            using StreamReader reader = new(_filePath);
            string command = "";

            while (!reader.EndOfStream)
            {
                int nextChar = reader.Read();
                char c = (char)nextChar;

                if (c == _delimiter || reader.EndOfStream)
                {
                    yield return ParseCommand(command);
                    command = "";
                }
                else
                {
                    command += c;
                }
            }
        }

        private static CommandHPGL ParseCommand(string commandStr)
        {
            string codeStr = commandStr.Substring(0, 2);
            CodeHPGL code = (CodeHPGL)Enum.Parse(typeof(CodeHPGL), codeStr);

            string argsStr = commandStr.Substring(2);
            if (argsStr.Length == 0)
            {
                return new CommandHPGL(code, Array.Empty<double>());
            }
            string[] argsArr = argsStr.Split(',');

            double[] arguments = new double[argsArr.Length];
            for (int i = 0; i < argsArr.Length; i++)
            {
                arguments[i] = Double.Parse(argsArr[i], CultureInfo.InvariantCulture);
                if (code != CodeHPGL.PC) //if numbers do not refer to special commands
                {
                    arguments[i] *= sf; //convert HPGL units to mm
                }
            }

            return new CommandHPGL(code, arguments);
        }
    }
}

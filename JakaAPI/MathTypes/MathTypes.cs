﻿using static System.Math;
using System.Globalization;
using System.Text.Json.Serialization;

namespace JakaAPI.Types.Math
{
    using DoubleTranslation = Func<double, double>;

    public struct CartesianPosition
    {
        public Point Point { get; private set; }
        public RPYRotation Rpymatrix { get; private set; }

        [JsonConstructor]
        public CartesianPosition(Point point, RPYRotation rpymatrix)
        {
            Point = point;
            Rpymatrix = rpymatrix;
        }

        public CartesianPosition(double x, double y, double z, double rx, double ry, double rz)
        {
            Point = new Point(x, y, z);
            Rpymatrix = new RPYRotation(rx, ry, rz);
        }

        public CartesianPosition(double[] pos) : this(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
        {
            if (pos.Length != 6)
            {
                throw new ArgumentException("Not enough positions");
            }
        }

        public override string ToString()
        {
            return
                $"[{Point.X.ToString(CultureInfo.InvariantCulture)}," +
                $"{Point.Y.ToString(CultureInfo.InvariantCulture)}," +
                $"{Point.Z.ToString(CultureInfo.InvariantCulture)}," +
                $"{Rpymatrix.Rx.ToString(CultureInfo.InvariantCulture)}," +
                $"{Rpymatrix.Ry.ToString(CultureInfo.InvariantCulture)}," +
                $"{Rpymatrix.Rz.ToString(CultureInfo.InvariantCulture)}]";
        }
    }

    /// <summary>
    /// A structure, which represents a Point
    /// </summary>
    public struct Point
    {
        public double X { get; private set; }
        public double Y { get; private set; }
        public double Z { get; private set; }

        [JsonConstructor]
        public Point(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static explicit operator Vector3(Point point) => new(point.X, point.Y, point.Z);

        public override string ToString()
        {
            return $"[{X.ToString(CultureInfo.InvariantCulture)}," +
                $"{Y.ToString(CultureInfo.InvariantCulture)}," +
                $"{Z.ToString(CultureInfo.InvariantCulture)}]";
        }
    }

    /// <summary>
    /// A structure, which represents a geometric vector in 3 dimentions
    /// </summary>
    public class Vector3
    {
        public double Dx { get; private set; }
        public double Dy { get; private set; }
        public double Dz { get; private set; }

        public Vector3() { }

        public Vector3(double x, double y, double z)
        {
            Dx = x;
            Dy = y;
            Dz = z;
        }

        public static Vector3 operator +(Vector3 first, Vector3 second)
        {
            return new Vector3(first.Dx + second.Dx, first.Dy + second.Dy, first.Dz + second.Dz);
        }

        public static Vector3 operator -(Vector3 vector) => new Vector3(-vector.Dx, -vector.Dy, -vector.Dz);

        public static Vector3 operator -(Vector3 first, Vector3 second) => first + (-second);

        public static Vector3 operator *(Vector3 vector, double multiplier)
        {
            return new Vector3(vector.Dx * multiplier, vector.Dy * multiplier, vector.Dz * multiplier);
        }

        public static Vector3 operator /(Vector3 vector, double divider) => vector * (1.0 / divider);

        public static explicit operator Point(Vector3 vector) => new Point(vector.Dx, vector.Dy, vector.Dz);

        /// <returns>The length of this <see cref="Vector3"> instance</returns>
        public double Length() => Sqrt(Dx * Dx + Dy * Dy + Dz * Dz);

        /// <returns>A <see cref="Vector3"/> with the unit length and the same direction as the base</returns>
        /// <exception cref="DivideByZeroException"></exception>
        public Vector3 Normalized()
        {
            double length = Length();
            if (length == 0)
            {
                throw new DivideByZeroException("Length of vector is zero: cannot be normalized");
            }
            return this / length;
        }

        /// <param name="a">First vector</param>
        /// <param name="b">Second vector</param>
        /// <param name="needNormalization"><see cref="Boolean"> to set whether vectors should be normalized in advance</param>
        /// <returns>A new <see cref="Vector3"/>, which is directed perpendicular to <i>a</i> and <i>b</i></returns>
        public static Vector3 VectorProduct(Vector3 a, Vector3 b) => new(a.Dy * b.Dz - a.Dz * b.Dy, a.Dz * b.Dx - a.Dx * b.Dz, a.Dx * b.Dy - a.Dy * b.Dx);

        /// <param name="a">First vector</param>
        /// <param name="b">Second vector</param>
        public static double DotProduct(Vector3 a, Vector3 b) => a.Dx * b.Dx + a.Dy * b.Dy + a.Dz * b.Dz;

        public override string ToString()
        {
            return $"[{Dx.ToString(CultureInfo.InvariantCulture)}," +
                $"{Dy.ToString(CultureInfo.InvariantCulture)}," +
                $"{Dz.ToString(CultureInfo.InvariantCulture)}]";
        }
    }

    public class Matrix
    {
        private readonly double[,] data;
        public int NbRows { get; private set; }
        public int NbCols { get; private set; }
        public double this[int i, int j] => data[i, j];

        public Matrix(int rows, int cols)
        {
            if (rows < 1 || cols < 1)
            {
                throw new Exception($"Invalid matrix size");
            }

            NbRows = rows;
            NbCols = cols;

            data = new double[rows, cols];
            FillMatrix(0);
        }

        public Matrix(Matrix A) : this(A.data) { }

        public Matrix(double[,] data)
        {
            NbRows = data.GetLength(0);
            NbCols = data.GetLength(1);

            if (NbRows < 1 || NbCols < 1)
            {
                throw new Exception($"Invalid matrix size");
            }

            this.data = new double[NbRows, NbCols];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    this.data[i, j] = data[i, j];
                }
            }
        }

        public Matrix(Vector3 x, Vector3 y, Vector3 z)
        {
            NbCols = NbRows = 3;

            data = new double[3, 3];
            data[0, 0] = x.Dx;
            data[1, 0] = x.Dy;
            data[2, 0] = x.Dz;

            data[0, 1] = y.Dx;
            data[1, 1] = y.Dy;
            data[2, 1] = y.Dz;

            data[0, 2] = z.Dx;
            data[1, 2] = z.Dy;
            data[2, 2] = z.Dz;
        }

        //public Vector3 this[int i] => new(data[0, i], data[1, i], data[2, i]);

        public static Matrix operator *(Matrix A, Matrix B)
        {
            if (A.NbCols != B.NbRows)
            {
                throw new Exception($"Invalid matrix size");
            }

            Matrix result = new Matrix(A.NbRows, B.NbCols);

            for (int i = 0; i < result.NbRows; i++)
            {
                for (int j = 0; j < result.NbCols; j++)
                {
                    for (int k = 0; k < A.NbCols; k++)
                    {
                        result.data[i, j] += A.data[i, k] * B.data[k, j];
                    }
                }
            }

            return result;
        }

        public static Matrix operator *(Matrix A, double value)
        {
            Matrix result = new Matrix(A.NbRows, A.NbCols);

            for (int i = 0; i < A.NbRows; i++)
            {
                for (int j = 0; j < A.NbCols; j++)
                {
                    result.data[i, j] = A.data[i, j] * value;
                }
            }

            return result;
        }

        public static Matrix operator *(double value, Matrix A) => A * value;

        public static readonly Matrix Identity = new(new double[3, 3]
                {
                    { 1, 0, 0 },
                    { 0, 1, 0 },
                    { 0, 0, 1 }
                });

        public static Vector3 operator *(Matrix A, Vector3 v)
        {
            if (A.NbCols != 3 || A.NbRows != 3)
            {
                throw new Exception($"Invalid matrix size");
            }

            double[] vc = new double[3] { 0, 0, 0 };
            double[] coord = new double[3] { v.Dx, v.Dy, v.Dz };
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    vc[i] += A.data[i, j] * coord[j];
                }
            }
            return new Vector3(vc[0], vc[1], vc[2]);
        }

        public static Matrix RotationMatrix(double rx, double ry, double rz, bool degrees = true)
        {
            DoubleTranslation translate = degrees ? MathDefinitions.DegToRad : (double arg) => { return arg; };
            (double sin, double cos)
                phi = SinCos(translate(rx)),
                theta = SinCos(translate(ry)),
                psi = SinCos(translate(rz));

            return new Matrix(new double[3, 3]
            {
                { psi.cos * theta.cos, psi.cos * theta.sin * phi.sin - psi.sin * phi.cos,  psi.cos * theta.sin * phi.cos + psi.sin * phi.sin},
                { psi.sin * theta.cos, psi.sin * theta.sin * phi.sin + psi.cos * phi.cos, psi.sin * theta.sin * phi.cos - psi.cos * phi.sin },
                { -theta.sin, theta.cos * phi.sin, theta.cos * phi.cos }
            });
        }

        public RPYRotation ToRPY(bool degrees = true)
        {
            DoubleTranslation translate = degrees ? MathDefinitions.RadToDeg : (double arg) => { return arg; };

            double theta = Asin(-data[2, 0]);
            double theta2 = PI - theta;

            double phi = Atan2(data[2, 1], data[2, 2]);
            double phi2 = Atan2(-data[2, 1], -data[2, 2]);
            
            double psi = Atan2(data[1, 0], data[0, 0]);
            double psi2 = Atan2(-data[1, 0], -data[0, 0]);

            double rx = translate(phi), ry = translate(theta), rz = translate(psi);
            double rx2 = translate(phi2), ry2 = translate(theta2), rz2 = translate(psi2);

            Console.WriteLine($"\nMain solution: {rx}, {ry}, {rz}");
            Console.WriteLine($"Alternative solution: {rx2}, {ry2}, {rz2}\n");

            return new (rx, ry, rz);
        }

        private void FillMatrix(double x)
        {
            for (int i = 0; i < NbRows; i++)
            {
                for (int j = 0; j < NbCols; j++)
                {
                    data[i, j] = x;
                }
            }
        }

        public static Matrix operator ~(Matrix A) => A.Transpose();

        public Matrix Transpose()
        {
            double[,] res = new double[NbRows, NbCols];

            for (int i = 0; i < NbRows; i++)
            {
                for (int j = 0; j < NbCols; j++)
                {
                    res[i, j] = data[j, i];
                }
            }

            return new Matrix(res);
        }

        public override string ToString()
        {
            return $"([{data[0, 0]}, {data[0, 1]}, {data[0, 2]}],\n[{data[1, 0]}, {data[1, 1]}, {data[1, 2]}],\n[{data[2, 0]}, {data[2, 1]}, {data[2, 2]}])";
        }
    }

    /// <summary>
    /// A structure, which represents a roll, pitch, yaw rotation matrix
    /// </summary>
    public struct RPYRotation
    {
        public double Rx { get; private set; }
        public double Ry { get; private set; }
        public double Rz { get; private set; }

        [JsonConstructor]
        public RPYRotation(double rx, double ry, double rz)
        {
            Rx = rx;
            Ry = ry;
            Rz = rz;
        }

        public override string ToString() => $"Roll: {Rx}, Pitch: {Ry}, Yaw: {Rz}";
    }

    public static class MathDefinitions
    {
        public static double DegToRad(double value) => value * PI / 180;

        public static double RadToDeg(double value) => value * 180 / PI;
    }
}

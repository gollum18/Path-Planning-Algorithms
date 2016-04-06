/**Copyright(C) 2016 Christen Ford

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.If not, see<http://www.gnu.org/licenses/>.

    For more information regarding this program or the author, please email him at
    <cford15@mail.bw.edu> or by mail to Baldwin-Wallace University, Berea OH.
**/

using System;

using Path_Planning_Algorithms.Maps;

namespace Path_Planning_Algorithms.Algorithms
{
    /// <summary>
    /// Contains useful static methods used in geometry/trigonometry.
    /// </summary>
    public static class Geometry
    {
        public const double D = 1;
        public const double QUADRANT_ONE = .5 * Math.PI;
        public const double QUADRANT_TWO = Math.PI;
        public const double QUADRANT_THREE = 1.5 * Math.PI;
        public const double QUADRANT_FOUR = 2 * Math.PI;

        /// <summary>
        /// Unwinds a radian measure that is 
        /// </summary>
        /// <param name="radians">The radial measure to unwind.</param>
        /// <returns>The radial measure less than 2PI whose quadrant is consistent with the initial radial measure.</returns>
        public static double Unwind(double radians)
        {
            if (radians < 0)
            {
                while (radians < 0)
                {
                    radians += QUADRANT_FOUR;
                }
            }
            else
            {
                while (radians > QUADRANT_FOUR)
                {
                    radians -= QUADRANT_FOUR;
                }
            }

            return radians;
        }

        /// <summary>
        /// Determines the quadrant for a radial measure.
        /// </summary>
        /// <param name="radians">The radial measure.</param>
        /// <returns>1-4 determined by the </returns>
        public static int Quadrant(double radians)
        {
            //First unwind the measure so we can determine its quadrant
            if (radians < 0 || radians > QUADRANT_FOUR)
            {
                radians = Unwind(radians);
            }

            //Next determine the quadrant
            if (radians >= 0 && radians <= QUADRANT_ONE) //Quadrant One
            {
                return 1;
            }
            else if (radians > QUADRANT_ONE && radians <= QUADRANT_TWO) //Quadrant Two
            {
                return 2;
            }
            else if (radians > QUADRANT_TWO && radians <= QUADRANT_THREE) //Quadrant Three
            {
                return 3;
            }
            else //Quadrant Four
            {
                return 4;
            }
        }

        /// <summary>
        /// Gets the measure of an angle on the unit circle that falls from 0 to 1/2 PI.
        /// </summary>
        /// <param name="radians">The radial measure.</param>
        /// <returns>Radial measure from 0 to 1.2 PI.</returns>
        public static double UnitCircleMeasure(double radians)
        {
            switch (Quadrant(radians))
            {
                case 1:
                    return radians;
                case 2:
                    return QUADRANT_TWO - radians;
                case 3:
                    return QUADRANT_THREE - radians;
                case 4:
                    return QUADRANT_FOUR - radians;
                default:
                    return 0;
            }
        }

        /// <summary>
        /// Calculates the octile distance between two cells, where D1 = 1 and D2 = Math.Sqrt(2).
        /// </summary>
        /// <param name="c1">The first cell.</param>
        /// <param name="c2">The second cell.</param>
        /// <returns>The octile distance.</returns>
        public static double OctileDistance(Cell c1, Cell c2)
        {
            double dx = Math.Abs(c1.X - c2.X);
            double dy = Math.Abs(c1.Y - c2.Y);
            return D * (dx + dy) + (Math.Sqrt(2.0) - 2 * D) * Math.Min(dx, dy);
        }

        /// <summary>
        /// Calculates the distance between two cells using the euclidean distance formula
        /// that has been modified for any-angle path planning.
        /// </summary>
        /// <param name="c1">The first cell.</param>
        /// <param name="c2">The second cell.</param>
        /// <returns>The any-angle euclidean distance.</returns>
        public static double EuclideanDistance(Cell c1, Cell c2)
        {
            double dx = Math.Abs(c1.X - c2.X);
            double dy = Math.Abs(c1.Y - c2.Y);
            return D * Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// Uses the standard euclidean Distance formula to calculate the Distance 
        /// between two cells.
        /// </summary>
        /// <param name="c1">The first cell.</param>
        /// <param name="c2">The second cell.</param>
        /// <returns>The Distance between the two cells.</returns>
        public static double Distance(Cell c1, Cell c2) =>
            Math.Sqrt(Math.Pow(c2.X - c1.X, 2) + Math.Pow(c2.Y - c1.Y, 2));

        /// <summary>
        /// Calculates the slope of a line.
        /// </summary>
        /// <param name="c1">The first cell on the line.</param>
        /// <param name="c2">The second cell on the line.</param>
        /// <returns>0 if any abnormal conditions occur like divide-by-zero, etc. 
        /// Otherwise the slope of the line.</returns>
        public static double Slope(Cell c1, Cell c2)
        {
            double xDiff = c2.X - c1.X;
            double yDiff = c2.Y - c1.Y;

            return (xDiff == 0 ? 0 : yDiff / xDiff);
        }

        /// <summary>
        /// Calculates the angle of a triangle using the law of cosines. NOTE: Vertex C is always the vertex that joins Vertices A and B.
        /// Any deviation from this rule will create the wrong result.
        /// </summary>
        /// <param name="angle">The angle to calculate.</param>
        /// <param name="vertexA">Vertex A of the triangle</param>
        /// <param name="vertexB">Vertex B of the triangle.</param>
        /// <param name="vertexC">Vertex C of the triangle.</param>
        /// <returns>0 if this is not a triangle, otherwise the angle in radians.</returns>
        /// <example>Vertex A -> Vertex C, Vertex B -> Vertex C, Vertex A -> Vertex B</example>
        public static double LawOfCosines(Angle angle, Cell vertexA, Cell vertexB, Cell vertexC)
        {
            //Need to determine which side goes with which angle
            double hypotenuse = 0;
            double opposite = 0;
            double adjacent = 0;
            double numerator = 0;
            double denominator = 0;

            if (angle == Angle.A)
            {
                opposite = Distance(vertexB, vertexC);
                if (Distance(vertexA, vertexB) > Distance(vertexA, vertexC))
                {
                    hypotenuse = Distance(vertexA, vertexB);
                    adjacent = Distance(vertexA, vertexC);
                }
                else
                {
                    hypotenuse = Distance(vertexA, vertexC);
                    adjacent = Distance(vertexA, vertexB);
                }
                numerator = (-Math.Pow(opposite, 2) + Math.Pow(adjacent, 2) + Math.Pow(hypotenuse, 2));
            }
            else if (angle == Angle.B)
            {
                opposite = Distance(vertexA, vertexC);
                if (Distance(vertexA, vertexB) > Distance(vertexB, vertexC))
                {
                    hypotenuse = Distance(vertexA, vertexB);
                    adjacent = Distance(vertexB, vertexC);
                }
                else
                {
                    hypotenuse = Distance(vertexB, vertexC);
                    adjacent = Distance(vertexA, vertexB);
                }
                numerator = (Math.Pow(adjacent, 2) - Math.Pow(opposite, 2) + Math.Pow(hypotenuse, 2));
            }
            else if (angle == Angle.C)
            {
                opposite = Distance(vertexA, vertexB);
                if (Distance(vertexA, vertexC) > Distance(vertexB, vertexC))
                {
                    hypotenuse = Distance(vertexA, vertexC);
                    adjacent = Distance(vertexB, vertexC);
                }
                else
                {
                    hypotenuse = Distance(vertexB, vertexC);
                    adjacent = Distance(vertexA, vertexC);
                }
                numerator = (Math.Pow(adjacent, 2) + Math.Pow(hypotenuse, 2) - Math.Pow(opposite, 2));
            }

            denominator = 2 * adjacent * hypotenuse;
            return (denominator == 0 ? 0 : Math.Acos(numerator / denominator));
        }

        /// <summary>
        /// Converts a measure from radians to degrees.
        /// </summary>
        /// <param name="radians">The radial measure.</param>
        /// <returns>The degree measure.</returns>
        public static double ToDegrees(double radians) =>
            radians * (180 / Math.PI);
    }

    public enum Angle
    {
        A, B, C
    }
}

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

namespace Path_Planning_Algorithms.Maps
{
    /// <summary>
    /// An implementation of a cell in a path-planning grid.
    /// </summary>
    public class Cell : IComparable<Cell>, ICloneable
    {
        //Constructors
        public Cell(int x, int y)
        {
            X = x; Y = y;
        }

        private Cell (int x, int y, double g, double h, double f, Cell p)
        {
            X = x; Y = y; G_Score = g; H_Score = h; F_Score = f; Parent = p;
        }

        // The X coordinate of this cell.
        public int X { get; set; }
        // The Y coordinate of this cell.
        public int Y { get; set; }
        // Distance so far.
        public double G_Score { get; internal set; }
        // Distance to finish.
        public double H_Score { get; internal set; }
        // Estimate of the distance so far and distance to finish.
        public double F_Score { get; internal set; }
        // Reference to the parent cell.
        public Cell Parent { get; internal set; }
        /// <summary>
        /// Gets the hashcode for this cell.
        /// </summary>
        /// <returns>this.X + this.Y</returns>
        public override int GetHashCode() => X + Y;
        /// <summary>
        /// Determines if this cell is equal to another cell.
        /// </summary>
        /// <param name="obj">The cell to cehck equality against.</param>
        /// <returns>True if this cell is equal to the other, false if not.</returns>
        public override bool Equals(object obj) => (X == ((Cell) obj)?.X && Y == ((Cell) obj)?.Y ? true : false);
        /// <summary>
        /// Gets a string represenatation of this cell.
        /// </summary>
        /// <returns>The coordinates of this cell as well as the values contained in this cell.</returns>
        public override string ToString() => $"({X}, {Y}), G:{G_Score}, H:{H_Score}, F:{F_Score}";
        /// <summary>
        /// Creates a deep copy of the cell.
        /// </summary>
        /// <returns>Deep copy of the cell.</returns>
        public object Clone() => new Cell(X, Y, G_Score, H_Score, F_Score, Parent);
        /// <summary>
        /// Determines ordering for cells based on their f-score.
        /// </summary>
        /// <param name="other">The other cell to compare against.</param>
        /// <returns>Comparable status of -1, 0, +1 as to whether this cells f-score is less than, equal to, or greater than another cells f-score.</returns>
        int IComparable<Cell>.CompareTo(Cell other) => F_Score.CompareTo(other?.F_Score);
    }
}

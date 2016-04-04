using System;

namespace Path_Planning_Algorithms.Maps
{
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

        //Attributes
        public int X { get; set; }
        public int Y { get; set; }
        public double G_Score { get; set; }
        public double H_Score { get; set; }
        public double F_Score { get; set; }
        public Cell Parent { get; set; }

        //Methods
        public override int GetHashCode() => X + Y;
        public override bool Equals(object obj) => (X == ((Cell) obj)?.X && Y == ((Cell) obj)?.Y ? true : false);
        public override string ToString() => $"({X}, {Y})";
        public object Clone() => new Cell(X, Y, G_Score, H_Score, F_Score, Parent);
        int IComparable<Cell>.CompareTo(Cell other) => F_Score.CompareTo(other?.F_Score);
    }
}

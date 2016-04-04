using System;
using System.Collections.Generic;
using System.Linq;

namespace Path_Planning_Algorithms.Algorithms
{
    public static class Utility
    {
        public static IList<T> CopyListOfPrimitives<T>(IList<T> list)
        {
            IList<T> copy = new T[list.Count];

            for (int i = 0; i < list.Count; i++)
            {
                copy[i] = list[i];
            }

            return copy;
        }

        public static IList<T> CopyListOfObjects<T>(IList<T> list) where T : ICloneable
        {
            IList<T> copy = new T[list.Count];

            for (int i = 0; i < list.Count; i++)
            {
                copy[i] = (T)list[i].Clone();
            }

            return copy;
        }

        public static Tuple<T1, T2> CopyPair<T1, T2>(Tuple<T1, T2> pair) =>
            new Tuple<T1, T2>(pair.Item1, pair.Item2);
    }
}

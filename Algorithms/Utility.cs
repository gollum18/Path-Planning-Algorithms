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
using System.Collections.Generic;

namespace Path_Planning_Algorithms.Algorithms
{
    /// <summary>
    /// Contains useful utility classes for manipulating objects and containers.
    /// </summary>
    public static class Utility
    {
        /// <summary>
        /// Creates a copy of a list containing value types (primitives).
        /// </summary>
        /// <typeparam name="T">Must be a primitive type.</typeparam>
        /// <param name="list">The list to copy.</param>
        /// <returns>Deep copy of the list.</returns>
        public static IList<T> CopyListOfPrimitives<T>(IList<T> list)
        {
            ///Perform argument checks
            if (list == null)
            {
                throw new NullReferenceException("ERROR! Cannot copy the list, it is null!");
            }
            else if (list.Count == 0)
            {
                throw new ArgumentException("ERROR! Cannot copy a list that contains no items!");
            }

            //Create a return list, same size as passed in list
            IList<T> copy = new T[list.Count];

            //Copy each item from the passed in list, to the return list
            foreach (var item in list)
            {
                copy.Add(item);
            }

            //Return the copy
            return copy;
        }

        /// <summary>
        /// Creates a copy of a list containing reference types (objects).
        /// </summary>
        /// <typeparam name="T">Must be a reference type.</typeparam>
        /// <param name="list">The list to copy.</param>
        /// <returns>Deep copy of the list.</returns>
        public static IList<T> CopyListOfObjects<T>(IList<T> list) where T : ICloneable
        {
            //Perform argument checks
            if (list == null)
            {
                throw new NullReferenceException("ERROR! Cannot copy the list, it is null!");
            }
            else if (list.Count == 0)
            {
                throw new ArgumentException("ERROR! Cannot copy a list that contains no items!");
            }

            //Create a return list, same size as the passed in list
            IList<T> copy = new T[list.Count];

            //Copy each item from the passed in list, to the return list.
            foreach (var item in list)
            {
                copy.Add((T)item.Clone());
            }

            //Return the copy
            return copy;
        }

        /// <summary>
        /// Creates a copy of a pair.
        /// </summary>
        /// <typeparam name="T1">The type of the first parameter in the pair.</typeparam>
        /// <typeparam name="T2">The type of the second parameter in the pair.</typeparam>
        /// <param name="pair">The pair to copy.</param>
        /// <returns>Null if the pair is null, otherwise a copy of the pair.</returns>
        public static Tuple<T1, T2> CopyPair<T1, T2>(Tuple<T1, T2> pair) =>
            (pair == null ? null : new Tuple<T1, T2>(pair.Item1, pair.Item2));
    }
}

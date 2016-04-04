using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Path_Planning_Algorithms.Maps;

namespace Path_Planning_Algorithms.Algorithms
{
    public static class Utility
    {
        public static Map[] CloneMapArray(Map[] array)
        {
            Map[] clone = new Map[array.Count()];

            int i = 0;
            foreach (Map map in array)
            {
                clone[i] = (Map)map.Clone();
                i++;
            }

            return array;
        } 
    }
}

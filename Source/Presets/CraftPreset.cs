using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Kramax.Presets
{
    public class CraftPreset
    {
        string name;
        APPreset pa;

        public CraftPreset(string Name, APPreset PA)
        {
            name = Name;
            pa = PA;
        }

        public string Name
        {
            get { return name; }
            set { name = value; }
        }

        public APPreset apPreset
        {
            get { return pa; }
            set { pa = value; }
        }

   
        public bool Dead
        {
            get { return pa == null; }
        }
    }
}

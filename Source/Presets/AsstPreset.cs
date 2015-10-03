using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Kramax.Utility;

namespace Kramax.Presets
{
    using PID;
    /// <summary>
    /// Holds all the PID tuning values for the 7 (or more if required) controllers involved.
    /// </summary>
    public class APPreset
    {
        public string name;
        public List<double[]> PIDGains = new List<double[]>();

        public APPreset(List<APController> controllers, string Name) // used for adding a new preset, can clone the current values
        {
            name = Name;
            Update(controllers);
        }

        public APPreset(APController[] controllers, string Name) // used for adding a new preset, can clone the current values
        {
            name = Name;
            Update(controllers);
        }

        public APPreset(List<double[]> gains, string Name) // used for loading presets from file
        {
            name = Name;
            PIDGains = gains;        
        }

        public void Update(List<APController> controllers)
        {
            PIDGains.Clear();
            foreach (APController controller in controllers)
            {
                double[] gains = new double[9];
                gains[0] = controller.PGain;
                gains[1] = controller.IGain;
                gains[2] = controller.DGain;
                gains[3] = controller.OutMin;
                gains[4] = controller.OutMax;
                gains[5] = controller.ClampLower;
                gains[6] = controller.ClampUpper;
                gains[7] = controller.Scalar;
                gains[8] = controller.Easing;

                PIDGains.Add(gains);
            }
        }

        public void Update(APController[] controllers)
        {
            PIDGains.Clear();
            foreach (APController controller in controllers)
            {
                double[] gains = new double[9];
                gains[0] = controller.PGain;
                gains[1] = controller.IGain;
                gains[2] = controller.DGain;
                gains[3] = controller.OutMin;
                gains[4] = controller.OutMax;
                gains[5] = controller.ClampLower;
                gains[6] = controller.ClampUpper;
                gains[7] = controller.Scalar;
                gains[8] = controller.Easing;

                PIDGains.Add(gains);
            }
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WpfApplication1
{
    class GaussianRandom
    {
        private readonly Random rand;

        public readonly double Mean;
        public readonly double StandardDeviation;

        public GaussianRandom(double mean = 0, double standardDeviation = 1, int? seed = null)
        {
            Mean = mean;
            StandardDeviation = standardDeviation;
            rand = seed == null ? new Random() : new Random(seed.Value);
        }

        public double NextDouble()
        {
            // these are uniform(0,1) random doubles
            double u1 = rand.NextDouble(); 
            double u2 = rand.NextDouble();

            // random normal(0,1)
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2); 

            // random normal(mean,stdDev^2)
            double randNormal = Mean + StandardDeviation * randStdNormal;

            return randNormal;
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using MathNet.Numerics.LinearAlgebra;

namespace WpfApplication1
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        GaussianRandom rnd;

        // [x y x' y']
        Vector<double> Xprev = Vector<double>.Build.Dense(4);

        public MainWindow()
        {
            InitializeComponent();

            rnd = new GaussianRandom(mean: 0, standardDeviation: 5);
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);

            double dt = 0.1;

            var A = Matrix<double>.Build.DenseOfArray(new double[,] {
                { 1, 0, dt, 0 },
                { 0, 1, 0,  dt},
                { 0, 0, 1,  0 },
                { 0, 0, 0,  1 }
            });

            var B = Vector<double>.Build.DenseOfArray(new[] {
                0.5 * dt*dt,
                0.5 * dt*dt,
                dt,
                dt
            });

            // No acceleration
            double u = 0;
            double w = 0;

            var Xk = A * Xprev + B * u + w;

            Point pos = e.GetPosition(inkCanvas);
            pos.Offset(rnd.NextDouble(), rnd.NextDouble());

            inkCanvas.Strokes.Add(new Stroke(new StylusPointCollection(new[] { pos })));
            if (inkCanvas.Strokes.Count > 50)
            {
                inkCanvas.Strokes.RemoveAt(0);
            }
        }
    }
}

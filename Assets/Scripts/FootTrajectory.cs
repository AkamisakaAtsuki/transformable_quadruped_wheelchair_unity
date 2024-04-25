using UnityEngine;
using System;

public abstract class FootTrajectoryGenerator
{
    public int tgPhase;  // stop: 0, swing: 1, stance: 2
    public double frequency;

    protected double baseFrequency;
    protected double initialPhi;

    protected FootTrajectoryGenerator(double baseFrequency, double initialPhi)
    {
        this.baseFrequency = baseFrequency;
        this.initialPhi = initialPhi;
    }

    protected double ComputePhi(double time, double frequencyOffset)
    {
        return (initialPhi + 2 * Math.PI * (baseFrequency + frequencyOffset) * time) % (2 * Math.PI);
    }

    public abstract Vector3 ComputeTrajectory(double time, double frequencyOffset = 0, double width = 1, double height = 1, bool stop = false);
}

public class SplineFootTrajectoryGenerator : FootTrajectoryGenerator
{
    public SplineFootTrajectoryGenerator(double baseFrequency, double initialPhi) : base(baseFrequency, initialPhi) { }

    public override Vector3 ComputeTrajectory(double time, double frequencyOffset = 0, double width = 1, double height = 1, bool stop = false)
    {
        frequency = baseFrequency + frequencyOffset;

        if (!stop)
        {
            double phi = ComputePhi(time, frequencyOffset);
            double x = width / 2 * Math.Cos(phi);
            double k = 2 * (phi - Math.PI) / Math.PI;

            double y;
            if (k >= 0 && k < 1)
            {
                tgPhase = 1; // swing
                y = height * (-2 * Math.Pow(k, 3) + 3 * Math.Pow(k, 2));
            }
            else if (k >= 1 && k < 2)
            {
                tgPhase = 1; // swing
                y = height * (2 * Math.Pow(k, 3) - 9 * Math.Pow(k, 2) + 12 * k - 4);
            }
            else
            {
                tgPhase = 2; // stance
                y = 0;
            }

            return new Vector3((float)x, (float)y, 0f);
        }
        else
        {
            tgPhase = 0;  // stop
            return new Vector3(0f, 0f, 0f);
        }
    }
}

public class SineFootTrajectoryGenerator : FootTrajectoryGenerator
{
    public SineFootTrajectoryGenerator(double baseFrequency, double initialPhi) : base(baseFrequency, initialPhi) { }

    public override Vector3 ComputeTrajectory(double time, double frequencyOffset = 0, double width = 1, double height = 1, bool stop = false)
    {
        frequency = baseFrequency + frequencyOffset;

        if (!stop)
        {
            double phi = ComputePhi(time, frequencyOffset);
            double x = width / 2 * Math.Cos(phi);

            double y;
            if (phi >= 0 && phi < Math.PI)
            {
                tgPhase = 2;  // stance
                y = 0;
            }
            else
            {
                tgPhase = 1;  // swing
                y = height * Math.Sin(phi - Math.PI);
            }

            return new Vector3((float)x, (float)y, 0f);
        }
        else
        {
            tgPhase = 0;  // stop
            return new Vector3(0f, 0f, 0f);
        }
    }
}

public class BezierFootTrajectoryGenerator : FootTrajectoryGenerator
{
    private const int DesiredVelocity = 10;
    private const int SwingPeriod = 1;
    private const int PointsCount = 11;
    private static readonly Vector2[] ControlPoints = new Vector2[]
    {
        new Vector2(-0.5f, 0),
        new Vector2(-(0.5f + DesiredVelocity / ((PointsCount+ 1) * SwingPeriod) / 340), 0),
        new Vector2(-0.88f, 0.73f),
        new Vector2(-0.88f, 0.73f),
        new Vector2(-0.88f, 0.73f),
        new Vector2(0, 0.73f),
        new Vector2(0, 0.73f),
        new Vector2(0, 1),
        new Vector2(0.88f, 1),
        new Vector2(0.88f, 1),
        new Vector2(0.5f + DesiredVelocity / ((PointsCount + 1) * SwingPeriod) / 340, 0),
        new Vector2(0.5f, 0)
    };

    public BezierFootTrajectoryGenerator(double baseFrequency, double initialPhi) : base(baseFrequency, initialPhi) { }

    private long Factorial(int n)
    {
        long result = 1;
        for (int i = 2; i <= n; i++)
        {
            result *= i;
        }
        return result;
    }

    private long Combinatorial(int n, int k)
    {
        return Factorial(n) / (Factorial(k) * Factorial(n - k));
    }

    private Vector3 BezierCurve(Vector2[] controlPoints, float t, double width, double height)
    {
        Vector2 result = new Vector2(0, 0);
        for (int i = 0; i <= PointsCount; i++)
        {
            result += (float)(Combinatorial(PointsCount, i) * Math.Pow(1 - t, PointsCount - i) * Math.Pow(t, i)) * controlPoints[i];
        }
        result.x = (float)(width * result.x);
        result.y = (float)(height * result.y);
   
        return new Vector3(result.x, result.y, 0f);
    }

    public override Vector3 ComputeTrajectory(double time, double frequencyOffset = 0, double width = 1, double height = 1, bool stop = false)
    {
        frequency = baseFrequency + frequencyOffset;

        if (!stop)
        {
            double phi = ComputePhi(time, frequencyOffset);
            double x = width / 2 * Math.Cos(phi);
            double k = 2 * (phi - Math.PI) / Math.PI;

            Vector3 result;
            if (phi >= 0 && phi < Math.PI)
            {
                tgPhase = 2;  // stance
                result = new Vector3((float)x, (float)(-0.1 * Math.Sin(phi)), 0f);
            }
            else
            {
                tgPhase = 1;  // swing
                result = BezierCurve(ControlPoints, (float)((phi - Math.PI) / Math.PI), width, height);
            }

            return result;
        }
        else
        {
            tgPhase = 0;  // stop
            return new Vector3(0f, 0f, 0f);
        }
    }
}


/*using System;
using UnityEngine;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

public class FootTrajectoryGenerator
{
    protected double baseFrequency;
    protected double initialPhi;

    public FootTrajectoryGenerator(double baseFrequency, double initialPhi)
    {
        this.baseFrequency = baseFrequency;
        this.initialPhi = initialPhi;
    }

    protected double ComputePhi(double time, double frequencyOffset)
    {
        return (initialPhi + 2 * Math.PI * (baseFrequency + frequencyOffset) * time) % (2 * Math.PI);
    }

    public virtual Vector<double> ComputeTrajectory(double time)
    {
        return DenseVector.OfArray(new double[] { 0, 0 });
    }
}

public class SplineFootTrajectoryGenerator : FootTrajectoryGenerator
{
    public SplineFootTrajectoryGenerator(double baseFrequency, double initialPhi) : base(baseFrequency, initialPhi) { }

    private Vector<double> ComputeTrajectory(double time, double frequencyOffset, double width, double height)
    {
        double phi = ComputePhi(time, frequencyOffset);
        double x = width / 2 * Math.Cos(phi);
        double k = 2 * (phi - Math.PI) / Math.PI;
        double y;

        if (k >= 0 && k < 1)
        {
            y = height * (-2 * Math.Pow(k, 3) + 3 * Math.Pow(k, 2));
        }
        else if (k >= 1 && k < 2)
        {
            y = height * (2 * Math.Pow(k, 3) - 9 * Math.Pow(k, 2) + 12 * k - 4);
        }
        else
        {
            y = 0;
        }

        return DenseVector.OfArray(new double[] { x, y });
    }

    public override Vector<double> ComputeTrajectory(double time, double frequencyOffset = 0, double width = 1, double height = 1)
    {
        return ComputeTrajectory(time, frequencyOffset, width, height);
    }
}

public class SineFootTrajectoryGenerator : FootTrajectoryGenerator
{
    public SineFootTrajectoryGenerator(double baseFrequency, double initialPhi) : base(baseFrequency, initialPhi) { }

    private Vector<double> ComputeTrajectory(double time, double frequencyOffset, double width, double height)
    {
        double phi = ComputePhi(time, frequencyOffset);
        double x = width / 2 * Math.Cos(phi);
        double y = (phi >= 0 && phi < Math.PI) ? 0 : height * Math.Sin(phi - Math.PI);

        return DenseVector.OfArray(new double[] { x, y });
    }

    public override Vector<double> ComputeTrajectory(double time, double frequencyOffset = 0, double width = 1, double height = 1)
    {
        return ComputeTrajectory(time, frequencyOffset, width, height);
    }
}

public class BezierFootTrajectoryGenerator : FootTrajectoryGenerator
{
    private const int DesiredVelocity = 10;
    private const int SwingPeriod = 1;
    private const int PointsCount = 11;
    private static readonly Vector<double>[] ControlPoints = {
        DenseVector.OfArray(new double[] {-0.5, 0}),
        DenseVector.OfArray(new double[] {-(0.5 + DesiredVelocity / ((PointsCount + 1) * SwingPeriod) / 340),  0}),
        DenseVector.OfArray(new double[] {-0.88, 0.73}),
        DenseVector.OfArray(new double[] {-0.88, 0.73}),
        DenseVector.OfArray(new double[] {-0.88, 0.73}),
        DenseVector.OfArray(new double[] {0, 0.73}),
        DenseVector.OfArray(new double[] {0, 0.73}),
        DenseVector.OfArray(new double[] {0, 1}),
        DenseVector.OfArray(new double[] {0.88, 1}),
        DenseVector.OfArray(new double[] {0.88, 1}),
        DenseVector.OfArray(new double[] {0.5 + DesiredVelocity / ((PointsCount + 1) * SwingPeriod) / 340, 0}),
        DenseVector.OfArray(new double[] {0.5, 0})
    };

    public BezierFootTrajectoryGenerator(double baseFrequency, double initialPhi) : base(baseFrequency, initialPhi) { }

    private Vector<double> BezierCurve(Vector<double>[] controlPoints, double t, double width, double height)
    {
        Vector<double> result = DenseVector.OfArray(new double[] { 0, 0 });

        for (int i = 0; i <= PointsCount; i++)
        {
            double coefficient = SpecialFunctions.Combinations(PointsCount, i);
            result += coefficient * controlPoints[i] * Math.Pow(1 - t, PointsCount - i) * Math.Pow(t, i);
        }
        result[0] *= width;
        result[1] *= height;

        return result;
    }

    private Vector<double> ComputeTrajectory(double time, double frequencyOffset, double width, double height)
    {
        double phi = ComputePhi(time, frequencyOffset);
        double x = width / 2 * Math.Cos(phi);
        double k = 2 * (phi - Math.PI) / Math.PI;

        Vector<double> result;
        if (phi >= 0 && phi < Math.PI)
        {
            result = DenseVector.OfArray(new double[] { x, -0.1 * Math.Sin(phi) });
        }
        else
        {
            result = BezierCurve(ControlPoints, (phi - Math.PI) / Math.PI, width, height);
        }

        return result;
    }

    public override Vector<double> ComputeTrajectory(double time, double frequencyOffset = 0, double width = 1, double height = 1)
    {
        return ComputeTrajectory(time, frequencyOffset, width, height);
    }
}*/

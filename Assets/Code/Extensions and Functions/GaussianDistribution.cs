/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: S2R Guassian Distributed Noise for Position and Orientation       *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class GaussianDistribution : MonoBehaviour
{  
    public float globalStdNoiseMeter = 0.1f;
    public float globalStdNoiseDegrees = 0.1f;

    // Helper Function Returning Position with Gaussian Noise
    public static Vector3 PositionWGaussianNoise (Vector3 actualPos, float stdNoiseMeters)
    {
        Vector3 predictedPos = 
            new Vector3(actualPos.x + RandomGaussian(-stdNoiseMeters, stdNoiseMeters),
            actualPos.y + RandomGaussian(-stdNoiseMeters, stdNoiseMeters),
            actualPos.z + RandomGaussian(-stdNoiseMeters, stdNoiseMeters));
        return predictedPos;
    }

    // Helper Function Returning Rotation with Gaussian Noise
    public static Vector3 RotationWGaussianNoise(Vector3 actualRot, float stdNoiseDegrees)
    {
        Vector3 predictedRot =
            new Vector3(actualRot.x + RandomGaussian(-stdNoiseDegrees, stdNoiseDegrees),
            actualRot.y + RandomGaussian(-stdNoiseDegrees, stdNoiseDegrees),
            actualRot.z + RandomGaussian(-stdNoiseDegrees, stdNoiseDegrees));
        return predictedRot;
    }

    // Returns Random Gaussian (Float)
    public static float RandomGaussian(float minValue = 0.0f, float maxValue = 1.0f)
    {
        float u, v, S;
        do
        {
            u = 2.0f * UnityEngine.Random.value - 1.0f;
            v = 2.0f * UnityEngine.Random.value - 1.0f;
            S = u * u + v * v;
        }
        while (S >= 1.0f);

        // Standard Normal Distribution
        float std = u * Mathf.Sqrt(-2.0f * Mathf.Log(S) / S);

        // Normal Distribution Centered Between the Clamped Min and Max Values
        // Following the "Three-sigma rule"
        float mean = (minValue + maxValue) / 2.0f;
        float sigma = (maxValue - mean) / 3.0f;
        return Mathf.Clamp(std * sigma + mean, minValue, maxValue);
    }
}

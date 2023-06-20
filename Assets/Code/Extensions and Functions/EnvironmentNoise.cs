/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Place Holder Script to Hold the Specified Noise Value (Meters)    *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnvironmentNoise : MonoBehaviour
{
    [Header("Noise Settings")]
    public float posNoiseMeters = 0.0f;

    void Start()
    {
        Debug.Log("Gaussian noise of " + (posNoiseMeters * 100f) + "cm, applied to: " + this.gameObject.name);
    }
}

/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Low-Pass Filter Implementation                                    *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LowPassFilter
{
    public float filteredValue;
    private float c, a1, a2, a3, b1, b2;

    public void ApplySettings(float cutOff, int sampleRate, float resonance)
    {
        c = 1.0f / (float)Mathf.Tan(Mathf.PI * cutOff / sampleRate);
        a1 = 1.0f / (1.0f + resonance * c + c * c);
        a2 = 2f * a1;
        a3 = a1;
        b1 = 2.0f * (1.0f - c * c) * a1;
        b2 = (1.0f - resonance * c + c * c) * a1;
    }

    private float[] inputHistory = new float[2];
    private float[] outputHistory = new float[3];

    public void UpdateFilter(float value)
    {
        float newOutput = a1 * value + a2 * this.inputHistory[0] + a3 * this.inputHistory[1] - b1 * this.outputHistory[0] - b2 * this.outputHistory[1];

        this.inputHistory[1] = this.inputHistory[0];
        this.inputHistory[0] = value;

        this.outputHistory[2] = this.outputHistory[1];
        this.outputHistory[1] = this.outputHistory[0];
        this.outputHistory[0] = newOutput;
        filteredValue = this.outputHistory[0];
    }
}

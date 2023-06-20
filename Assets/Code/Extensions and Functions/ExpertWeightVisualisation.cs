/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Visualises the Weight Assignments by the Manipulation Network     *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExpertWeightVisualisation : MonoBehaviour
{
    public Transform [] visExperts;
    public float [] weightsExperts;

    void Start()
    {
        weightsExperts = new float[visExperts.Length];
    }

    void Update()
    {
        for (int i = 0; i < visExperts.Length; i++)
        {
            weightsExperts[i] = Mathf.Clamp(weightsExperts[i], 0f, 1f);
            visExperts[i].localScale =
                new Vector3
                (
                    visExperts[i].localScale.x,
                    weightsExperts[i],
                    visExperts[i].localScale.z
                );
            visExperts[i].localPosition =
                new Vector3
                (
                    visExperts[i].localPosition.x,
                    weightsExperts[i] - weightsExperts[i] / 2,
                    visExperts[i].localPosition.z
                );
        }
    }
}

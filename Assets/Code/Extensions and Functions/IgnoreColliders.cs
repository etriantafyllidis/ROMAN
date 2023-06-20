/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Disables Collisions Between an Array of Specified Objects         *        
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IgnoreColliders : MonoBehaviour
{
    public Transform [] IgnoredObjectsTR;
    void Start()
    {
        Collider[] IgnoredObjectsCL = new Collider[IgnoredObjectsTR.Length];
        for (int i = 0; i < IgnoredObjectsTR.Length; i++)
            IgnoredObjectsCL[i] = IgnoredObjectsTR[i].GetComponent<Collider>();
        for (int i = 0; i < IgnoredObjectsCL.Length; i++)
            for (int j = 0; j < IgnoredObjectsCL.Length; j++)
                Physics.IgnoreCollision(IgnoredObjectsCL[i], IgnoredObjectsCL[j]);
        Destroy(GetComponent<IgnoreColliders>());
    }
}

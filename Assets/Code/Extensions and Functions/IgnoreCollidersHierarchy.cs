/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Disables Inter-Collisions on all Children Objects Based on Root   *
 *          Useful for Complex Objects Consisting of Multiple Colliders       *         
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IgnoreCollidersHierarchy : MonoBehaviour
{
    public Transform root;
    private Collider[] rootColliders; 

    void Start()
    {
        rootColliders = root.GetComponentsInChildren<Collider>();
        for (int i = 0; i < rootColliders.Length; i++)
        {
            for (int j = 0; j < rootColliders.Length; j++)
            {
                Physics.IgnoreCollision(rootColliders[i], rootColliders[j]);
            }
        }
        Destroy(GetComponent<IgnoreCollidersHierarchy>());
    }
}

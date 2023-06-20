/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Simple Script Raising Boolen Flag Based on Specified Collision    *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectInCollisionWith : MonoBehaviour
{
    public int conditionLayer;
    public GameObject gameObjectInCollision;
    public bool inContactWithTarget = false;

    void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.layer == conditionLayer)
        {
            inContactWithTarget = true;
            gameObjectInCollision = collision.gameObject;
        }
    }

    void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.layer == conditionLayer)
        {
            inContactWithTarget = false;
            gameObjectInCollision = null;
        }
    }
}

/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Simulated Force Sensor                                            *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class ForceReading : MonoBehaviour
{
    [System.NonSerialized]
    public string collisionName;

    public float fRaw;
    public float fMax; 
    public float fNorm; 
    public bool inContact;
    public bool inCollisionNoForce;

    void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.layer == 9)  
        {
            collisionName = collision.gameObject.name;
            inCollisionNoForce = true;
            fRaw = Mathf.Abs(collision.impulse.magnitude / Time.fixedDeltaTime);
            fMax = Mathf.Abs(collision.rigidbody.mass * Physics.gravity.y);
            fRaw = Mathf.Clamp(fRaw, 0, fMax);
            fNorm = fRaw / fMax;

            if (fNorm > 0.0f)
                inContact = true;
        }
    }

    void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.layer == 9) 
        {
            inCollisionNoForce = false;
            fRaw = 0.0f;
            fMax = 0.0f;
            fNorm = 0.0f;
            inContact = false;
        }
    }
}

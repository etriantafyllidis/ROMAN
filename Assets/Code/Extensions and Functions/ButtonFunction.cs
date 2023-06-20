/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: The State of a Simple Button Raising a Boolean Flag               *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ButtonFunction : MonoBehaviour
{
    public float buttonLinearDistanceNorm;
    public bool buttonActivated;

    private Vector3 initalPos;
    private float maxLimitLinearDistance;

    void Start()
    {
        initalPos = this.transform.localPosition;
        maxLimitLinearDistance = this.GetComponent<ConfigurableJoint>().linearLimit.limit;
    }

    void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.layer == 11)
        {
            buttonLinearDistanceNorm = Mathf.Clamp(Vector3.Distance(initalPos, transform.localPosition) / maxLimitLinearDistance, 0, 1);
            if (buttonLinearDistanceNorm >= 0.8f)
            {
                buttonActivated = true;
            }
        }
    }
}

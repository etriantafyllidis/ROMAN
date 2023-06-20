/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Franka Gripper State with Open / Close Functions (Rigidbody)      *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripperState : MonoBehaviour
{
    public float gripperForce = 250.0f;
    public bool openGripper = true;
    public Rigidbody RBFingerA, RBFingerB;
    private Transform TFingerA, TFingerB;
    private ConfigurableJoint CJFingerA, CJFingerB;
    private Vector3 FingerAPosOffset, FingerARest, FingerBPosOffset, FingerBRest;

    void Start()
    {
        CJFingerA = RBFingerA.GetComponent<ConfigurableJoint>();
        CJFingerB = RBFingerB.GetComponent<ConfigurableJoint>();
        FingerARest = RBFingerA.transform.localPosition;
        FingerBRest = RBFingerB.transform.localPosition;
        TFingerA = RBFingerA.transform;
        TFingerB = RBFingerB.transform;
    }

    // Conrols the Gripper State by Either ">": Opening or "<": Closing
    void FixedUpdate()
    {        
        if (openGripper)
        {
            FingerAPosOffset = new Vector3(FingerARest.x, FingerARest.y, FingerARest.z - CJFingerA.linearLimit.limit)
                - TFingerA.localPosition;
            FingerBPosOffset = TFingerB.localPosition -
                new Vector3(FingerBRest.x, FingerBRest.y, FingerBRest.z + CJFingerB.linearLimit.limit);
            Vector3 FingerAForce = FingerAPosOffset * RBFingerA.mass * gripperForce;
            Vector3 FingerBForce = FingerBPosOffset * RBFingerB.mass * gripperForce;
            RBFingerA.AddRelativeForce(FingerAForce, ForceMode.Force);
            RBFingerB.AddRelativeForce(FingerBForce, ForceMode.Force);
        }
        else
        {
            FingerAPosOffset = new Vector3(FingerARest.x, FingerARest.y, FingerARest.z + CJFingerA.linearLimit.limit)
                - TFingerA.localPosition;
            FingerBPosOffset = TFingerB.localPosition -
                new Vector3(FingerBRest.x, FingerBRest.y, FingerBRest.z - CJFingerB.linearLimit.limit);
            Vector3 FingerAForce = FingerAPosOffset * RBFingerA.mass * gripperForce;
            Vector3 FingerBForce = FingerBPosOffset * RBFingerB.mass * gripperForce;
            RBFingerA.AddRelativeForce(FingerAForce, ForceMode.Force);
            RBFingerB.AddRelativeForce(FingerBForce, ForceMode.Force);
        }
        
    }
}

/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Moves and rotates the camera with the respective input controls   *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveCamera : MonoBehaviour
{
    Vector3 anchorPoint;
    Quaternion anchorRot;
    public float translationSpeed = 1f;
    public float rotationSpeed = 1f;
    private float timeCount = 0f;

    void Update()
    {
        if (Input.GetKey(KeyCode.D))
            transform.Translate(new Vector3(translationSpeed * Time.deltaTime, 0, 0));
        if (Input.GetKey(KeyCode.A))
            transform.Translate(new Vector3(-translationSpeed * Time.deltaTime, 0, 0));
        if (Input.GetKey(KeyCode.Q))
            transform.Translate(new Vector3(0, -translationSpeed * Time.deltaTime, 0));
        if (Input.GetKey(KeyCode.E))
            transform.Translate(new Vector3(0, translationSpeed * Time.deltaTime, 0));
        if (Input.GetKey(KeyCode.W))
            transform.Translate(new Vector3(0, 0, translationSpeed * Time.deltaTime));
        if (Input.GetKey(KeyCode.S))
            transform.Translate(new Vector3(0, 0, -translationSpeed * Time.deltaTime));
        if (Input.GetKey(KeyCode.LeftShift))
            translationSpeed = 4f;
        else
            translationSpeed = 2f;
        if (Input.GetMouseButtonDown(1))
        {
            anchorPoint = new Vector3(Input.mousePosition.y, -Input.mousePosition.x);
            anchorRot = transform.rotation;
        }

        if (Input.GetMouseButton(1))
        {
            Quaternion rot = anchorRot;
            Vector3 dif = anchorPoint - new Vector3(Input.mousePosition.y, -Input.mousePosition.x);
            rot.eulerAngles += dif * 0.25f;
            transform.rotation = Quaternion.Lerp(transform.rotation, rot, timeCount * rotationSpeed);
        }

        timeCount = timeCount + Time.deltaTime;
    }
}

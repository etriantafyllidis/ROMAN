/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Extends the "DecisionRequester" for Custom Behaviour Frequencies  *
 ******************************************************************************/

using UnityEngine;
using MLAgents;

public class CustomFrequency : MonoBehaviour
{
    public int desiredFrequency = 100;
    private int currentFixedFrequency;
    private int updateFrequency, updateCounter;
    Agent agentScript;

    void Awake()
    {
        currentFixedFrequency = (int) (1/Time.fixedDeltaTime);
        agentScript = GetComponent<Agent>();
        updateFrequency = currentFixedFrequency / desiredFrequency;
    }

    void FixedUpdate()
    {
        updateCounter++;
        if (updateCounter % updateFrequency == 0)
        {
            agentScript.RequestDecision();
            agentScript.RequestAction();
        }
    }
}

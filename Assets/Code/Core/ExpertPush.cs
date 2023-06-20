/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: The Main Code for the Expert Responsible for Pushing              *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using MLAgents.Sensors;
using System.IO;

public class ExpertPush : Agent
{
    [Header("Master Network Controlled")]
    public bool agentControlled = false;
    public float agentWeight = 1.0f;

    [Header("Agent Settings")]
    public int stepSafetySync = 320;
    public float speed = 1f;

    [Header("Object Assignments")]
    public SceneManagement sceneManager;
    public Transform agentTR;
    public Transform rackTargetTR;
    public Transform endLocationTargetTR;

    [Header("Low Pass Filter Settings")]
    public bool applyFilter = false;
    public float specifiedCutOff;
    public int specifiedSampleRate;
    public float specifiedResonance = Mathf.Sqrt(2);
    private const int requiredFilteredValues = 25;
    LowPassFilter[] lowPassFilter;

    [System.NonSerialized]
    public int episode = -1;
    [System.NonSerialized]
    EnvironmentNoise environmentNoise;

    private float actualTime;
    private Rigidbody agentRB;
    private GripperState gripperState;
    private ForceReading forceFingerA, forceFingerB;
    private Vector3 agentDefaultInitPos;

    void Awake()
    {
        agentDefaultInitPos = agentTR.localPosition;
    }

    // Initialise Agent Settings and Variables Thereafter Used
    void Start()
    {
        if (!GetComponent<EnvironmentNoise>())
            environmentNoise = gameObject.AddComponent<EnvironmentNoise>();
        else
            environmentNoise = gameObject.GetComponent<EnvironmentNoise>();

        agentRB = agentTR.GetComponent<Rigidbody>();
        gripperState = agentTR.GetComponent<GripperState>();
        forceFingerA = gripperState.RBFingerA.transform.GetComponent<ForceReading>();
        forceFingerB = gripperState.RBFingerB.transform.GetComponent<ForceReading>();

        lowPassFilter = new LowPassFilter[requiredFilteredValues];
        for (int i = 0; i < lowPassFilter.Length; i++)
        {
            lowPassFilter[i] = new LowPassFilter();
            lowPassFilter[i].ApplySettings(specifiedCutOff, specifiedSampleRate, specifiedResonance);
        }
    }

    // For New Episode
    public override void OnEpisodeBegin()
    {
        if (!agentControlled)
        {
            episodeHasResetForButtonPress = true;
            ResetEpisodeParameters();
            RandomizeEpisodeVariables();
        }
        episode++;
    }

    // Objects of Interest (OIs)
    private Vector3 rackTargetPosWithNoise;
    private Vector3 rackEndLocationTargetPosWithNoise;

    // State Space
    public override void CollectObservations(VectorSensor sensor)
    {
        // Input: Robot Proprioception (6)
        sensor.AddObservation(agentTR.localPosition);
        sensor.AddObservation(agentRB.velocity);

        // Input: Robot Force Sensor (2)
        sensor.AddObservation(forceFingerA.fNorm);
        sensor.AddObservation(forceFingerB.fNorm);

        // Input: Noisy Environment Data (6)
        rackTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(rackTargetTR.localPosition, environmentNoise.posNoiseMeters);
        rackTargetPosWithNoise = LowPassFilteredPosition(rackTargetPosWithNoise, new int[] { 0, 1, 2 });
        sensor.AddObservation(rackTargetPosWithNoise);
        rackEndLocationTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(endLocationTargetTR.localPosition, environmentNoise.posNoiseMeters);
        rackEndLocationTargetPosWithNoise = LowPassFilteredPosition(rackEndLocationTargetPosWithNoise, new int[] { 3, 4, 5 });
        sensor.AddObservation(rackEndLocationTargetPosWithNoise);
        sensor.AddObservation(distanceAgentToInitPosNorm);
    }

    // The Max Allowable Divergence After Which the Episode Terminates
    private float maxAllowableDivergence = 1.5f;

    // Action Space
    public override void OnActionReceived(float[] vectorAction)
    {
        // Robot Position Control via Applied Force
        Vector3 controlSignalPos = Vector3.zero;
        controlSignalPos.y = vectorAction[0] * agentWeight;
        controlSignalPos.z = vectorAction[1] * agentWeight;
        controlSignalPos.x = vectorAction[2] * agentWeight;
        Vector3 totalForcePos = controlSignalPos * speed * agentRB.mass;
        agentRB.AddForce(totalForcePos);

        // Open or Close the Gripper
        if (vectorAction[3] * agentWeight < -0.9f)
            gripperState.openGripper = false;
        if (vectorAction[3] * agentWeight > 0.9f)
            gripperState.openGripper = true;

        // Apply Agent-Specific Rewards
        if (StepCount >= (maxStep / (maxStep <= 80000 ? 100 : stepSafetySync)))
            AgentSpecificReward();

        // Terminate if Agent Divergence is Higher than Threshold
        if (Vector3.Distance(agentDefaultInitPos, agentTR.localPosition) > maxAllowableDivergence)
            EndEpisode();

        totalReward = GetCumulativeReward();
        PrintEpisodeInfo(false);
    }

    // Agent Specific Rewards
    public float totalReward = 0;
    public float distanceRackToEndLocation;
    public float distanceAgentToRackNorm, distanceRackToEndLocationNorm, distanceAgentToInitPosNorm;
    public float maxDistanceAgentToRack = -1f, maxDistanceRackToEndLocation = -1f, maxDistanceAgentToInitPos = -1f;
    [System.NonSerialized]
    public float distanceToBase = 0.0f;
    void AgentSpecificReward()
    {
        // R1: Distance to Rack
        float distanceAgentToRack = Vector3.Distance(agentTR.localPosition, rackTargetPosWithNoise);
        if (maxDistanceAgentToRack < 0)
            maxDistanceAgentToRack = distanceAgentToRack;
        distanceAgentToRackNorm = Mathf.Clamp(distanceAgentToRack / maxDistanceAgentToRack, 0, 1);
        AddReward((1 - distanceAgentToRackNorm) * 0.0005f * speed * (distanceRackToEndLocationNorm));

        // R2: Distance Rack to Conveyor Belt Position
        distanceRackToEndLocation = Mathf.Abs(rackTargetPosWithNoise.z - rackEndLocationTargetPosWithNoise.z);
        if (maxDistanceRackToEndLocation < 0)
            maxDistanceRackToEndLocation = distanceRackToEndLocation;
        distanceRackToEndLocationNorm = Mathf.Clamp(distanceRackToEndLocation / maxDistanceRackToEndLocation, 0, 1);
        AddReward((1 - distanceRackToEndLocationNorm) * 0.005f * speed);

        // R3: Distance to Initial Position
        float distanceAgentToInitPos = Vector3.Distance(agentTR.localPosition, agentDefaultInitPos);
        if (maxDistanceAgentToInitPos < 0)
            maxDistanceAgentToInitPos = distanceAgentToInitPos;
        distanceAgentToInitPosNorm = Mathf.Clamp(distanceAgentToInitPos / maxDistanceAgentToInitPos, 0, 1);
        AddReward((1 - distanceAgentToInitPosNorm) * 0.0005f * speed * (1f - distanceRackToEndLocationNorm));
        distanceToBase = distanceAgentToInitPos;

        // Primary Goal is Achieved and Agent is Wihin Initial Position
        if (distanceAgentToInitPos < 0.1f && distanceRackToEndLocationNorm <= 0.05f && (StepCount >= (maxStep / 10)) && !agentControlled)
        {
            SetReward(1000f);
            PrintEpisodeInfo(true);
            EndEpisode();
        }
    }

    // For Printing and Debugging Purposes
    void PrintEpisodeInfo(bool invariantStepCount)
    {
        if (currentlyHeuristic && (invariantStepCount || ((StepCount) % 1000) == 0))
        {
            Debug.Log("Cumulative reward: " + GetCumulativeReward()
               + ", for episode: " + episode + ", at environment step: " + StepCount
               + ", with wall-clock time: " + (Time.realtimeSinceStartup - actualTime) + " sec.");
        }
    }

    // Heuristic Actions (For Demonstration / Imitation Purposes)
    public float[] heuristicActions;
    private bool currentlyHeuristic = false;
    public override float[] Heuristic()
    {
        heuristicActions = new float[4];
        heuristicActions[0] = Input.GetAxis("Vertical");
        heuristicActions[1] = Input.GetAxis("Horizontal");
        heuristicActions[2] = Input.GetAxis("Up_Down");
        heuristicActions[3] = Input.GetAxis("OpenCloseGripper");

        if (ButtonIsBeingPressed() && episodeHasResetForButtonPress)
            for (int i = 0; i < heuristicActions.Length; i++)
                heuristicActions[i] = 0f;
        else
            episodeHasResetForButtonPress = false;
        currentlyHeuristic = true;
        return heuristicActions;
    }

    // Functions to Reset Internal Variables and Scene Parameters
    private bool episodeHasResetForButtonPress;
    bool ButtonIsBeingPressed()
    {
        if (Input.GetButton("Vertical") || Input.GetButton("Horizontal") ||
            Input.GetButton("Up_Down") || Input.GetButton("OpenCloseGripper"))
            return true;
        return false;
    }

    void ResetEpisodeParameters()
    {
        maxDistanceAgentToRack = -1f;
        maxDistanceRackToEndLocation = -1f;
        maxDistanceAgentToInitPos = -1f;
        sceneManager.ResetScene();
    }

    // Randomise Next Episode Parameters
    void RandomizeEpisodeVariables()
    {
        sceneManager.RandomizeAgent();
        sceneManager.RandomizeScene_ExpertPush();
    }

    // Returns Filtered Position via a Low-Pass Filter (Optional)
    Vector3 LowPassFilteredPosition(Vector3 unfilteredPos, int[] parsedIndeces)
    {
        if (!applyFilter)
            return unfilteredPos;

        Vector3 filteredPos = new Vector3(
            ReturnFilteredFloat(unfilteredPos.x, parsedIndeces[0]),
            ReturnFilteredFloat(unfilteredPos.y, parsedIndeces[1]),
            ReturnFilteredFloat(unfilteredPos.z, parsedIndeces[2]));
        return filteredPos;
    }

    float ReturnFilteredFloat(float unfilteredValue, int index)
    {
        lowPassFilter[index].UpdateFilter(unfilteredValue);
        float filteredValue = lowPassFilter[index].filteredValue;
        return filteredValue;
    }
}

/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: The Main Code for the Expert Responsible of Picking & Dropping    *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using MLAgents.Sensors;
using System.IO;

public class ExpertPickDrop : Agent
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
    public Transform boxTopTargetTR;
    public Transform unpackingLocationTR;
    public ObjectInCollisionWith unpackLocationInCollisionWithBoxTopTarget;

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
    private Vector3 boxTargetPosWithNoise, unpackLocationPosWithNoise;

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
        boxTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(boxTopTargetTR.localPosition, environmentNoise.posNoiseMeters);
        boxTargetPosWithNoise = LowPassFilteredPosition(boxTargetPosWithNoise, new int[] { 0, 1, 2 });
        sensor.AddObservation(boxTargetPosWithNoise);
        unpackLocationPosWithNoise = GaussianDistribution.PositionWGaussianNoise(unpackingLocationTR.localPosition, environmentNoise.posNoiseMeters);
        unpackLocationPosWithNoise = LowPassFilteredPosition(unpackLocationPosWithNoise, new int[] { 3, 4, 5 });
        sensor.AddObservation(unpackLocationPosWithNoise);
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
    public bool boxTopInCollisionWithUnpackLocation = false;
    public float distanceAgentToBoxTopTargetNorm, distanceBoxTopTargetToUnpackLocationNorm, distanceAgentToInitPosNorm;
    public float maxDistanceAgentToBoxTopTarget = -1f, maxDistanceBoxTopTargetToUnpackLocation = -1f, maxDistanceAgentToInitPos = -1f;
    [System.NonSerialized]
    public float distanceToBase = 0.0f;
    void AgentSpecificReward()
    {
        // R1: Agent Distance to Box Cover
        float distanceAgentToBoxTopTarget = Vector3.Distance(agentTR.localPosition, boxTargetPosWithNoise);
        if (maxDistanceAgentToBoxTopTarget < 0)
            maxDistanceAgentToBoxTopTarget = distanceAgentToBoxTopTarget;
        distanceAgentToBoxTopTargetNorm = Mathf.Clamp(distanceAgentToBoxTopTarget / maxDistanceAgentToBoxTopTarget, 0, 1);
        AddReward((1 - distanceAgentToBoxTopTargetNorm) * 0.0005f * speed * (!boxTopInCollisionWithUnpackLocation ? 1 : 0));

        // R2: Force Sensor Based Reward
        if (!gripperState.openGripper && forceFingerA.collisionName == "[Target Dynamic] Box - Top" || forceFingerB.collisionName == "[Target Dynamic] Box - Top")
            AddReward((forceFingerA.fNorm + forceFingerB.fNorm) * 0.0005f * speed * (!boxTopInCollisionWithUnpackLocation ? 1 : 0));

        // R3: Box Cover in Designated Unpacking Location
        float distanceBoxTopTargetToUnpackingLocation = Vector3.Distance(boxTargetPosWithNoise, unpackLocationPosWithNoise);
        if (maxDistanceBoxTopTargetToUnpackLocation < 0)
            maxDistanceBoxTopTargetToUnpackLocation = distanceBoxTopTargetToUnpackingLocation;
        distanceBoxTopTargetToUnpackLocationNorm = Mathf.Clamp(distanceBoxTopTargetToUnpackingLocation / maxDistanceBoxTopTargetToUnpackLocation, 0, 1);
        AddReward((1 - distanceBoxTopTargetToUnpackLocationNorm) * 0.001f * speed * (!boxTopInCollisionWithUnpackLocation ? 1 : 0));
        boxTopInCollisionWithUnpackLocation = unpackLocationInCollisionWithBoxTopTarget.inContactWithTarget &&
            unpackLocationInCollisionWithBoxTopTarget.gameObjectInCollision.name == "[Target Dynamic] Box - Top";
        AddReward((boxTopInCollisionWithUnpackLocation ? 1 : 0) * 0.005f * speed);

        // R4: Distance to Initial Position
        float distanceAgentToInitPos = Vector3.Distance(agentTR.localPosition, agentDefaultInitPos);
        if (maxDistanceAgentToInitPos < 0)
            maxDistanceAgentToInitPos = distanceAgentToInitPos;
        distanceAgentToInitPosNorm = Mathf.Clamp(distanceAgentToInitPos / maxDistanceAgentToInitPos, 0, 1);
        AddReward((1 - distanceAgentToInitPosNorm) * 0.0005f * speed * (boxTopInCollisionWithUnpackLocation ? 1 : 0));
        distanceToBase = distanceAgentToInitPos;

        // Primary Goal is Achieved and Agent is Wihin Initial Position
        if (distanceAgentToInitPos < 0.1f && boxTopInCollisionWithUnpackLocation && (StepCount >= (maxStep / 10)) && !agentControlled)
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
        maxDistanceAgentToBoxTopTarget = -1f;
        maxDistanceBoxTopTargetToUnpackLocation = -1f;
        maxDistanceAgentToInitPos = -1f;
        sceneManager.ResetScene();
    }

    // Randomise Next Episode Parameters
    void RandomizeEpisodeVariables()
    {
        sceneManager.RandomizeAgent();
        sceneManager.RandomizeScene_ExpertPickDrop();
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

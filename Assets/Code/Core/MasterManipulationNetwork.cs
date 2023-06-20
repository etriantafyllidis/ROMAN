/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: The Main Code for the Gating Network (Manipulation Network)       *
 ******************************************************************************/

using MLAgents;
using MLAgents.Sensors;
using UnityEngine;

public class MasterManipulationNetwork : Agent
{
    [Header("Agent Experts")]
    public ExpertPull expertPull;
    public ExpertPickDrop expertPickDrop;
    public ExpertRotateOpen expertRotateOpen;
    public ExpertPickPlace expertPickPlace;
    public ExpertPickInsert expertPickInsert;
    public ExpertPush expertPush;
    public ExpertButton expertButton;

    [Header("Agent Settings")]
    public int stepSafetySync = 320;
    public float speed = 1f;

    [Header("Object Assignments")]
    public SceneManagement sceneManager;
    public Transform agentTR;

    [Header("Expert Visualisation")]
    public ExpertWeightVisualisation expertVis;

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

    [System.NonSerialized]
    public ForceReading forceFingerA;
    [System.NonSerialized]
    public ForceReading forceFingerB;

    private float actualTime;
    private Rigidbody agentRB;
    private GripperState gripperState;
    private Vector3 agentDefaultInitPos;

    void Awake()
    {
        expertPull.agentControlled = true;
        expertPickDrop.agentControlled = true;
        expertRotateOpen.agentControlled = true;
        expertPickPlace.agentControlled = true;
        expertPickInsert.agentControlled = true;
        expertPush.agentControlled = true;
        expertButton.agentControlled = true;

        expertPull.stepSafetySync = stepSafetySync;
        expertPickDrop.stepSafetySync = stepSafetySync;
        expertRotateOpen.stepSafetySync = stepSafetySync;
        expertPickPlace.stepSafetySync = stepSafetySync;
        expertPickInsert.stepSafetySync = stepSafetySync;
        expertPush.stepSafetySync = stepSafetySync;
        expertButton.stepSafetySync = stepSafetySync;
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
        episodeHasResetForButtonPress = true;
        ResetEpisodeParameters();
        RandomizeEpisodeVariables();
        episode++;
    }

    // Objects of Interest (OIs)
    [System.NonSerialized]
    public Vector3 handleTargetPosWithNoise; // Pull
    [System.NonSerialized]
    public Vector3 boxTargetPosWithNoise; // Pick & Drop
    [System.NonSerialized]
    public Vector3 doorHandleTargetPosWithNoise; // Rotate Open
    [System.NonSerialized]
    public Vector3 rackTargetPosWithNoise; // Pick & Place / Push
    [System.NonSerialized]
    public Vector3 vialTargetPosWithNoise; // Pick & Insert
    [System.NonSerialized]
    public Vector3 buttonTargetPosWithNoise; // Button

    // State Space
    public override void CollectObservations(VectorSensor sensor)
    {
        // Input: Robot Proprioception (6)
        sensor.AddObservation(agentTR.localPosition);
        sensor.AddObservation(agentRB.velocity);

        // Input: Robot Force Sensor (2)
        sensor.AddObservation(forceFingerA.fNorm);
        sensor.AddObservation(forceFingerB.fNorm);

        // Input: Noisy Environment Data (21)        
        // Expert: Pull
        handleTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(expertPull.drawerhandleTR.localPosition, environmentNoise.posNoiseMeters); // Add noise
        handleTargetPosWithNoise = LowPassFilteredPosition(handleTargetPosWithNoise, new int[] { 0, 1, 2 });
        sensor.AddObservation(handleTargetPosWithNoise);

        // Expert: Pick & Drop
        boxTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(expertPickDrop.boxTopTargetTR.localPosition, environmentNoise.posNoiseMeters); // Add noise
        boxTargetPosWithNoise = LowPassFilteredPosition(boxTargetPosWithNoise, new int[] { 3, 4, 5 });
        sensor.AddObservation(boxTargetPosWithNoise);

        // Expert: Rotate Open
        doorHandleTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(expertRotateOpen.doorHandleTargetTR.localPosition, environmentNoise.posNoiseMeters); // Add noise
        doorHandleTargetPosWithNoise = LowPassFilteredPosition(doorHandleTargetPosWithNoise, new int[] { 9, 10, 11 });
        sensor.AddObservation(doorHandleTargetPosWithNoise);

        // Expert: Pick & Place / Expert: Push
        rackTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(expertPickPlace.rackTargetTR.localPosition, environmentNoise.posNoiseMeters); // Add noise
        rackTargetPosWithNoise = LowPassFilteredPosition(rackTargetPosWithNoise, new int[] { 12, 13, 14 });
        sensor.AddObservation(rackTargetPosWithNoise);

        // Expert: Pick & Insert
        vialTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(expertPickInsert.vialTargetTR.localPosition, environmentNoise.posNoiseMeters); // Add noise
        vialTargetPosWithNoise = LowPassFilteredPosition(vialTargetPosWithNoise, new int[] { 15, 16, 17 });
        sensor.AddObservation(vialTargetPosWithNoise);

        // Expert: Button
        buttonTargetPosWithNoise = GaussianDistribution.PositionWGaussianNoise(expertButton.buttonTargetTR.localPosition, environmentNoise.posNoiseMeters); // Add noise
        buttonTargetPosWithNoise = LowPassFilteredPosition(buttonTargetPosWithNoise, new int[] { 18, 19, 20 });
        sensor.AddObservation(buttonTargetPosWithNoise);
        sensor.AddObservation(expertButton.buttonFunction.buttonActivated);

        // Agent to Initial Position
        sensor.AddObservation(distanceAgentToInitPosNorm);
    }

    // Applied Actions (Weights)
    [System.NonSerialized]
    public float [] vectorActionReadOnly = new float[7];

    // Action Space
    public override void OnActionReceived(float[] vectorAction)
    {
        // Clamp Actions 0->1
        vectorAction[0] = Mathf.Clamp(vectorAction[0], 0, 1);
        vectorAction[1] = Mathf.Clamp(vectorAction[1], 0, 1);
        vectorAction[2] = Mathf.Clamp(vectorAction[2], 0, 1);
        vectorAction[3] = Mathf.Clamp(vectorAction[3], 0, 1);
        vectorAction[4] = Mathf.Clamp(vectorAction[4], 0, 1);
        vectorAction[5] = Mathf.Clamp(vectorAction[5], 0, 1);
        vectorAction[6] = Mathf.Clamp(vectorAction[6], 0, 1);
        for (int i = 0; i < vectorAction.Length; i++)
            vectorAction[i] *= 10;

        // Use Specified Activation Function
        vectorAction = ActivationFunction.softMax(vectorAction);

        // Apply Corresponding Weights to Experts
        expertPull.agentWeight = vectorAction[0];
        expertPickDrop.agentWeight = vectorAction[1];
        expertRotateOpen.agentWeight = vectorAction[2];
        expertPickPlace.agentWeight = vectorAction[3];
        expertPickInsert.agentWeight = vectorAction[4];
        expertPush.agentWeight = vectorAction[5];
        expertButton.agentWeight = vectorAction[6];

        // Apply Agent-Specific Rewards
        if (StepCount >= (maxStep / (maxStep <= 80000 ? 100 : stepSafetySync))) 
            AgentSpecificReward(vectorAction);

        // For Information and Debugging
        totalReward = GetCumulativeReward();
        PrintEpisodeInfo(false);

        // Terminate Episode if Needed
        if ((StepCount >= (maxStep / 10)) && AtLeastOneExpertHasFinished())
            TerminateAll();

        // Visualise the Weight Assignments if Needed
        if (expertVis && expertVis.gameObject.activeInHierarchy)
            VisualiseExpertWeights(vectorAction);

        // Display the Weights
        vectorActionReadOnly = vectorAction;
    }

    // Agent Specific Rewards
    public float totalReward = 0;
    public float distanceAgentToInitPosNorm;
    public float[] r;// wR;
    public float rackToConveyorDistanceNorm = 0.0f;
    private float distanceAgentToInitPos, maxDistanceAgentToInitPos = -1f;
    private const float maxRackConveyorDistance = 0.3f;
    void AgentSpecificReward(float[] vectorAction)
    {
        // Primary Rewards
        r = new float[7];

        // R1: Pull 
        r[0] = expertPull.drawerOpenDistanceNorm;
        // R2: Pick & Drop
        r[1] = expertPickDrop.boxTopInCollisionWithUnpackLocation ? 1 : 0;
        // R3: Rotate Open
        r[2] = expertRotateOpen.doorHingeOpenAngleNorm;
        // R4: Pick & Place
        bool vialPlacedInRackAndPushed = rackToConveyorDistanceNorm <= 0.15f && expertPickInsert.vialTargetInCollsionWithRack;
        r[3] = expertPickPlace.rackInContactWithRack ? 1 : (vialPlacedInRackAndPushed ? 1 : 0);
        // R5: Pick & Insert
        r[4] = expertPickInsert.vialTargetInCollsionWithRack ? 1 : 0;
        // R6: Push
        rackToConveyorDistanceNorm = Mathf.Clamp((expertPush.distanceRackToEndLocation / maxRackConveyorDistance), 0, 1);
        r[5] = 1f - rackToConveyorDistanceNorm;
        // R7: Button
        r[6] = expertButton.buttonFunction.buttonActivated ? 1 : 0;

        for (int i = 0; i < r.Length; i++)
        {
            r[i] *= 0.001f;
            AddReward(r[i]);
        }

        // Distance to Initial Position
        distanceAgentToInitPos = Vector3.Distance(agentTR.localPosition, agentDefaultInitPos);
        if (maxDistanceAgentToInitPos < 0)
            maxDistanceAgentToInitPos = distanceAgentToInitPos;
        distanceAgentToInitPosNorm = Mathf.Clamp(distanceAgentToInitPos / maxDistanceAgentToInitPos, 0, 1);

        // Terminate if Primary Goal(-s) are Satisified
        // (i) Vial is Inside Rack, 
        // (ii) Rack is Pushed onto Converyor Belt,
        // (iii) Button is Pressed 
        // (iv) Agent has Returned to Initial Position
        if (expertPickInsert.vialTargetInCollsionWithRack &&
            rackToConveyorDistanceNorm <= 0.15f &&
            expertButton.buttonFunction.buttonActivated &&
            distanceAgentToInitPos <= 0.3f
            && (StepCount >= (maxStep / 100)))
        {
            SetReward(1000f);
            PrintEpisodeInfo(true);
            TerminateAll();
        }
    }

    // For Printing and Debugging Purposes
    private int printCount = 1;
    void PrintEpisodeInfo(bool invariantStepCount)
    {
        if (currentlyHeuristic && (invariantStepCount || (StepCount >= 1000 * printCount)))
        {
            printCount++;
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
        // Numpads "[ExpertID]"
        heuristicActions = new float[7];
        heuristicActions[0] = Input.GetAxis("E1");
        heuristicActions[1] = Input.GetAxis("E2");
        heuristicActions[2] = Input.GetAxis("E3");
        heuristicActions[3] = Input.GetAxis("E4");
        heuristicActions[4] = Input.GetAxis("E5");
        heuristicActions[5] = Input.GetAxis("E6");
        heuristicActions[6] = Input.GetAxis("E7");

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
        if (Input.GetButton("E1") || Input.GetButton("E2") || Input.GetButton("E3") || Input.GetButton("E4") ||
            Input.GetButton("E5") || Input.GetButton("E6") || Input.GetButton("E7"))
            return true;
        return false;
    }

    bool AtLeastOneExpertHasFinished()
    {
        return (expertPull.m_Info.done || expertPickDrop.m_Info.done || expertRotateOpen.m_Info.done ||
                expertPickPlace.m_Info.done || expertPickInsert.m_Info.done || expertPush.m_Info.done || expertButton.m_Info.done);
    }

    void TerminateAll()
    {
        expertPull.EndEpisode();
        expertPickDrop.EndEpisode();
        expertRotateOpen.EndEpisode();
        expertPickPlace.EndEpisode();
        expertPickInsert.EndEpisode();
        expertPush.EndEpisode();
        expertButton.EndEpisode();
        EndEpisode();
    }

    void ResetEpisodeParameters()
    {
        // Reset Scene Objects Positions and Velocities
        sceneManager.ResetScene();
        maxDistanceAgentToInitPos = -1f;
        printCount = 1;

        // Pull
        expertPull.maxDistanceAgentToHandle = -1f;
        expertPull.maxDistanceAgentToInitPos = -1f;

        // Pick & Drop
        expertPickDrop.maxDistanceAgentToBoxTopTarget = -1f;
        expertPickDrop.maxDistanceBoxTopTargetToUnpackLocation = -1f;
        expertPickDrop.maxDistanceAgentToInitPos = -1f;

        // Rotate & Open
        expertRotateOpen.maxDistanceAgentToDoorHandle = -1f;
        expertRotateOpen.maxDistanceAgentToInitPos = -1f;

        // Pick & Place
        expertPickPlace.maxDistanceAgentToRack = -1f;
        expertPickPlace.maxDistanceRackToRackLocation = -1f;
        expertPickPlace.maxDistanceAgentToInitPos = -1f;

        // Pick & Insert
        expertPickInsert.maxDistanceAgentToVialTarget = -1f;
        expertPickInsert.maxDistanceVialTargetToRackTarget = -1f;
        expertPickInsert.maxDistanceAgentToInitPos = -1f;
        expertPickInsert.vialHeightPenalizedThisEpisode = false;

        // Push
        expertPush.maxDistanceAgentToRack = -1f;
        expertPush.maxDistanceRackToEndLocation = -1f;
        expertPush.maxDistanceAgentToInitPos = -1f;

        // Button
        expertButton.maxDistanceAgentToButton = -1f;
        expertButton.maxDistanceAgentToInitPos = -1f;
        expertButton.buttonFunction.buttonActivated = false;
    }

    // Randomise Next Episode Parameters
    [System.NonSerialized]
    public int scenarioCase = -1;
    void RandomizeEpisodeVariables()
    {
        sceneManager.RandomizeAgent();
        scenarioCase = Random.Range(0,7);
        sceneManager.RandomizeScene_ManipulationNetwork(scenarioCase);
    }

    // Visualise Expert Weights (Optional)
    void VisualiseExpertWeights(float[] vectorAction)
    {
        if (expertVis)
        {
            expertVis.weightsExperts[0] = Mathf.Clamp(vectorAction[0], 0, 1);
            expertVis.weightsExperts[1] = Mathf.Clamp(vectorAction[1], 0, 1);
            expertVis.weightsExperts[2] = Mathf.Clamp(vectorAction[2], 0, 1);
            expertVis.weightsExperts[3] = Mathf.Clamp(vectorAction[3], 0, 1);
            expertVis.weightsExperts[4] = Mathf.Clamp(vectorAction[4], 0, 1);
            expertVis.weightsExperts[5] = Mathf.Clamp(vectorAction[5], 0, 1);
            expertVis.weightsExperts[6] = Mathf.Clamp(vectorAction[6], 0, 1);
        }
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

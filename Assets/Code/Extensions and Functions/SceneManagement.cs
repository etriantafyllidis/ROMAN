/******************************************************************************
 * RObotic MAnipulation Network (ROMAN)                                       *
 * Hybrid Hierarchical Learning for Solving Complex Sequential Tasks          *
 * -------------------------------------------------------------------------- *
 * Purpose: Manages the Entire Scene (Whether in Inference or Not)            *
 *          Initialises, Sets, Resets and Randomises all Relevant Objects     *
 *          Public Variables and Functions are Accessible by all NN of ROMAN  *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SceneManagement : MonoBehaviour
{
    public Transform agentTR, drawerHandleTR, drawerTR,
    handleDoorTR, doorHingeTR,
        boxTopTR, vialTR, rackTR, buttonTR;

    private Rigidbody agentRB, drawerHandleRB, drawerRB,
        handleDoorRB, doorHingeRB,
        boxTopRB, vialRB, rackRB, buttonRB;
    public Vector3 agentInitPos, drawerHandleInitPos, drawerInitPos,
        handleDoorInitPos, doorHingeInitPos,
        boxTopInitPos, vialInitPos, rackInitPos, buttonInitPos;
    public Quaternion agentInitRot, drawerHandleInitRot, drawerInitRot,
        handleDoorInitRot, doorHingeInitRot,
        boxTopInitRot, vialInitRot, rackInitRot, buttonInitRot;

    [System.NonSerialized]
    public ConfigurableJoint drawerCJ;
    [System.NonSerialized]
    public ConfigurableJoint doorHingeCJ;

    void Start()
    {
        // Save Positions
        {
            agentInitPos = agentTR.localPosition;
            drawerHandleInitPos = drawerHandleTR.localPosition;
            drawerInitPos = drawerTR.localPosition;
            handleDoorInitPos = handleDoorTR.localPosition;
            doorHingeInitPos = doorHingeTR.localPosition;
            boxTopInitPos = boxTopTR.localPosition;
            vialInitPos = vialTR.localPosition;
            rackInitPos = rackTR.localPosition;
            buttonInitPos = buttonTR.localPosition;
        }

        // Save Rotations
        {
            agentInitRot = agentTR.localRotation;
            drawerHandleInitRot = drawerHandleTR.localRotation;
            drawerInitRot = drawerTR.localRotation;
            handleDoorInitRot = handleDoorTR.localRotation;
            doorHingeInitRot = doorHingeTR.localRotation;
            boxTopInitRot = boxTopTR.localRotation;
            vialInitRot = vialTR.localRotation;
            rackInitRot = rackTR.localRotation;
            buttonInitRot = buttonTR.localRotation;
        }

        // Retrieve Rigidbodies
        {
            agentRB = agentTR.GetComponent<Rigidbody>();
            drawerHandleRB = drawerHandleTR.GetComponent<Rigidbody>();
            drawerRB = drawerTR.GetComponent<Rigidbody>();
            handleDoorRB = handleDoorTR.GetComponent<Rigidbody>();
            doorHingeRB = doorHingeTR.GetComponent<Rigidbody>();
            boxTopRB = boxTopTR.GetComponent<Rigidbody>();
            vialRB = vialTR.GetComponent<Rigidbody>();
            rackRB = rackTR.GetComponent<Rigidbody>();
            buttonRB = buttonTR.GetComponent<Rigidbody>();
        }

        // Retrieve Configurable Joints where Applicable
        {
            drawerCJ = drawerTR.GetComponent<ConfigurableJoint>();
            doorHingeCJ = doorHingeTR.GetComponent<ConfigurableJoint>();
        }

        ConfigureConfigurableJoints();
    }

    // Reconfigures all CJ that have AutoConfigureConnectedAnchor OFF
    // This is in Respect to the World Position of the Parent Environment
    void ConfigureConfigurableJoints()
    {
        Vector3 newlyConnectedWorldAnchor = new Vector3(
            this.transform.position.x,
            this.transform.position.y - 2f,
            this.transform.position.z);

        if (!drawerCJ.autoConfigureConnectedAnchor)
            drawerCJ.connectedAnchor += newlyConnectedWorldAnchor;

        if (!doorHingeCJ.autoConfigureConnectedAnchor)
            doorHingeCJ.connectedAnchor += newlyConnectedWorldAnchor;
    }

    // Reset all Objects Including the Agent in the Scene
    public void ResetScene()
    {
        // Reset Velocities
        agentRB.velocity = Vector3.zero;
        drawerHandleRB.velocity = Vector3.zero;
        drawerRB.velocity = Vector3.zero;
        handleDoorRB.velocity = Vector3.zero;
        doorHingeRB.velocity = Vector3.zero;
        boxTopRB.velocity = Vector3.zero;
        vialRB.velocity = Vector3.zero;
        rackRB.velocity = Vector3.zero;
        buttonRB.velocity = Vector3.zero;

        // Reset Angular Velocities
        agentRB.angularVelocity = Vector3.zero;
        drawerHandleRB.angularVelocity = Vector3.zero;
        drawerRB.angularVelocity = Vector3.zero;
        handleDoorRB.angularVelocity = Vector3.zero;
        doorHingeRB.angularVelocity = Vector3.zero;
        boxTopRB.angularVelocity = Vector3.zero;
        vialRB.angularVelocity = Vector3.zero;
        rackRB.angularVelocity = Vector3.zero;
        buttonRB.angularVelocity = Vector3.zero;

        // Reset Positions
        agentTR.localPosition = agentInitPos;
        drawerHandleTR.localPosition = drawerHandleInitPos;
        drawerTR.localPosition = drawerInitPos;
        handleDoorTR.localPosition = handleDoorInitPos;
        doorHingeTR.localPosition = doorHingeInitPos;
        boxTopTR.localPosition = boxTopInitPos;
        vialTR.localPosition = vialInitPos;
        rackTR.localPosition = rackInitPos;
        buttonTR.localPosition = buttonInitPos;

        // Reset Rotations
        agentTR.localRotation = agentInitRot;
        drawerHandleTR.localRotation = drawerHandleInitRot;
        drawerTR.localRotation = drawerInitRot;
        handleDoorTR.localRotation = handleDoorInitRot;
        doorHingeTR.localRotation = doorHingeInitRot;
        boxTopTR.localRotation = boxTopInitRot;
        vialTR.localRotation = vialInitRot;
        rackTR.localRotation = rackInitRot;
        buttonTR.localRotation = buttonInitRot;
    }

    // Called by all Involved NNs of ROMAN
    public void RandomizeAgent()
    {
        // Randomize Agent
        Vector3 random = new Vector3(Random.Range(-0.3f, 0.3f), Random.Range(-0.2f, 0.2f), Random.Range(-0.3f, 0.3f));
        agentTR.localPosition = new Vector3(random.x + agentInitPos.x, random.y + agentInitPos.y, random.z + agentInitPos.z);
    }

    // Randomise Scene-Related Variables of the Pull Expert
    public void RandomizeScene_ExpertPull()
    {
        // Relevant Objects: Drawer Handle, Drawer, Vial, Box Cover
        float drawerLinearLimit = drawerCJ.linearLimit.limit * 2;
        float drawerLinearDistance = Random.Range(-drawerLinearLimit * 0.25f, 0.0f);
        drawerTR.localPosition = new Vector3(drawerInitPos.x,
            drawerInitPos.y, drawerInitPos.z + drawerLinearDistance);
        drawerHandleTR.localPosition = new Vector3(drawerHandleInitPos.x,
            drawerHandleInitPos.y, drawerHandleInitPos.z + drawerLinearDistance);
        boxTopTR.localPosition = new Vector3(boxTopInitPos.x,
            boxTopInitPos.y, boxTopInitPos.z + drawerLinearDistance);

        vialTR.localPosition = new Vector3(vialInitPos.x,
            vialInitPos.y, vialInitPos.z + drawerLinearDistance);
        ScenarioSpecific_RandomizeVialInsideDrawer();
    }

    // Randomise Scene-Related Variables of the Pick & Drop Expert
    public void RandomizeScene_ExpertPickDrop()
    {
        // Relevant Objects: Drawer Handle, Drawer, Vial, Box Cover
        float drawerLinearLimit = drawerCJ.linearLimit.limit * 2;
        float drawerLinearDistance = Random.Range(-drawerLinearLimit, -drawerLinearLimit * 0.9f);
        drawerTR.localPosition = new Vector3(drawerInitPos.x,
            drawerInitPos.y, drawerInitPos.z + drawerLinearDistance);
        drawerHandleTR.localPosition = new Vector3(drawerHandleInitPos.x,
            drawerHandleInitPos.y, drawerHandleInitPos.z + drawerLinearDistance);
        boxTopTR.localPosition = new Vector3(boxTopInitPos.x,
            boxTopInitPos.y, boxTopInitPos.z + drawerLinearDistance);
        vialTR.localPosition = new Vector3(vialInitPos.x,
            vialInitPos.y, vialInitPos.z + drawerLinearDistance);
        ScenarioSpecific_RandomizeVialInsideDrawer();
    }

    // Randomise Scene-Related Variables of the Pick & Insert Expert
    public void RandomizeScene_ExpertPickInsert()
    {
        // Relevant Ojects: Drawer Handle, Drawer, Vial, Box Cover, Rack
        float drawerLinearLimit = drawerCJ.linearLimit.limit * 2;
        float drawerLinearDistance = Random.Range(-drawerLinearLimit, -drawerLinearLimit * 0.9f);
        drawerTR.localPosition = new Vector3(drawerInitPos.x,
            drawerInitPos.y, drawerInitPos.z + drawerLinearDistance);
        drawerHandleTR.localPosition = new Vector3(drawerHandleInitPos.x,
            drawerHandleInitPos.y, drawerHandleInitPos.z + drawerLinearDistance);

        ScenarioSpecific_RandomizeBoxCoverPlacement();

        vialTR.localPosition = new Vector3(vialInitPos.x,
            vialInitPos.y, vialInitPos.z + drawerLinearDistance);
        ScenarioSpecific_RandomizeVialInsideDrawer();

        float angularLimit = doorHingeCJ.highAngularXLimit.limit;
        float randomizedHingleAngle = Random.Range(angularLimit * 0.9f, angularLimit);
        Vector3 eulerChange =
            new Vector3(doorHingeTR.localRotation.eulerAngles.x,
            doorHingeTR.localRotation.eulerAngles.y + randomizedHingleAngle,
            doorHingeTR.localRotation.eulerAngles.z);
        doorHingeTR.localEulerAngles = eulerChange;

        ScenarioSpecific_RandomizeRackOutsideCabinet();
    }

    // Randomise Scene-Related Variables of the Rotate Open Expert
    public void RandomizeScene_ExpertRotateOpen()
    {
        // Relevant Ojects: All of Pick & Drop Expert + Cabinet Door Hinge Angle
        RandomizeScene_ExpertPickDrop();

        float angularLimit = doorHingeCJ.highAngularXLimit.limit;
        float randomizedHingleAngle = Random.Range(0f, angularLimit * 0.1f);
        Vector3 eulerChange =
            new Vector3(doorHingeTR.localRotation.eulerAngles.x,
            doorHingeTR.localRotation.eulerAngles.y + randomizedHingleAngle,
            doorHingeTR.localRotation.eulerAngles.z);
        doorHingeTR.localEulerAngles = eulerChange;
    }

    // Randomise Scene-Related Variables of the Pick & Place Expert
    public void RandomizeScene_ExpertPickPlace()
    {
        // Relevant Ojects: All of Pick & Drop Expert + Box Cover, Cabinet Door
        RandomizeScene_ExpertPickDrop();
       
        ScenarioSpecific_RandomizeBoxCoverPlacement();

        float angularLimit = doorHingeCJ.highAngularXLimit.limit;
        float randomizedHingleAngle = Random.Range(angularLimit * 0.85f, angularLimit);
        Vector3 eulerChange =
            new Vector3(doorHingeTR.localRotation.eulerAngles.x,
            doorHingeTR.localRotation.eulerAngles.y + randomizedHingleAngle,
            doorHingeTR.localRotation.eulerAngles.z);
        doorHingeTR.localEulerAngles = eulerChange;
    }

    // Randomise Scene-Related Variables of the Push Expert
    public void RandomizeScene_ExpertPush()
    {
        // Relevant Ojects: Drawer, Drawer Handle, Box Cover, Vial, Rack, Cabinet Door
        float drawerLinearLimit = drawerCJ.linearLimit.limit * 2;
        float drawerLinearDistance = Random.Range(-drawerLinearLimit, -drawerLinearLimit * 0.9f);
        drawerTR.localPosition = new Vector3(drawerInitPos.x,
            drawerInitPos.y, drawerInitPos.z + drawerLinearDistance);
        drawerHandleTR.localPosition = new Vector3(drawerHandleInitPos.x,
            drawerHandleInitPos.y, drawerHandleInitPos.z + drawerLinearDistance);

        ScenarioSpecific_RandomizeBoxCoverPlacement();

        vialRB.velocity = Vector3.zero;
        vialRB.angularVelocity = Vector3.zero;
        rackRB.velocity = Vector3.zero;
        rackRB.angularVelocity = Vector3.zero;

        float angularLimit = doorHingeCJ.highAngularXLimit.limit;
        float randomizedHingleAngle = Random.Range(angularLimit * 0.9f, angularLimit);
        Vector3 eulerChange =
            new Vector3(doorHingeTR.localRotation.eulerAngles.x,
            doorHingeTR.localRotation.eulerAngles.y + randomizedHingleAngle,
            doorHingeTR.localRotation.eulerAngles.z);
        doorHingeTR.localEulerAngles = eulerChange;

        ScenarioSpecific_RandomizeRackAndVialOutsideDrawer();
    }

    // Randomise Scene-Related Variables of the Push-Button Expert
    public void RandomizeScene_ExpertButton()
    {
        // Relevant Ojects: All of Push Expert + Rack and Vial Location
        RandomizeScene_ExpertPush();

        float maxDistanceTowardsConveyor = 0.3f;
        float randomizedDistance = Random.Range(-maxDistanceTowardsConveyor, -maxDistanceTowardsConveyor * 0.9f);
        rackTR.localPosition =
            new Vector3(rackTR.localPosition.x,
            rackTR.localPosition.y,
            rackTR.localPosition.z + randomizedDistance);
        vialTR.localPosition =
            new Vector3(vialTR.localPosition.x,
            vialTR.localPosition.y,
            vialTR.localPosition.z + randomizedDistance);
    }

    // Randomise Scene-Related Variables of the Manipulation Network (Master / Gating NN)
    public void RandomizeScene_ManipulationNetwork(int caseScenario)
    {
        switch (caseScenario)
        {
            // Needed: Push-Button
            case 0: 
                RandomizeScene_ExpertButton();
                break;

            // Needed: Push + Push-Button
            case 1: 
                RandomizeScene_ExpertPush();
                break;

            // Needed: Pick & Insert + Push + Push-Button
            case 2: 
                RandomizeScene_ExpertPickInsert();
                break;

            // Needed: Pick & Place + Pick & Insert + Push + Push-Button
            case 3: 
                RandomizeScene_ExpertPickPlace();
                break;

            // Needed: Rotate Open + Pick & Place + Pick & Insert + Push + Push-Button
            case 4: 
                RandomizeScene_ExpertRotateOpen();
                ScenarioSpecific_RandomizeBoxCoverPlacement();
                break;

            // Needed: Pick & Drop + Rotate Open + Pick & Place + Pick & Insert + Push + Push-Button
            case 5: 
                RandomizeScene_ExpertPickDrop();
                break;

            // Needed: Pull + Pick & Drop + Rotate Open + Pick & Place + Pick & Insert + Push + Push-Button
            case 6: 
                RandomizeScene_ExpertPull();
                break;
        }
    }

    // The Following Functions Randomize Object-Specific and Area-Wide Variables
    // These are set at Random Ranges Within the Specific Environment Training Workspace
    int vialHoleCounter = 0;

    // Randomise Vial Position within Drawer
    public void ScenarioSpecific_RandomizeVialInsideDrawer()
    {
        vialHoleCounter = -1;
        switch (vialHoleCounter)
        {
            case 0:
                vialTR.localPosition = new Vector3(-0.07474f, vialTR.localPosition.y, vialTR.localPosition.z);
                break;
            case 1:
                vialTR.localPosition = new Vector3(-0.0238f, vialTR.localPosition.y, vialTR.localPosition.z);
                break;
            case 2:
                vialTR.localPosition = new Vector3(0.028f, vialTR.localPosition.y, vialTR.localPosition.z);
                break;
            case 3:
                vialTR.localPosition = new Vector3(0.0814f, vialTR.localPosition.y, vialTR.localPosition.z);
                break;
        }
        vialHoleCounter = vialHoleCounter >= 3 ? 0 : vialHoleCounter + 1;
    }

    // Randomise Rack and Vial Position
    public void ScenarioSpecific_RandomizeRackAndVialOutsideDrawer()
    {
        float rackX = Random.Range(-0.05f, 0.05f);
        float rackZ = Random.Range(-0.02f, 0.02f);

        rackTR.localPosition =
            new Vector3(rackTR.localPosition.x + rackX,
            0.612f,
            rackTR.localPosition.z + rackZ);

        int vialHoleCase = Random.Range(0, 8);
        switch (vialHoleCase)
        {
            case 0:
                vialTR.localPosition = new Vector3(-0.1153f, 0.585f, -0.1862f);
                break;
            case 1:
                vialTR.localPosition = new Vector3(-0.0578f, 0.585f, -0.1862f);
                break;
            case 2:
                vialTR.localPosition = new Vector3(-0.0007f, 0.585f, -0.1862f);
                break;
            case 3:
                vialTR.localPosition = new Vector3(0.0593f, 0.585f, -0.1862f);
                break;
            case 4:
                vialTR.localPosition = new Vector3(-0.1153f, 0.585f, -0.2592f);
                break;
            case 5:
                vialTR.localPosition = new Vector3(-0.0578f, 0.585f, -0.2592f);
                break;
            case 6:
                vialTR.localPosition = new Vector3(-0.0007f, 0.585f, -0.2592f);
                break;
            case 7:
                vialTR.localPosition = new Vector3(0.0593f, 0.585f, -0.2592f);
                break;
        }
        vialTR.localPosition =
            new Vector3(vialTR.localPosition.x + rackX, vialTR.localPosition.y, vialTR.localPosition.z + rackZ);
    }

    // Randomise the Box Cover Placement
    public void ScenarioSpecific_RandomizeBoxCoverPlacement()
    {
        boxTopTR.localPosition = new Vector3(Random.Range(-0.1f, 0.1f), 0.575f, Random.Range(0.9f, 1.0f));
    }

    // Randomise the Rack within the Cabinet
    public void ScenarioSpecific_RandomizeRackInsideCabinet()
    {
        rackTR.localPosition =
            new Vector3(rackTR.localPosition.x + Random.Range(-0.03f, 0.03f),
            rackTR.localPosition.y,
            rackTR.localPosition.z);
    }

    // Randomise the Rack Outside of the Cabinet
    public void ScenarioSpecific_RandomizeRackOutsideCabinet()
    {
        float rackX = Random.Range(-0.05f, 0.05f);
        float rackZ = Random.Range(-0.025f, 0.025f);

        rackTR.localPosition =
            new Vector3(rackTR.localPosition.x + rackX,
            0.612f,
            rackTR.localPosition.z + rackZ);
    }
}

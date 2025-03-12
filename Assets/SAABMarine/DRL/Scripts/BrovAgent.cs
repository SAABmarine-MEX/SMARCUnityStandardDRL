using System;
using DefaultNamespace.LookUpTable;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Unity.Mathematics;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using VehicleComponents.Actuators;

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

//using DefaultNamespace;

namespace DefaultNamespace
{
public class BrovAgent : Agent
{
	//DefaultNamespace.BrovPhysics brovPhysics2 = new DefaultNamespace.BrovPhysics();
    private BrovPhysics brovPhysics;
    Vector3 inputForce = Vector3.zero;
    Vector3 inputTorque = Vector3.zero;
    private List<Vector3> gatePositions = new List<Vector3>();
    private List<Vector3> next2Gates = new List<Vector3>() { Vector3.zero, Vector3.zero }; // TODO: make this to be a set length of 2, array instead of list maybe?
    private int iNextGate = 1;

    // for continous rewards
    private Vector3 prevPos;
    Vector<float> prevActions = Vector<float>.Build.Dense(6, 0f);
    private ActionBuffers lastActions;


    
    // DRL training parameters
    private float gamma = 0.99f;
    private float epsilon = 0.2f;
    private float lambda1 = 1f, lambda2 = 0.02f, lambda3 = -10f, lambda4 = -2e-4f, lambda5 = -1e-4f;

    public override void Initialize()
    {
        print("Init");
        brovPhysics = GetComponent<BrovPhysics>();
if (brovPhysics == null)
{
    Debug.LogError("BrovPhysics component not found on " + gameObject.name);
}
else
{
    Debug.Log("Successfully found BrovPhysics on " + gameObject.name);
}
		print("!!!!!!!!!!FDA");
		print(brovPhysics.GetLocalPos());
        prevPos = brovPhysics.GetLocalPos();
        print("HHHHHHHHHHHHHHHHHHHHHHHHH");
        //Vector<float> prevActions = Vector<float>.Build.DenseOfEnumerable(prevActions);
        print(prevActions[0]);
        print(prevActions[1]);
        print(prevActions[2]);
        print(prevActions[3]);
        print(prevActions[4]);
        print(prevActions[5]);
        print("KKKKKKKKKKKKKKKKKKKKKK");
        

        GameObject gates = GameObject.Find("Gates");
        if (gates != null)
        {
            // Iterate over direct children of Gates. They are already sorted from Unity scene, i.e first child is first gate, second is second etc.
            foreach (Transform child in gates.transform)
            {
                Debug.Log("Found child: " + child.gameObject.name);
                Debug.Log("Child's position: " + child.localPosition);
                gatePositions.Add(child.localPosition);
                // You can also access the child object:
                GameObject childObject = child.gameObject;
            }
            Debug.Log("tot n:" + gatePositions.Count);
        }
        else
        {
            Debug.LogError("Gates object not found!");
        }
		/*
		Transform checkpointsTransform = transform.Find("Checkpoints");
		print("CHECKISAR");
        foreach (Transform checkpointSingleTransform in checkpointsTransform)
        { 
            Debug.Log(checkpointSingleTransform.name);
           	//checkpoints.Add(checkpointSingleTransform);
            
        }
		*/
    }
    public override void OnEpisodeBegin()
    {
        //print("New Episode");
        // Agent's starting state for the track
        Vector3 localPosition = new Vector3(-1.5f, -1f, -0.8f);
        Quaternion localRotation = Quaternion.Euler(0, -90, 0);
        brovPhysics.SetPosAndRot(localPosition, localRotation);

        // Reset next gate positions
        next2Gates[0] = gatePositions[0];
        next2Gates[1] = gatePositions[1];
        iNextGate = 1;
    }

    // Sensor/perception input for the agent
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(brovPhysics.GetLocalRot()); // Orientation quaternion
        // Velocities
        //Vector<float> vel_vec = brovPhysics.GetVelocity();
        sensor.AddObservation(brovPhysics.GetVelocity());
        Vector3 relVec2Gate1 = next2Gates[0] - brovPhysics.GetLocalPos();
        Vector3 relVec2Gate2 = next2Gates[1] - brovPhysics.GetLocalPos();
        sensor.AddObservation(relVec2Gate1); // Relative vector to next gate
        //sensor.AddObservation(relVec2Gate2); // Relative vector to second next gate
        //TODO: add prev actions as they did in drone report
        
        /*Reward stuff
            // TODO: have gates as Transform data type to be able to get alignment
            // Distance to next gate
            Vector3 directionToGate = nextGate.position - transform.position;
            float distanceToGate = directionToGate.magnitude;

            // Camera alignment (angle between drone forward direction and gate)
            float alignment = Vector3.Dot(transform.forward, directionToGate.normalized);
        */
    }

    // What actions the agent can preform
    public override void OnActionReceived(ActionBuffers actions)
    {
        lastActions = actions;
        // TODO: the input will be norm between -1 and 1. how to scale it correctly??
            //TODO: what is min and max forces for each dof?? put some arbatrary for now
        // Get the normalized continuous actions (values in [-1, 1])
        
        // x,y,z noted with irl coord sys
        float forceYNorm = actions.ContinuousActions[0];
        float forceZNorm = actions.ContinuousActions[1];
        float forceXNorm = actions.ContinuousActions[2];

        float torquePitchNorm = actions.ContinuousActions[3];
        float torqueYawNorm = actions.ContinuousActions[4];
        float torqueRollNorm = actions.ContinuousActions[5];
    

        // min max values for each dof
        int numActions = actions.ContinuousActions.Length;
        // min max ranges for each dof
        Vector2[] ranges = new Vector2[numActions];
        // TODO: add scaling given the Heuristic input values
        ranges[0] = new Vector2(-90f, 90f); // y
        ranges[1] = new Vector2(-50f, 50f); // z
        ranges[2] = new Vector2(-100f, 100f); // x
        ranges[3] = new Vector2(-30f, 30f); // pitch
        ranges[4] = new Vector2(-45f, 45f); // yaw
        ranges[5] = new Vector2(-60f, 60f); // roll

        // Create an array to hold the scaled values
        float[] scaledActions = new float[numActions];
        // Loop over 
        // Scale each continuous action using its specific range.
        for (int i = 0; i < numActions; i++)
        {
            float normalizedAction = actions.ContinuousActions[i]; // should be in [-1, 1]
            scaledActions[i] = ((normalizedAction + 1f) / 2f) * (ranges[i].y - ranges[i].x) + ranges[i].x;
        }
        //inputForce  = new Vector3(scaledActions[0], scaledActions[1], scaledActions[2]);
        //inputTorque = new Vector3(scaledActions[3], scaledActions[4], scaledActions[5]);


        // Suppose you want to map [-1, 1] to [minValue, maxValue]
        // TODO: do this for each dof
        float minValue = -90f;
        float maxValue = 90f;
        float scaledAction = ((forceXNorm + 1f) / 2f) * (maxValue - minValue) + minValue;
			
        // TODO: added the scaled actions below instead
        // TODO: why does this change the movement speed???
        inputForce  = new Vector3(actions.ContinuousActions[0], actions.ContinuousActions[1], actions.ContinuousActions[2]);
        inputTorque = new Vector3(actions.ContinuousActions[3], actions.ContinuousActions[4], actions.ContinuousActions[5]);

        brovPhysics.SetInput(inputForce, inputTorque);

        //float moveRotate = actions.ContinuousActions[0]; // X-axis rotation -1 - +1
        //float moveForward = actions.ContinuousActions[1]; // Z-axis movement -1 - +1
        //float moveVertical = actions.ContinuousActions[2]; // Y-axis movement -1 - +1

        // Forward/backward movement
        //mainBody.MovePosition(rb.position + transform.forward * moveForward * moveSpeed * Time.deltaTime);
        // Vertical movement (up/down)
        //rb.MovePosition(rb.position + Vector3.up * moveVertical * moveSpeed * Time.deltaTime);
        // Rotation
        //transform.Rotate(0f, moveRotate * moveSpeed, 0f, Space.Self);
    }
    // Teleop
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        //continuousActions[0] = Input.GetAxisRaw("Horizontal");
        //continuousActions[1] = Input.GetAxisRaw("Vertical");
        
        // TODO: would be clean if this only sets a force and that is then applied elsewhere
        // Reset input forces every fixed update
        //Vector3 inputForce = Vector3.zero;
        //Vector3 inputTorque = Vector3.zero;
        // Keyboard controlls
        if (Input.GetKey(KeyCode.W))
        {
            inputForce[2] += 86;
        }

        if (Input.GetKey(KeyCode.A))
        {
            inputForce[0] -= 85;
        }

        if (Input.GetKey(KeyCode.S))
        {
            inputForce[2] -= 85;
        }

        if (Input.GetKey(KeyCode.D))
        {
            inputForce[0] += 85;
        }

        if (Input.GetKey(KeyCode.Space))
        {
            inputForce[1] += 122;
        }

        if (Input.GetKey(KeyCode.LeftShift))
        {
            inputForce[1] -= 122;
        }

        if (Input.GetKey(KeyCode.Q))
        {
            inputTorque[1] -= 14;
        }

        if (Input.GetKey(KeyCode.E))
        {
            inputTorque[1] += 14;
        }

        if (Input.GetKey(KeyCode.X))
        {
            inputTorque[0] += 14;
        }

        if (Input.GetKey(KeyCode.C))
        {
            inputTorque[2] += 14;
        }
        // Forces
        continuousActions[0] = inputForce[0];
        continuousActions[1] = inputForce[1];
        continuousActions[2] = inputForce[2];
        // Torques
        continuousActions[3] = inputTorque[0];
        continuousActions[4] = inputTorque[1];
        continuousActions[5] = inputTorque[2];
    }
    private void ContinousRewards(ActionBuffers actions) // TODO: make sure it does this at each time step
    {
        // r_progression
        float d_prev = Vector3.Distance(next2Gates[0], prevPos);
        Vector3 currPos = brovPhysics.GetLocalPos();
        float d_curr = Vector3.Distance(next2Gates[0], currPos);
        float r_prog = lambda1 * (d_prev - d_curr);
        
        // r_perception
        Vector3 directionToGate = (next2Gates[0] - currPos).normalized;
        //float cameraAlignment = Vector3.Dot(brovPhysics.GetForwardUnitVec(), directionToGate);
        float angleToGate = Mathf.Acos(Vector3.Dot(brovPhysics.GetForwardUnitVec(), directionToGate)) * Mathf.Rad2Deg;
        float r_perc = lambda2 * Mathf.Exp(lambda3 * Mathf.Pow(angleToGate, 4));
        
        // r_command
        /*
        Vector<float> actionsVec = Vector<float>.Build.DenseOfArray(new float[]
        {
            v1.x, v1.y, v1.z,
            v2.x, v2.y, v2.z,
        };
        */
        // Convert ActionSegment<float> to Vector<float>
        //Vector<float> actionVector = Vector<float>.Build.DenseOfArray(continuousActions.ToArray());
        ActionSegment<float> continuousActions = actions.ContinuousActions;
        Vector<float> currActions = Vector<float>.Build.DenseOfEnumerable(continuousActions);
        //print("!!!CURR ACTIONS!!!");
        //print(currActions);
        //Vector<float> actionDiff = currActions.Subtract(prevActions);
        Vector<float> actionDiff = currActions - prevActions;
        float actionDiffNorm = (float) actionDiff.L2Norm();
        float magnitude = Mathf.Pow(actionDiffNorm, 2);
        //lambda5*((action_curr-action_prev).magnitude)², actions are "mass"-normalized. we dont have commanded velocities so that part is not used
        float r_cmd = lambda5*magnitude; 

        float r_t = r_prog + r_perc + r_cmd;
        AddReward(r_t);
        
        prevPos = currPos;
        prevActions = currActions;
    }
		// TODOS: 3. inför reward system varje check point (se videon för det)
        // Collision handeling, and rewards
    private void OnTriggerEnter(Collider other)
    {
		// Try to get the CheckpointData component from the collider.
    	CheckpointSingle cpData = other.GetComponent<CheckpointSingle>();
    	if (cpData != null)
    	{
			Debug.Log("CHECKPOINT INDEX: " + cpData.checkpointIndex);
			Debug.Log("CORRECT INDEX: " + iNextGate);
        	// Optionally, verify the checkpoint order.
        	if (cpData.checkpointIndex == iNextGate)
        	{
				Debug.Log("RÄTT ORDNING");
                AddReward(10f);
				// Move to the next gate TODO: test if this logic works
				// TODO: make sure they are in order
				iNextGate = (iNextGate + 1) % (gatePositions.Count+1); // TODO: remake so gate numbers starts from 0 instead of 1. then gatePos.count+1 not needed, only gatePos.count
				if (iNextGate == 0) { iNextGate = 1; } // Restart. TODO: modulus. GÖR SÅ DOM STARTAR PÅ 0! SLIPPER DENNA OCH PLUS 1 PÅ COUNT!
				
				next2Gates[0] = next2Gates[1];
				next2Gates[1] = gatePositions[iNextGate];
            }else{
				// TODO: fix so that it doesnt give this multiple times when passing through
				// Wrong order!
				Debug.Log("FEL ORDNING");
				AddReward(-1f); // TODO: den vart bättre med denna men eftersom den är skum så borde det inte bli så??
            }
        }
        if (other.gameObject.tag == "Wall")
        {
            AddReward(-5f);
            EndEpisode();
        }
    }

    void FixedUpdate()
    {
        ContinousRewards(lastActions);
        // Reset input forces every fixed update
        inputForce = Vector3.zero;
        inputTorque = Vector3.zero;
    }
}
}

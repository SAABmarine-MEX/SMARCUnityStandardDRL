   using System.Collections;
   using UnityEngine;
   using Unity.Robotics.ROSTCPConnector;
   using RosMessageTypes.Std;  // For standard message types (e.g., String)
   
   public class StringPublisher : MonoBehaviour
   {
       private ROSConnection ros; // ROS connection instance
       public string topicName = "/simple_string_topic";  // ROS topic to publish to
       private string message = "Hello from Unity!";  // The string message to send
       private float publishRate = 1.0f; // Interval in seconds between each publish
   
       void Start()
       {
           // Initialize ROS connection
           ros = ROSConnection.GetOrCreateInstance();
   
           // Start the publishing coroutine
           StartCoroutine(PublishStringMessage());
       }
   
       /// <summary>
       /// Coroutine to publish a string message at regular intervals.
       /// </summary>
       private IEnumerator PublishStringMessage()
       {
           while (true)
           {
               // Create a new String message
               StringMsg stringMsg = new StringMsg(message);
   
               // Publish the string message to the ROS topic
               ros.Publish(topicName, stringMsg);
   
               // Wait for the next publish cycle
               yield return new WaitForSeconds(publishRate);
           }
       }
   }

/*
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Standard messages

// NOTE: READY FOR TESTING. ASK GPT ON HOW TO TEST IT USING ROS IN TERMINAL. SEND SOMETHING TO THE SUBS AND VIEW THE PUB
public class RosDRLNode : MonoBehaviour
{
    // Topic names
    public string topicInput = "/controller_input";
    public string topicOutput = "/controller_output";
    public string topicControlMode = "/control_mode";
    
    // Global variables
    private Float64MultiArray controlOutput;
    bool controlOn = false;
    
    // Reference to the ROS connection
    private ROSConnection ros;
    void Start()
    {
        // Get the singleton instance of ROSConnection
        ros = ROSConnection.instance;
        // Register pubs
        ros.RegisterPublisher<Float64MultiArray>(topicInput); // TODO: kan man sätta namn på denna och är detta en valid datatyp?
        ros.RegisterPublisher<bool>(topicControlMode);
        // Register sub with the message type and a callback method
        ROSConnection.instance.Subscribe<Float64MultiArray>(topicOutput, RecieveOutput);
    }
    void RecieveOutput(Float64MultiArray msg)
    {
        Debug.Log("Recieved output: " + msg.data);
        controlOutput = msg.data;
    }
    void Update()
    {
        // Send control mode and control input
        controlOn = false; // TODO: later add this to come from the tick in unity. make sure to add it to the scene from saabmarine mpc
        Float64MultiArray controlInput = 0; // TODO: make right format
        ros.Send(topicControlMode, controlOn);
        ros.Send(topicInput, controlInput);
        
        // Send controlOutput to the force script. TODO: how do I do this?
    }
}
*/
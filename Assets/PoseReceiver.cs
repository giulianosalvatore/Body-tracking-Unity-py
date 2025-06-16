using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;

// Define a class to match the JSON structure from Python
[System.Serializable]
public class Landmark
{
    public int id;
    public string name;
    public float x;
    public float y;
    public float z;
    public float visibility;
}

[System.Serializable]
public class LandmarkList
{
    public List<Landmark> landmarks;
}


public class PoseReceiver : MonoBehaviour
{
    public Animator characterAnimator; // Assign your character's Animator component here in the Inspector
    public float positionScale = 6.5f; // Adjust to scale landmark positions to your scene
    public float depthScale = 3.0f;   // Adjust for Z-axis scaling
    public Vector3 positionOffset = new Vector3(0, 0, 0); // Adjust to position character correctly

    // --- For direct bone manipulation (more complex, advanced) ---
    // public Transform leftShoulder, leftElbow, leftWrist;
    // public Transform rightShoulder, rightElbow, rightWrist;
    // public Transform hip; // A reference point

    // --- For IK (Inverse Kinematics - simpler for limbs) ---
    private Vector3 leftHandTargetPos, rightHandTargetPos;
    private Vector3 leftElbowHintPos, rightElbowHintPos; // For elbow aiming
    private Quaternion leftHandTargetRot, rightHandTargetRot;

    private Thread receiveThread;
    private UdpClient client;
    public int port = 5052; // Must match Python script's port
    private volatile bool dataReceived = false; // Flag to indicate new data
    private string latestJsonData = "";
    private Landmark[] currentLandmarks;

    // Mapping from MediaPipe landmark names to HumanBodyBones (for reference, not directly used in simple IK)
    private Dictionary<string, HumanBodyBones> landmarkToBoneMap;

    void Start()
    {
        if (characterAnimator == null)
        {
            Debug.LogError("Character Animator not assigned!");
            enabled = false;
            return;
        }

        InitializeLandmarkMap();

        // --- Initialize IK targets to current hand positions to avoid snapping ---
        if (characterAnimator.GetBoneTransform(HumanBodyBones.LeftHand) != null)
        {
            leftHandTargetPos = characterAnimator.GetBoneTransform(HumanBodyBones.LeftHand).position;
            leftHandTargetRot = characterAnimator.GetBoneTransform(HumanBodyBones.LeftHand).rotation;
        }
        if (characterAnimator.GetBoneTransform(HumanBodyBones.RightHand) != null)
        {
            rightHandTargetPos = characterAnimator.GetBoneTransform(HumanBodyBones.RightHand).position;
            rightHandTargetRot = characterAnimator.GetBoneTransform(HumanBodyBones.RightHand).rotation;
        }
        // Initialize elbow hints slightly offset from elbows
        if (characterAnimator.GetBoneTransform(HumanBodyBones.LeftLowerArm) != null)
        {
            leftElbowHintPos = characterAnimator.GetBoneTransform(HumanBodyBones.LeftLowerArm).position - characterAnimator.transform.forward * 0.2f;
        }
         if (characterAnimator.GetBoneTransform(HumanBodyBones.RightLowerArm) != null)
        {
            rightElbowHintPos = characterAnimator.GetBoneTransform(HumanBodyBones.RightLowerArm).position - characterAnimator.transform.forward * 0.2f;
        }


        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log("UDP Receiver started on port " + port);
    }

    private void ReceiveData()
    {
        client = new UdpClient(port);
        while (true)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = client.Receive(ref anyIP);

                //Debug.Log($"Received {data.Length} bytes from {anyIP.Address}");
                
                string text = Encoding.UTF8.GetString(data);
                
                // For Unity's JsonUtility to parse an array, it needs to be wrapped in an object
                string jsonToParse = "{\"landmarks\":" + text + "}";

                lock (latestJsonData) // Lock for thread safety
                {
                    latestJsonData = jsonToParse;
                    dataReceived = true;
                }
            }
            catch (Exception err)
            {
                Debug.LogError(err.ToString());
            }
        }
    }

    void Update()
    {
        if (dataReceived)
        {
            string jsonDataToProcess;
            lock (latestJsonData)
            {
                jsonDataToProcess = latestJsonData;
                dataReceived = false; // Reset flag
            }
            try // Add a try-catch block for parsing
            {

            LandmarkList parsedData = JsonUtility.FromJson<LandmarkList>(jsonDataToProcess);
            if (parsedData != null && parsedData.landmarks != null)
            {
                Debug.Log($"Successfully parsed {parsedData.landmarks.Count} landmarks. First landmark name: {parsedData.landmarks[0].name}");
                currentLandmarks = parsedData.landmarks.ToArray();
                // Process landmarks to update IK targets or bone rotations
                UpdateIKTargets();
            }
            else
            {
                // ADD THIS WARNING:
                Debug.LogWarning("Parsed data is null or landmark list is empty.");
                if(parsedData == null) Debug.LogWarning("parsedData object itself is null.");

                else Debug.LogWarning($"parsedData.landmarks list is null or empty. Count: {(parsedData.landmarks?.Count ?? -1)}");
                Debug.LogWarning("Received JSON string was: " + jsonDataToProcess); // Log the problematic JSON
            }
            }
            catch (Exception e)
            {
            // ADD THIS ERROR LOG:
            Debug.LogError($"JSON Parsing Exception: {e.Message}\nStackTrace: {e.StackTrace}");
            Debug.LogError("Problematic JSON string: " + jsonDataToProcess); // Log the JSON that caused the error
            }
        }
    }

    void InitializeLandmarkMap()
    {
        landmarkToBoneMap = new Dictionary<string, HumanBodyBones>
        {
            {"LEFT_SHOULDER", HumanBodyBones.LeftShoulder},
            {"RIGHT_SHOULDER", HumanBodyBones.RightShoulder},
            {"LEFT_ELBOW", HumanBodyBones.LeftLowerArm}, // Using LowerArm as elbow joint typically
            {"RIGHT_ELBOW", HumanBodyBones.RightLowerArm},
            {"LEFT_WRIST", HumanBodyBones.LeftHand},
            {"RIGHT_WRIST", HumanBodyBones.RightHand},
            {"LEFT_HIP", HumanBodyBones.LeftUpperLeg}, // Hip influences upper leg
            {"RIGHT_HIP", HumanBodyBones.RightUpperLeg},
            // Add more as needed: KNEE, ANKLE, etc.
        };
    }

    Landmark GetLandmark(string name)
    {
        if (currentLandmarks == null) return null;
        foreach (var lm in currentLandmarks)
        {
            if (lm.name == name) return lm;
        }
        return null;
    }

    Vector3 GetLandmarkPosition(string landmarkName, Transform referenceRoot)
    {
        Landmark lm = GetLandmark(landmarkName);
        if (lm == null || lm.visibility < 0.5f) // Check visibility
        {
            // Return a default/previous position or indicate error
            // For simplicity, returning Vector3.zero, but might need better handling
            return Vector3.zero; // This can cause issues if not handled properly
        }

        // MediaPipe X, Y are normalized screen coords (0-1). Y is often inverted (0 at top).
        // Z is pseudo-depth, relative to hips.
        // Convert to Unity world space. This requires careful calibration.
        // This is a very basic mapping and will need tweaking!
        float x = (lm.x - 0.5f) * positionScale; // Center X and scale
        float y = (0.5f - lm.y) * positionScale; // Invert Y, center, and scale
        float z = lm.z * depthScale;       // Scale depth

        // The landmark positions are relative to the camera.
        // We need to transform them into the character's local space or world space based on a reference.
        // Simplistic approach: Apply offset and treat as world positions relative to character's root.
        // A better way is to define a "bounding box" in camera view and map to a 3D volume for the character.

        // `positionOffset` can be used to place the character relative to (0,0,0) where landmarks are projected
        // `referenceRoot.position` can be used if you want landmarks relative to the character's current root
        return new Vector3(x, y, z) + positionOffset;
    }


    void UpdateIKTargets()
    {
        if (currentLandmarks == null || characterAnimator == null) return;

        Landmark nose = GetLandmark("NOSE");
        Landmark leftShoulder = GetLandmark("LEFT_SHOULDER");
        Landmark rightShoulder = GetLandmark("RIGHT_SHOULDER");
        Landmark leftElbow = GetLandmark("LEFT_ELBOW");
        Landmark rightElbow = GetLandmark("RIGHT_ELBOW");
        Landmark leftWrist = GetLandmark("LEFT_WRIST");
        Landmark rightWrist = GetLandmark("RIGHT_WRIST");
        Landmark leftHip = GetLandmark("LEFT_HIP");
        Landmark rightHip = GetLandmark("RIGHT_HIP");

        if (leftWrist != null && leftWrist.visibility > 0.5f)
            leftHandTargetPos = Vector3.Lerp(leftHandTargetPos, GetLandmarkPosition("LEFT_WRIST", characterAnimator.transform), 0.5f);
        if (rightWrist != null && rightWrist.visibility > 0.5f)
            rightHandTargetPos = Vector3.Lerp(rightHandTargetPos, GetLandmarkPosition("RIGHT_WRIST", characterAnimator.transform), 0.5f);

        // Basic elbow hint positioning (can be improved)
        // Position hint behind the elbow relative to shoulder-wrist line.
        if (leftElbow != null && leftElbow.visibility > 0.5f && leftShoulder != null && leftWrist != null) {
             Vector3 shoulderPos = GetLandmarkPosition("LEFT_SHOULDER", characterAnimator.transform);
             Vector3 wristPos = GetLandmarkPosition("LEFT_WRIST", characterAnimator.transform);
             Vector3 elbowPos = GetLandmarkPosition("LEFT_ELBOW", characterAnimator.transform);
             Vector3 shoulderToWristDir = (wristPos - shoulderPos).normalized;
             // Project elbow onto shoulder-wrist line to find a point on that line
             // Then move hint away from that line in a direction perpendicular to character's forward and shoulderToWristDir
             Vector3 elbowPlaneNormal = Vector3.Cross(shoulderToWristDir, characterAnimator.transform.forward);
             leftElbowHintPos = Vector3.Lerp(leftElbowHintPos, elbowPos - elbowPlaneNormal * 0.3f, 0.5f); // Adjust 0.3f factor
        }

        if (rightElbow != null && rightElbow.visibility > 0.5f && rightShoulder != null && rightWrist != null) {
             Vector3 shoulderPos = GetLandmarkPosition("RIGHT_SHOULDER", characterAnimator.transform);
             Vector3 wristPos = GetLandmarkPosition("RIGHT_WRIST", characterAnimator.transform);
             Vector3 elbowPos = GetLandmarkPosition("RIGHT_ELBOW", characterAnimator.transform);
             Vector3 shoulderToWristDir = (wristPos - shoulderPos).normalized;
             Vector3 elbowPlaneNormal = Vector3.Cross(shoulderToWristDir, characterAnimator.transform.forward);
             rightElbowHintPos = Vector3.Lerp(rightElbowHintPos, elbowPos - elbowPlaneNormal * 0.3f, 0.5f);
        }

        // Simple head rotation (LookAt) based on nose (optional)
        // This is very basic and might need refinement for natural head movement
        if (nose != null && nose.visibility > 0.5f && characterAnimator.GetBoneTransform(HumanBodyBones.Head) != null)
        {
            Transform head = characterAnimator.GetBoneTransform(HumanBodyBones.Head);
            Vector3 headTarget = GetLandmarkPosition("NOSE", characterAnimator.transform) + characterAnimator.transform.forward * 1.0f; // Look slightly ahead
            // head.LookAt(headTarget); // This might be too aggressive, use Slerp for smoother rotation
        }

        // Simple body orientation (less reliable with only MediaPipe Pose)
        // You might want to orient the root of the character based on shoulder/hip alignment
        if(leftShoulder != null && rightShoulder != null && leftShoulder.visibility > 0.5f && rightShoulder.visibility > 0.5f)
        {
            // Vector3 shoulderCenter = (GetLandmarkPosition("LEFT_SHOULDER", characterAnimator.transform) +
            //                         GetLandmarkPosition("RIGHT_SHOULDER", characterAnimator.transform)) * 0.5f;
            // characterAnimator.transform.position = Vector3.Lerp(characterAnimator.transform.position, new Vector3(shoulderCenter.x, characterAnimator.transform.position.y, shoulderCenter.z), 0.1f);
            // This moves the whole character. `positionOffset` in GetLandmarkPosition is key.
        }
    }

    // This function is called by Unity when IK is processed
    void OnAnimatorIK(int layerIndex)
    {
        if (characterAnimator)
        {
            if (currentLandmarks == null) return; // No data yet

            // Set IK targets for hands
            characterAnimator.SetIKPositionWeight(AvatarIKGoal.LeftHand, 0.8f); // Weight 0 to 1
            characterAnimator.SetIKRotationWeight(AvatarIKGoal.LeftHand, 0.0f); // Set to 0 if not calculating rotation
            characterAnimator.SetIKPosition(AvatarIKGoal.LeftHand, leftHandTargetPos);
            // characterAnimator.SetIKRotation(AvatarIKGoal.LeftHand, leftHandTargetRot); // Optional: if you calculate hand rotation

            characterAnimator.SetIKPositionWeight(AvatarIKGoal.RightHand, 0.8f);
            characterAnimator.SetIKRotationWeight(AvatarIKGoal.RightHand, 0.0f);
            characterAnimator.SetIKPosition(AvatarIKGoal.RightHand, rightHandTargetPos);
            // characterAnimator.SetIKRotation(AvatarIKGoal.RightHand, rightHandTargetRot);

            // Set IK hints for elbows (helps control bending direction)
            characterAnimator.SetIKHintPositionWeight(AvatarIKHint.LeftElbow, 0.5f);
            characterAnimator.SetIKHintPosition(AvatarIKHint.LeftElbow, leftElbowHintPos);

            characterAnimator.SetIKHintPositionWeight(AvatarIKHint.RightElbow, 0.5f);
            characterAnimator.SetIKHintPosition(AvatarIKHint.RightElbow, rightElbowHintPos);
        }
    }

    void OnDestroy()
    {
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }
        if (client != null)
        {
            client.Close();
        }
        Debug.Log("UDP Receiver stopped and closed.");
    }
}

// Copyright 2025 Azumi Lab.
//
// DreamerV3 Environment Reset via Topic
// Listens to /reset_command topic for reset commands

using System.Collections.Generic;
using UnityEngine;
using ROS2;
using Awsim.Entity;
using Awsim.Common;

namespace Awsim.DreamerV3
{
    /// <summary>
    /// Resets the environment when receiving a message on /reset_command topic.
    /// Attach this script to a manager object in the scene.
    /// DefaultExecutionOrder(-100) ensures Awake() runs before AccelVehicle etc.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class EnvironmentReset : MonoBehaviour
    {
        [Header("ROS 2 Settings")]
        [SerializeField] string _resetCommandTopic = "/reset_command";
        [SerializeField] string _resetResponseTopic = "/reset_response";
        
        [Header("Wheelchair Settings")]
        [SerializeField] Transform _wheelchairTransform;
        [SerializeField] Rigidbody _wheelchairRigidbody;
        [SerializeField] AccelVehicle _accelVehicle;
        
        [Header("Wheelchair Randomization (Optional)")]
        [SerializeField] bool _randomizeWheelchair = false;
        [SerializeField] Vector3 _wheelchairRandomRangeMin = new Vector3(-0.5f, 0f, -0.5f);
        [SerializeField] Vector3 _wheelchairRandomRangeMax = new Vector3(0.5f, 0f, 0.5f);
        [SerializeField] bool _randomizeWheelchairRotation = false;
        [SerializeField] float _wheelchairRotationMin = -15f;
        [SerializeField] float _wheelchairRotationMax = 15f;
        
        [Header("Obstacle Settings (Optional)")]
        [SerializeField] Transform[] _obstacles;
        [SerializeField] Vector3[] _obstacleInitialPositions;
        [SerializeField] Quaternion[] _obstacleInitialRotations;
        [SerializeField] bool _randomizeObstacles = false;
        [SerializeField] Vector3 _obstacleRandomRangeMin = new Vector3(-0.4f, 0f, 1f);
        [SerializeField] Vector3 _obstacleRandomRangeMax = new Vector3(0.4f, 0f, 2f);
        [SerializeField] bool _randomizeObstacleRotation = false;
        [SerializeField] float _obstacleRotationMin = 0f;
        [SerializeField] float _obstacleRotationMax = 360f;
        
        [Header("Goal Settings (Optional)")]
        [SerializeField] Transform _goalObject;
        [SerializeField] string _goalPositionTopic = "/awsim/goal_position";
        [SerializeField, Tooltip("Seconds between re-publications of the goal position (0 = only at start and on reset). "
            + "The topic is VOLATILE, so a client started after AWSIM only receives later messages.")]
        float _goalPublishInterval = 1.0f;
        [SerializeField, Tooltip("geometry_msgs/PoseArray (map frame): poses of the obstacles listed above, published with the goal "
            + "(after every reset and every goal interval). For plots and checks; not part of the agent's observation.")]
        string _obstaclePosesTopic = "/awsim/episode/obstacles";
        
        [Header("Episode settings over ROS 2 (map frame, same as /ground_truth/odom)")]
        [SerializeField, Tooltip("geometry_msgs/Pose of base_link for the next resets. The height is taken from the ground below (x, y); "
            + "position.z is only a hint for the ground search (NaN = search from high above).")]
        string _startPoseTopic = "/awsim/episode/start_pose";
        [SerializeField, Tooltip("geometry_msgs/Point: moves the goal object to (x, y) on the ground (z: hint as for the start pose).")]
        string _goalTopic = "/awsim/episode/goal";
        
        [Header("Debug")]
        [SerializeField] bool _enableDebugLogs = true;
        
        private ISubscription<std_msgs.msg.Bool> _resetCommandSub;
        private IPublisher<std_msgs.msg.Bool> _resetResponsePub;
        private IPublisher<geometry_msgs.msg.Point> _goalPositionPub;
        private IPublisher<geometry_msgs.msg.PoseArray> _obstaclePosesPub;
        private ISubscription<geometry_msgs.msg.Pose> _startPoseSub;
        private ISubscription<geometry_msgs.msg.Point> _goalSub;
        // Height of the vehicle root / goal above the ground below them, captured at start
        private float _vehicleClearance = float.NaN;
        private float _goalClearance = float.NaN;
        private readonly object _episodeLock = new object();
        private geometry_msgs.msg.Pose _pendingStartPose;
        private geometry_msgs.msg.Point _pendingGoal;
        // base_link (GroundTruthPublisher) relative to the vehicle root, captured at start
        private Vector3 _baseLinkOffset = Vector3.zero;
        private bool _isInitialized = false;
        private bool _initialPoseCaptured = false;
        private bool _goalPositionPublished = false;
        private float _nextGoalPublishTime = 0f;
        
        // Captured from scene placement in Awake() - NOT serialized
        private Vector3 _initialPosition;
        private Quaternion _initialRotation;
        
        // Flag for pending reset request
        private volatile bool _resetRequested = false;
        
        void Awake()
        {
            // Auto-find components if not assigned (Awake runs before any Start/physics)
            if (_wheelchairTransform == null)
            {
                var accelVehicle = FindFirstObjectByType<AccelVehicle>();
                if (accelVehicle != null)
                {
                    _wheelchairTransform = accelVehicle.transform;
                    _wheelchairRigidbody = accelVehicle.GetComponent<Rigidbody>();
                    _accelVehicle = accelVehicle;
                }
                else
                {
                    Debug.LogWarning("[EnvironmentReset] AccelVehicle not found in scene!");
                }
            }
            
            // Always capture the scene placement position as the initial pose
            // This runs in Awake() - the earliest Unity callback, before any physics
            if (_wheelchairTransform != null)
            {
                _initialPosition = _wheelchairTransform.position;
                _initialRotation = _wheelchairTransform.rotation;
                
                if (_enableDebugLogs)
                {
                    Debug.Log($"[EnvironmentReset] Captured scene position in Awake(): pos={_initialPosition}, rot={_initialRotation.eulerAngles}");
                }
            }
            // Capture obstacle initial positions and rotations
            if (_obstacles != null && _obstacles.Length > 0)
            {
                _obstacleInitialPositions = new Vector3[_obstacles.Length];
                _obstacleInitialRotations = new Quaternion[_obstacles.Length];
                for (int i = 0; i < _obstacles.Length; i++)
                {
                    if (_obstacles[i] != null)
                    {
                        _obstacleInitialPositions[i] = _obstacles[i].position;
                        _obstacleInitialRotations[i] = _obstacles[i].rotation;
                    }
                }
            }
        }
        
        void Start()
        {
            // Backup: If Awake failed to find wheelchair/capture pose, try again in Start
            if (!_initialPoseCaptured)
            {
                if (_wheelchairTransform == null)
                {
                    var accelVehicle = FindFirstObjectByType<AccelVehicle>();
                    if (accelVehicle != null)
                    {
                        _wheelchairTransform = accelVehicle.transform;
                        _wheelchairRigidbody = accelVehicle.GetComponent<Rigidbody>();
                        _accelVehicle = accelVehicle;
                    }
                }
                
                if (_wheelchairTransform != null)
                {
                    _initialPosition = _wheelchairTransform.position;
                    _initialRotation = _wheelchairTransform.rotation;
                    _initialPoseCaptured = true;
                    
                    if (_enableDebugLogs)
                    {
                        Debug.Log($"[EnvironmentReset] Captured scene position in Start(): pos={_initialPosition}, rot={_initialRotation.eulerAngles}");
                    }
                }
                else
                {
                    Debug.LogError("[EnvironmentReset] Wheelchair transform is missing! Reset function will not work.");
                }
            }
            
            if (_wheelchairTransform != null)
            {
                var baseLink = _wheelchairTransform.GetComponentInChildren<GroundTruthPublisher>(true);
                if (baseLink != null)
                    _baseLinkOffset = Quaternion.Inverse(_wheelchairTransform.rotation)
                        * (baseLink.transform.position - _wheelchairTransform.position);
            }
            
            // Clearances of the scene placement, reused when the start pose / goal is moved
            if (_wheelchairTransform != null && GroundProbe.TryFindGround(_initialPosition.x, _initialPosition.z,
                    _initialPosition.y + GroundProbe.ProbeAboveHint, IgnoredByProbe(), out float g))
                _vehicleClearance = _initialPosition.y - g;
            if (_goalObject != null && GroundProbe.TryFindGround(_goalObject.position.x, _goalObject.position.z,
                    _goalObject.position.y + GroundProbe.ProbeAboveHint, IgnoredByProbe(), out float gg))
                _goalClearance = _goalObject.position.y - gg;
            if (_enableDebugLogs)
                Debug.Log($"[EnvironmentReset] Clearance above ground: vehicle {_vehicleClearance:F3} m, goal {_goalClearance:F3} m");
            
            // Try to initialize ROS 2 in Start
            TryInitializeRos2();
        }
        
        void Update()
        {
            // Retry ROS 2 initialization if failed
            if (!_isInitialized)
            {
                TryInitializeRos2();
            }
            
            ApplyPendingEpisodeSettings();
            
            // Publish the goal position after ROS 2 is initialized, then periodically and on every reset.
            bool goalDue = !_goalPositionPublished
                || (_goalPublishInterval > 0f && Time.unscaledTime >= _nextGoalPublishTime);
            if (_isInitialized && goalDue && _goalObject != null && _goalPositionPub != null)
            {
                var unityPos = _goalObject.position;
                // Unity → ROS coordinate conversion: ROS(x,y,z) = Unity(z, -x, y)
                var goalMsg = new geometry_msgs.msg.Point();
                goalMsg.X = unityPos.z;
                goalMsg.Y = -unityPos.x;
                goalMsg.Z = unityPos.y;
                _goalPositionPub.Publish(goalMsg);
                PublishObstaclePoses();
                _nextGoalPublishTime = Time.unscaledTime + _goalPublishInterval;
                bool firstTime = !_goalPositionPublished;
                _goalPositionPublished = true;
                
                if (_enableDebugLogs && firstTime)
                {
                    Debug.Log($"[EnvironmentReset] Published goal position: Unity({unityPos}) → ROS({goalMsg.X:F3}, {goalMsg.Y:F3}, {goalMsg.Z:F3})");
                }
            }
            
            // Handle pending reset request on main thread
            if (_resetRequested)
            {
                _resetRequested = false;
                
                try
                {
                    ResetEnvironment();
                    _goalPositionPublished = false;   // re-publish the goal with the new episode
                    
                    // Publish response
                    if (_resetResponsePub != null)
                    {
                        var response = new std_msgs.msg.Bool();
                        response.Data = true;
                        _resetResponsePub.Publish(response);
                    }
                    
                    if (_enableDebugLogs)
                    {
                        Debug.Log("[EnvironmentReset] Reset completed successfully");
                    }
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"[EnvironmentReset] Reset failed: {e.Message}");
                }
            }
        }
        
        void TryInitializeRos2()
        {
            try
            {
                // Use AwsimRos2Node for consistency with other AWSIM components
                _resetCommandSub = AwsimRos2Node.CreateSubscription<std_msgs.msg.Bool>(
                    _resetCommandTopic,
                    OnResetCommand
                );
                
                _resetResponsePub = AwsimRos2Node.CreatePublisher<std_msgs.msg.Bool>(_resetResponseTopic);
                
                // Goal position publisher
                if (_goalObject != null)
                {
                    _goalPositionPub = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.Point>(_goalPositionTopic);
                    _obstaclePosesPub = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseArray>(_obstaclePosesTopic);
                    _goalSub = AwsimRos2Node.CreateSubscription<geometry_msgs.msg.Point>(_goalTopic, msg =>
                    {
                        lock (_episodeLock) _pendingGoal = msg;
                    });
                }
                
                _startPoseSub = AwsimRos2Node.CreateSubscription<geometry_msgs.msg.Pose>(_startPoseTopic, msg =>
                {
                    lock (_episodeLock) _pendingStartPose = msg;
                });
                
                _isInitialized = true;
                
                if (_enableDebugLogs)
                {
                    Debug.Log($"[EnvironmentReset] Initialized - listening on {_resetCommandTopic}");
                    if (_goalObject != null)
                    {
                        Debug.Log($"[EnvironmentReset] Goal object: {_goalObject.name} at Unity({_goalObject.position})");
                    }
                }
            }
            catch (System.Exception e)
            {
                // ROS 2 not ready yet, will retry next frame
                if (_enableDebugLogs && e.Message.Contains("not initialized") == false)
                {
                    Debug.LogError($"[EnvironmentReset] Initialization error: {e.Message}");
                }
            }
        }
        
        /// <summary>
        /// Apply start pose / goal received over ROS 2 (main thread).
        /// The start pose takes effect at the next reset; the goal moves immediately.
        /// </summary>
        void ApplyPendingEpisodeSettings()
        {
            geometry_msgs.msg.Pose startPose;
            geometry_msgs.msg.Point goal;
            lock (_episodeLock)
            {
                startPose = _pendingStartPose;
                goal = _pendingGoal;
                _pendingStartPose = null;
                _pendingGoal = null;
            }
            
            if (startPose != null)
                ApplyStartPose(startPose);
            if (goal != null)
                ApplyGoal(goal);
        }
        
        /// <summary>Start pose of base_link (map frame) for the next resets, put on the ground (z: height hint, NaN = none).</summary>
        void ApplyStartPose(geometry_msgs.msg.Pose startPose)
        {
            if (_wheelchairTransform != null)
            {
                var rotation = Ros2Utility.Ros2ToUnityRotation(startPose.Orientation);
                // Upright: keep only the heading (the map is not flat; the vehicle settles on the ground)
                rotation = Quaternion.Euler(0f, rotation.eulerAngles.y, 0f);
                var baseLink = Ros2Utility.Ros2ToUnityPosition(new geometry_msgs.msg.Point
                    { X = startPose.Position.X, Y = startPose.Position.Y, Z = 0.0 });
                var root = baseLink - rotation * _baseLinkOffset;
                root.y = _initialPosition.y;   // fallback: height of the previous start pose
                float top = GroundProbe.ProbeTop(startPose.Position.Z);
                if (!float.IsNaN(_vehicleClearance)
                    && GroundProbe.TryFindGround(root.x, root.z, top, IgnoredByProbe(), out float ground))
                    root.y = ground + _vehicleClearance + 0.02f;
                else
                    Debug.LogWarning($"[EnvironmentReset] No ground found below the start pose; keeping the height {root.y:F2}");
                _initialPosition = root;
                _initialRotation = rotation;
                _initialPoseCaptured = true;
                if (_enableDebugLogs)
                    Debug.Log($"[EnvironmentReset] Start pose set: root={_initialPosition} rot={_initialRotation.eulerAngles}");
            }
        }
        
        /// <summary>Moves the goal object to (x, y) of the map frame, on the ground (z: height hint, NaN = none).</summary>
        void ApplyGoal(geometry_msgs.msg.Point goal)
        {
            if (_goalObject != null)
            {
                var p = Ros2Utility.Ros2ToUnityPosition(new geometry_msgs.msg.Point { X = goal.X, Y = goal.Y, Z = 0.0 });
                float y = _goalObject.position.y;
                if (!float.IsNaN(_goalClearance)
                    && GroundProbe.TryFindGround(p.x, p.z, GroundProbe.ProbeTop(goal.Z), IgnoredByProbe(), out float ground))
                    y = ground + _goalClearance;
                _goalObject.position = new Vector3(p.x, y, p.z);
                _goalPositionPublished = false;
                if (_enableDebugLogs)
                    Debug.Log($"[EnvironmentReset] Goal moved to Unity {_goalObject.position}");
            }
        }
        
        void OnResetCommand(std_msgs.msg.Bool msg)
        {
            if (msg.Data)
            {
                _resetRequested = true;
                
                if (_enableDebugLogs)
                {
                    Debug.Log("[EnvironmentReset] Reset command received");
                }
            }
        }
        
        /// <summary>
        /// Synchronous episode reset (main thread), used by the "reset_episode" request of the TCP
        /// bridge (OptimizerBridge): optional start pose and goal (map frame; NaN height = no hint),
        /// then the reset. Returns the simulation time (Time.timeAsDouble, the clock of the sensor
        /// stamps) right after the reset, so that the client can wait for sensor data taken after it.
        /// </summary>
        public double ResetEpisode(bool hasStart, double startX, double startY, double startYaw, double startZ,
            bool hasGoal, double goalX, double goalY, double goalZ)
        {
            if (hasStart)
            {
                var pose = new geometry_msgs.msg.Pose();
                pose.Position.X = startX; pose.Position.Y = startY; pose.Position.Z = startZ;
                pose.Orientation.Z = System.Math.Sin(startYaw / 2.0);
                pose.Orientation.W = System.Math.Cos(startYaw / 2.0);
                ApplyStartPose(pose);
            }
            if (hasGoal)
                ApplyGoal(new geometry_msgs.msg.Point { X = goalX, Y = goalY, Z = goalZ });
            ResetEnvironment();
            _goalPositionPublished = false;   // re-publish goal and obstacle poses
            return Time.timeAsDouble;
        }
        
        /// <summary>
        /// Reset the environment to initial state.
        /// </summary>
        public void ResetEnvironment()
        {
            if (_enableDebugLogs) Debug.Log("[EnvironmentReset] ResetEnvironment() called");
            
            // Reset wheelchair
            ResetWheelchair();
            
            // Reset obstacles
            ResetObstacles();
        }
        
        void PublishObstaclePoses()
        {
            if (_obstaclePosesPub == null)
                return;
            var msg = new geometry_msgs.msg.PoseArray();
            msg.Header.Frame_id = "map";
            var poses = new System.Collections.Generic.List<geometry_msgs.msg.Pose>();
            if (_obstacles != null)
            {
                foreach (var o in _obstacles)
                {
                    if (o == null || !o.gameObject.activeInHierarchy)
                        continue;
                    var p = Ros2Utility.UnityToRos2Position(o.position);
                    var q = Ros2Utility.UnityToRosRotation(o.rotation);
                    var pose = new geometry_msgs.msg.Pose();
                    pose.Position.X = p.x; pose.Position.Y = p.y; pose.Position.Z = p.z;
                    pose.Orientation.X = q.x; pose.Orientation.Y = q.y; pose.Orientation.Z = q.z; pose.Orientation.W = q.w;
                    poses.Add(pose);
                }
            }
            msg.Poses = poses.ToArray();
            _obstaclePosesPub.Publish(msg);
        }
        
        /// <summary>Objects the ground search must not land on: the vehicle, the goal, the randomized obstacles.</summary>
        List<Transform> IgnoredByProbe()
        {
            var list = new List<Transform> { _wheelchairTransform, _goalObject };
            if (_obstacles != null) list.AddRange(_obstacles);
            return list;
        }
        
        void ResetWheelchair()
        {
            if (_enableDebugLogs) Debug.Log($"[EnvironmentReset] ResetWheelchair() called. Randomize={_randomizeWheelchair}");
            
            // Late Initialization: Try to find wheelchair if null
            if (_wheelchairTransform == null)
            {
                var accelVehicle = FindFirstObjectByType<AccelVehicle>();
                if (accelVehicle != null)
                {
                    _wheelchairTransform = accelVehicle.transform;
                    _wheelchairRigidbody = accelVehicle.GetComponent<Rigidbody>();
                    _accelVehicle = accelVehicle;
                }
            }

            if (_wheelchairTransform == null) 
            {
                Debug.LogWarning("[EnvironmentReset] Wheelchair transform is null! Creating failed.");
                return;
            }
            
            // NO LATE CAPTURE: Rely only on Awake/Start to prevent drift.
            if (!_initialPoseCaptured)
            {
                Debug.LogError("[EnvironmentReset] Initial pose was NEVER captured in Awake/Start! Resetting to (0,0,0) or current position might occur, but disabling late capture to debug drift.");
                // Fallback to current position ONLY if absolutely necessary, but log ERROR.
                // Ideally we should have captured it in Awake.
                if (_initialPosition == Vector3.zero)
                {
                     _initialPosition = _wheelchairTransform.position;
                     _initialRotation = _wheelchairTransform.rotation;
                     _initialPoseCaptured = true;
                     Debug.LogError($"[EnvironmentReset] FORCE CAPTURE at {_initialPosition} (Safety Fallback)");
                }
            }
            
            // Compute reset position based on captured initial pose
            Vector3 resetPosition = _initialPosition;
            Quaternion resetRotation = _initialRotation;
            
            if (_randomizeWheelchair)
            {
                // Randomize position within specified range (relative to initial position)
                Vector3 randomOffset = new Vector3(
                    Random.Range(_wheelchairRandomRangeMin.x, _wheelchairRandomRangeMax.x),
                    Random.Range(_wheelchairRandomRangeMin.y, _wheelchairRandomRangeMax.y),
                    Random.Range(_wheelchairRandomRangeMin.z, _wheelchairRandomRangeMax.z)
                );
                resetPosition = _initialPosition + randomOffset;
                
                if (_enableDebugLogs)
                {
                    Debug.Log($"[EnvironmentReset] Wheelchair randomized: offset=({randomOffset.x:F2}, {randomOffset.y:F2}, {randomOffset.z:F2}), pos={resetPosition}");
                }
            }
            
            if (_randomizeWheelchairRotation)
            {
                // Randomize Y-axis rotation
                float randomYaw = Random.Range(_wheelchairRotationMin, _wheelchairRotationMax);
                resetRotation = _initialRotation * Quaternion.Euler(0f, randomYaw, 0f);
                
                if (_enableDebugLogs)
                {
                    Debug.Log($"[EnvironmentReset] Wheelchair rotation randomized: yaw={randomYaw:F1}°");
                }
            }
            
            // Apply position and rotation
            // If Rigidbody is attached, use its position/rotation to ensure physics update
            if (_wheelchairRigidbody != null)
            {
                _wheelchairRigidbody.position = resetPosition;
                _wheelchairRigidbody.rotation = resetRotation;
                _wheelchairRigidbody.linearVelocity = Vector3.zero;
                _wheelchairRigidbody.angularVelocity = Vector3.zero;
                _wheelchairRigidbody.WakeUp();
            }
            
            // Also update Transform just in case
            _wheelchairTransform.position = resetPosition;
            _wheelchairTransform.rotation = resetRotation;
            
            // Force physics update
            Physics.SyncTransforms();
            
            // Reset AccelVehicle internal state
            if (_accelVehicle != null)
            {
                _accelVehicle.TargetLinearVelocity = 0f;
                _accelVehicle.TargetYawRate = 0f;
                _accelVehicle.AccelerationInput = 0f;
                _accelVehicle.GearInput = Gear.Drive;
            }
        }
        
        void ResetObstacles()
        {
            if (_obstacles == null || _obstacles.Length == 0) return;
            
            for (int i = 0; i < _obstacles.Length; i++)
            {
                if (_obstacles[i] == null) continue;
                
                if (_randomizeObstacles)
                {
                    // Ensure initial positions are captured
                    if (_obstacleInitialPositions == null || _obstacleInitialPositions.Length != _obstacles.Length)
                    {
                        _obstacleInitialPositions = new Vector3[_obstacles.Length];
                        _obstacleInitialRotations = new Quaternion[_obstacles.Length];
                        for (int j = 0; j < _obstacles.Length; j++)
                        {
                            if (_obstacles[j] != null)
                            {
                                _obstacleInitialPositions[j] = _obstacles[j].position;
                                _obstacleInitialRotations[j] = _obstacles[j].rotation;
                            }
                        }
                    }
                    
                    // Randomize obstacle position within specified range
                    Vector3 randomPos = new Vector3(
                        Random.Range(_obstacleRandomRangeMin.x, _obstacleRandomRangeMax.x),
                        Random.Range(_obstacleRandomRangeMin.y, _obstacleRandomRangeMax.y),
                        Random.Range(_obstacleRandomRangeMin.z, _obstacleRandomRangeMax.z)
                    );
                    
                    // Position is relative to obstacle's initial scene position
                    _obstacles[i].position = _obstacleInitialPositions[i] + randomPos;
                    
                    if (_enableDebugLogs)
                    {
                        Debug.Log($"[EnvironmentReset] Obstacle {i} randomized to pos={_obstacles[i].position}");
                    }
                }
                else if (_obstacleInitialPositions != null && i < _obstacleInitialPositions.Length)
                {
                    // Reset to initial position
                    _obstacles[i].position = _obstacleInitialPositions[i];
                }
                
                // Randomize or reset rotation
                if (_randomizeObstacleRotation)
                {
                    float randomYaw = Random.Range(_obstacleRotationMin, _obstacleRotationMax);
                    _obstacles[i].rotation = Quaternion.Euler(0f, randomYaw, 0f);
                    
                    if (_enableDebugLogs)
                    {
                        Debug.Log($"[EnvironmentReset] Obstacle {i} rotation={randomYaw:F1}°");
                    }
                }
                else if (_obstacleInitialRotations != null && i < _obstacleInitialRotations.Length)
                {
                    _obstacles[i].rotation = _obstacleInitialRotations[i];
                }
                
                // Reset obstacle Rigidbody if exists
                var obstacleRb = _obstacles[i].GetComponent<Rigidbody>();
                if (obstacleRb != null)
                {
                    obstacleRb.linearVelocity = Vector3.zero;
                    obstacleRb.angularVelocity = Vector3.zero;
                }
            }
        }
        
        /// <summary>
        /// Manually trigger reset (for testing in editor).
        /// </summary>
        [ContextMenu("Test Reset")]
        public void TestReset()
        {
            ResetEnvironment();
            Debug.Log("[EnvironmentReset] Manual reset triggered");
        }
        
        void OnDestroy()
        {
            if (_resetCommandSub != null)
            {
                AwsimRos2Node.RemoveSubscription<std_msgs.msg.Bool>(_resetCommandSub);
            }
            if (_resetResponsePub != null)
            {
                AwsimRos2Node.RemovePublisher<std_msgs.msg.Bool>(_resetResponsePub);
            }
            if (_goalPositionPub != null)
            {
                AwsimRos2Node.RemovePublisher<geometry_msgs.msg.Point>(_goalPositionPub);
            }
            if (_obstaclePosesPub != null)
            {
                AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseArray>(_obstaclePosesPub);
            }
            if (_goalSub != null)
            {
                AwsimRos2Node.RemoveSubscription<geometry_msgs.msg.Point>(_goalSub);
            }
            if (_startPoseSub != null)
            {
                AwsimRos2Node.RemoveSubscription<geometry_msgs.msg.Pose>(_startPoseSub);
            }
        }
    }
}

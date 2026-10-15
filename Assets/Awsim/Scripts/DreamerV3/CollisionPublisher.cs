// Copyright 2025 Azumi Lab.
//
// DreamerV3 Collision Detection Publisher
// Publishes collision events to /collision_sensor topic

using UnityEngine;
using ROS2;

namespace Awsim.DreamerV3
{
    /// <summary>
    /// Detects collisions with obstacles and publishes to ROS 2.
    /// Attach this script to the AccelVehicle (wheelchair) GameObject.
    /// </summary>
    public class CollisionPublisher : MonoBehaviour
    {
        [Header("ROS 2 Settings")]
        [SerializeField] string _topicName = "/collision_sensor";
        
        [Header("Collision Settings")]
        [SerializeField] string _obstacleTag = "Obstacle";
        [SerializeField] float _cooldownTime = 0.5f;
        
        [Header("Debug")]
        [SerializeField] bool _enableDebugLogs = true;
        
        private ROS2Node _ros2Node;
        private IPublisher<std_msgs.msg.Bool> _publisher;
        private float _lastCollisionTime = -1f;
        private bool _isInitialized = false;
        
        void Update()
        {
            // Lazy initialization - wait for ROS 2 to be ready
            if (!_isInitialized)
            {
                TryInitializeRos2();
            }
        }
        
        void TryInitializeRos2()
        {
            try
            {
                int safeId = Mathf.Abs(GetInstanceID());
                _ros2Node = new ROS2Node("collision_publisher_" + safeId);
                _publisher = _ros2Node.CreatePublisher<std_msgs.msg.Bool>(_topicName);
                _isInitialized = true;
                
                if (_enableDebugLogs)
                {
                    Debug.Log($"[CollisionPublisher] Initialized. Publishing to {_topicName}");
                }
            }
            catch (NotInitializedException)
            {
                // ROS 2 not ready yet, will retry next frame
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[CollisionPublisher] Unexpected error: {e.Message}");
            }
        }
        
        void OnCollisionEnter(Collision collision)
        {
            if (!_isInitialized || _publisher == null) return;
            
            // Check if collision is with an obstacle
            if (!collision.gameObject.CompareTag(_obstacleTag))
            {
                return;
            }
            
            // Apply cooldown to prevent rapid repeated triggers
            float currentTime = Time.time;
            if (currentTime - _lastCollisionTime < _cooldownTime)
            {
                return;
            }
            _lastCollisionTime = currentTime;
            
            // Publish collision event
            PublishCollision();
            
            if (_enableDebugLogs)
            {
                Debug.Log($"[CollisionPublisher] Collision detected with {collision.gameObject.name}");
            }
        }
        
        void OnTriggerEnter(Collider other)
        {
            if (!_isInitialized || _publisher == null) return;
            
            // Also support trigger colliders
            if (!other.CompareTag(_obstacleTag))
            {
                return;
            }
            
            float currentTime = Time.time;
            if (currentTime - _lastCollisionTime < _cooldownTime)
            {
                return;
            }
            _lastCollisionTime = currentTime;
            
            PublishCollision();
            
            if (_enableDebugLogs)
            {
                Debug.Log($"[CollisionPublisher] Trigger collision with {other.gameObject.name}");
            }
        }
        
        void PublishCollision()
        {
            var msg = new std_msgs.msg.Bool();
            msg.Data = true;
            _publisher.Publish(msg);
        }
        
        /// <summary>
        /// Manually publish a collision event (for testing).
        /// </summary>
        public void ForcePublishCollision()
        {
            if (_isInitialized && _publisher != null)
            {
                PublishCollision();
            }
        }
        
        void OnDestroy()
        {
            // Cleanup handled by ROS2Node disposal
        }
    }
}

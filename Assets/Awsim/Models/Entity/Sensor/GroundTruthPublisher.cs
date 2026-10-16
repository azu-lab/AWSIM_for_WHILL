using UnityEngine;
using ROS2;

public class GroundTruthPublisher : MonoBehaviour
{
    [Header("ROS 2 Settings")]
    public string topicName = "/ground_truth/odom";
    public string frameId = "map";
    public string childFrameId = "base_link";
    public float frequency = 20.0f;

    private ROS2Node ros2Node;
    private IPublisher<nav_msgs.msg.Odometry> odomPublisher;
    private float timeElapsed;
    private Rigidbody rb;
    private Transform sensorTransform; // このコンポーネントのtransform

    void Start()
    {
        // このコンポーネントのtransformを使用
        sensorTransform = transform;
        
        // 親階層からRigidbodyを探す（速度・角速度取得用）
        rb = GetComponentInParent<Rigidbody>();
        if (rb == null)
        {
            // Debug.LogError("Rigidbody not found in parent hierarchy.");
        }
        else
        {
            // Debug.Log($"GroundTruthPublisher: Sensor at {sensorTransform.name}, using Rigidbody from {rb.transform.name}");
        }
        
        // Startではまだノードを作らない
    }

    void Update()
    {
        // 1. まだノードが作られていなければ、作成を試みる
        if (ros2Node == null)
        {
            try
            {
                // 【修正】Mathf.Abs()を追加して、マイナス記号が入らないようにしました
                int safeId = Mathf.Abs(GetInstanceID());
                ros2Node = new ROS2Node("ground_truth_publisher_" + safeId);
                
                odomPublisher = ros2Node.CreatePublisher<nav_msgs.msg.Odometry>(topicName);
                // Debug.Log("GroundTruthPublisher: ROS 2 Node Initialized!");
            }
            catch (NotInitializedException)
            {
                // ROS 2がまだ準備中なので待機
                return;
            }
            catch (System.Exception e)
            {
                // Debug.LogError("Unexpected error initializing ROS 2 node: " + e.Message);
                return;
            }
        }

        // 2. 準備ができていなければ何もしない
        if (odomPublisher == null || rb == null) return;

        // 3. 配信処理
        // Keep the remainder, so that the rate is the set frequency (resetting to 0 gave 23 Hz for 30 Hz)
        timeElapsed += Time.deltaTime;
        float period = 1.0f / frequency;
        if (timeElapsed < period) return;
        timeElapsed -= period;
        if (timeElapsed > period) timeElapsed = 0;   // fell behind (low frame rate): no burst

        nav_msgs.msg.Odometry msg = new nav_msgs.msg.Odometry();
        
        // Header
        msg.Header.Frame_id = frameId;
        
        double time = Time.timeAsDouble; 
        msg.Header.Stamp.Sec = (int)time;
        msg.Header.Stamp.Nanosec = (uint)((time - (int)time) * 1e9);
        
        msg.Child_frame_id = childFrameId;

        // Pose - このセンサー位置を使用
        Vector3 sensorPosition = sensorTransform.position;
        
        // Unity座標系 → ROS座標系に変換
        msg.Pose.Pose.Position.X = sensorPosition.z;
        msg.Pose.Pose.Position.Y = -sensorPosition.x;
        msg.Pose.Pose.Position.Z = sensorPosition.y;

        // 姿勢もこのセンサーの姿勢を使用
        msg.Pose.Pose.Orientation.X = -sensorTransform.rotation.z;
        msg.Pose.Pose.Orientation.Y = sensorTransform.rotation.x;
        msg.Pose.Pose.Orientation.Z = -sensorTransform.rotation.y;
        msg.Pose.Pose.Orientation.W = sensorTransform.rotation.w;

        // Twist - センサー位置での速度を計算
        Vector3 cogVelocity = rb.linearVelocity;
        Vector3 angularVel = rb.angularVelocity;
        
        // センサー位置での速度 = 重心の速度 + (角速度 × 重心からセンサーへのオフセット)
        Vector3 cogToSensor = sensorTransform.position - rb.transform.position;
        Vector3 sensorVelocity = cogVelocity + Vector3.Cross(angularVel, cogToSensor);
        
        // センサーのローカル座標系に変換
        Vector3 localVel = sensorTransform.InverseTransformDirection(sensorVelocity);
        
        // デバッグログ
        // float speedDot = Vector3.Dot(sensorVelocity, sensorTransform.forward);
        // Debug.Log($"[GroundTruth] CoGVel: {cogVelocity.magnitude:F3} | SensorVel: {sensorVelocity.magnitude:F3} | ForwardSpeed: {speedDot:F3} | LocalVel.z: {localVel.z:F3} | AngVel.y: {angularVel.y:F3}");
        
        msg.Twist.Twist.Linear.X = localVel.z;  // 前方速度
        msg.Twist.Twist.Linear.Y = -localVel.x; // 横方向速度
        msg.Twist.Twist.Linear.Z = localVel.y;  // 上下方向速度

        // 角速度もセンサーのローカル座標系に変換
        Vector3 localAngVel = sensorTransform.InverseTransformDirection(angularVel);
        msg.Twist.Twist.Angular.X = -localAngVel.z;
        msg.Twist.Twist.Angular.Y = localAngVel.x;
        msg.Twist.Twist.Angular.Z = -localAngVel.y;

        odomPublisher.Publish(msg);
    }
    
    void OnDestroy()
    {
        // エラー回避のため空にしておく
    }
}
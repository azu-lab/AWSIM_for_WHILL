using UnityEngine;
using ROS2;
using Awsim.Entity;

namespace Awsim.Ros2
{
    public class WheelSpeedPublisher : MonoBehaviour
    {
        [Header("ROS 2 Settings")]
        public string topicName = "/whill/wheel_velocities_sim";
        public string nodeName = "wheel_speed_publisher";
        public float frequency = 60.0f; // 実機に合わせて60Hz推奨

        [Header("Target")]
        [SerializeField] AccelVehicle targetVehicle;

        // 内部変数
        private ROS2Node ros2Node;
        private IPublisher<std_msgs.msg.Float32MultiArray> wheelSpeedPublisher;
        private std_msgs.msg.Float32MultiArray msg;
        private float timeElapsed;

        void Update()
        {
            // 1. ノード作成の試行（まだ作れていない場合）
            if (ros2Node == null)
            {
                // ▼▼▼ 修正: Instanceチェックを削除しました ▼▼▼
                // いきなり作成を試みて、ダメなら catch で待機します
                try
                {
                    // GroundTruthと同じやり方でノードを作成
                    ros2Node = new ROS2Node(nodeName);
                    
                    // パブリッシャー作成
                    wheelSpeedPublisher = ros2Node.CreatePublisher<std_msgs.msg.Float32MultiArray>(topicName);
                    
                    // メッセージ初期化
                    msg = new std_msgs.msg.Float32MultiArray();
                    msg.Layout = new std_msgs.msg.MultiArrayLayout();
                    msg.Data = new float[4];

                    Debug.Log($"WheelSpeedPublisher: ROS 2 Node '{nodeName}' Initialized!");
                }
                catch (System.Exception)
                {
                    // まだROS 2の準備ができていない場合はここで待機
                    // (GroundTruthPublisherと同じ挙動)
                    return;
                }
            }

            // 2. 準備不足ならスキップ
            if (wheelSpeedPublisher == null || targetVehicle == null) return;

            // 3. 配信頻度の制御
            timeElapsed += Time.deltaTime;
            if (timeElapsed < (1.0f / frequency)) return;
            timeElapsed = 0;

            // 4. データ取得と配信
            if (msg.Data.Length != 4) msg.Data = new float[4];

            // [0]: 左の実測 (Actual)
            msg.Data[0] = targetVehicle.LeftRearWheelVelocity;
            // [1]: 右の実測 (Actual)
            msg.Data[1] = targetVehicle.RightRearWheelVelocity;
            // [2]: 左の理論値 (Target)
            msg.Data[2] = targetVehicle.LeftTargetVelocity;
            // [3]: 右の理論値 (Target)
            msg.Data[3] = targetVehicle.RightTargetVelocity;

            wheelSpeedPublisher.Publish(msg);
        }

        void OnDestroy()
        {
            ros2Node = null;
            wheelSpeedPublisher = null;
        }
    }
}
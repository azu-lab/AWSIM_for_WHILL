// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.IO;
using UnityEngine;
using Awsim.Entity;

namespace Awsim.Optimizer
{
    /// <summary>
    /// シンプルなTCPベースのJSON-RPC最適化ブリッジ
    /// 長さプレフィックス付きメッセージフレーミングを使用
    /// </summary>
    public class OptimizerBridge : MonoBehaviour
    {
        [Header("Server Settings")]
        [SerializeField] private int _port = 50051;
        [SerializeField, Tooltip("Address to listen on. 127.0.0.1 accepts local clients only; "
            + "use 0.0.0.0 only on a trusted network (the protocol has no authentication).")]
        private string _bindAddress = "127.0.0.1";
        [SerializeField] private bool _startOnAwake = true;
        
        [Header("Vehicle Reference")]
        [SerializeField] private AccelVehicle _accelVehicle;
        [SerializeField] private Rigidbody _vehicleRigidbody;
        [SerializeField] private Transform _vehicleTransform;
        [SerializeField] private AccelVehicleControlModeBasedInputter _vehicleInputter;
        [SerializeField, Tooltip("GroundTruthPublisherと同じセンサー位置(base_linkなど)を指定")] 
        private Transform _sensorTransform;
        
        [Header("Simulation Settings")]
        [SerializeField] private float _defaultTimeScale = 1.0f;
        [SerializeField] private bool _enableDebugLogs = true;
        
        private TcpListener _listener;
        private CancellationTokenSource _cancellationTokenSource;
        private bool _isRunning = false;
        
        // メインスレッドで実行するアクションのキュー
        private ConcurrentQueue<Action> _mainThreadActions = new ConcurrentQueue<Action>();
        
        // 現在のシナリオ実行状態
        private volatile bool _isScenarioRunning = false;
        private volatile bool _scenarioCompleted = false;
        private List<OdomData> _recordedOdoms = new List<OdomData>();
        private float _scenarioStartTime = 0f;
        private float _scenarioDuration = 0f;
        private List<TimestampedCmdVel> _cmdVelSequence = new List<TimestampedCmdVel>();
        private int _cmdVelIndex = 0;
        
        // 初期位置・姿勢の保存
        private Vector3 _initialPosition;
        private Quaternion _initialRotation;
        
        [System.Serializable]
        public class OdomData
        {
            public double timestamp;
            public float position_x;
            public float position_y;
            public float position_z;
            public float orientation_x;
            public float orientation_y;
            public float orientation_z;
            public float orientation_w;
            public float linear_velocity_x;
            public float linear_velocity_y;
            public float linear_velocity_z;
            public float angular_velocity_x;
            public float angular_velocity_y;
            public float angular_velocity_z;
        }
        
        [System.Serializable]
        public class TimestampedCmdVel
        {
            public double timestamp;
            public float linear_x;
            public float linear_y;
            public float linear_z;
            public float angular_x;
            public float angular_y;
            public float angular_z;
        }
        
        void Awake()
        {
            if (_accelVehicle == null)
                _accelVehicle = GetComponent<AccelVehicle>();
            
            if (_vehicleRigidbody == null && _accelVehicle != null)
                _vehicleRigidbody = _accelVehicle.GetComponent<Rigidbody>();
                
            if (_vehicleTransform == null && _accelVehicle != null)
                _vehicleTransform = _accelVehicle.transform;

            if (_vehicleInputter == null && _accelVehicle != null)
                _vehicleInputter = _accelVehicle.GetComponent<AccelVehicleControlModeBasedInputter>();
            
            // センサーTransformが未設定の場合、自動的に探す試み
            if (_sensorTransform == null && _accelVehicle != null)
            {
                // "base_link" という名前の子オブジェクトを探す
                var baseLink = _accelVehicle.transform.Find("base_link");
                if (baseLink != null) 
                {
                    _sensorTransform = baseLink;
                    if (_enableDebugLogs) Debug.Log($"[OptimizerBridge] Found sensor transform: {baseLink.name}");
                }
                // なければ GroundTruthPublisher を探してそのtransformを使う
                else
                {
                    var gtp = _accelVehicle.GetComponentInChildren<GroundTruthPublisher>();
                    if (gtp != null)
                    {
                        // GroundTruthPublisherは通常自分自身のtransformを使っている
                        _sensorTransform = gtp.transform;
                         if (_enableDebugLogs) Debug.Log($"[OptimizerBridge] Found GroundTruthPublisher transform: {_sensorTransform.name}");
                    }
                }
            }
            
            // 初期位置を保存
            if (_vehicleTransform != null)
            {
                _initialPosition = _vehicleTransform.position;
                _initialRotation = _vehicleTransform.rotation;
            }
            
            if (_startOnAwake)
            {
                StartServer();
            }
        }
        
        void OnDestroy()
        {
            StopServer();
        }
        
        void Update()
        {
            // メインスレッドでアクションを実行
            while (_mainThreadActions.TryDequeue(out Action action))
            {
                try
                {
                    action?.Invoke();
                }
                catch (Exception e)
                {
                    Debug.LogError($"[OptimizerBridge] MainThread action error: {e}");
                }
            }
        }
        
        void FixedUpdate()
        {
            if (_isScenarioRunning)
            {
                ProcessScenario();
            }
        }
        
        private void EnqueueMainThread(Action action)
        {
            _mainThreadActions.Enqueue(action);
        }
        
        public void StartServer()
        {
            if (_isRunning) return;
            
            _cancellationTokenSource = new CancellationTokenSource();
            
            try
            {
                _listener = new TcpListener(IPAddress.Parse(_bindAddress), _port);
                _listener.Start();
                _isRunning = true;
                
                Debug.Log($"[OptimizerBridge] Server started on {_bindAddress}:{_port}");
                
                // 接続待機を開始
                Task.Run(() => AcceptClientsAsync(_cancellationTokenSource.Token));
            }
            catch (Exception e)
            {
                Debug.LogError($"[OptimizerBridge] Failed to start server: {e.Message}");
            }
        }
        
        public void StopServer()
        {
            _isRunning = false;
            _cancellationTokenSource?.Cancel();
            _listener?.Stop();
            Debug.Log("[OptimizerBridge] Server stopped");
        }
        
        private async Task AcceptClientsAsync(CancellationToken token)
        {
            while (!token.IsCancellationRequested && _isRunning)
            {
                try
                {
                    var client = await _listener.AcceptTcpClientAsync();
                    _ = Task.Run(() => HandleClientAsync(client, token));
                }
                catch (SocketException) { }
                catch (ObjectDisposedException) { }
            }
        }
        
        private async Task HandleClientAsync(TcpClient client, CancellationToken token)
        {
            if (_enableDebugLogs)
                Debug.Log("[OptimizerBridge] Client connected");
            
            using (client)
            using (var stream = client.GetStream())
            {
                while (!token.IsCancellationRequested && client.Connected)
                {
                    try
                    {
                        // 長さプレフィックス (4バイト, big-endian) を読み取り
                        byte[] lengthBytes = new byte[4];
                        int bytesRead = await ReadExactAsync(stream, lengthBytes, 0, 4, token);
                        if (bytesRead < 4) break;
                        
                        int messageLength = (lengthBytes[0] << 24) | (lengthBytes[1] << 16) | 
                                           (lengthBytes[2] << 8) | lengthBytes[3];
                        
                        if (messageLength <= 0 || messageLength > 50 * 1024 * 1024) // 50MB limit
                        {
                            Debug.LogWarning($"[OptimizerBridge] Invalid message length: {messageLength}");
                            break;
                        }
                        
                        // メッセージ本体を読み取り
                        byte[] messageBytes = new byte[messageLength];
                        bytesRead = await ReadExactAsync(stream, messageBytes, 0, messageLength, token);
                        if (bytesRead < messageLength) break;
                        
                        string request = Encoding.UTF8.GetString(messageBytes);
                        string response = await ProcessRequestAsync(request);
                        
                        // 長さプレフィックス付きでレスポンスを送信
                        byte[] responseBytes = Encoding.UTF8.GetBytes(response);
                        byte[] responseLengthBytes = new byte[4];
                        responseLengthBytes[0] = (byte)(responseBytes.Length >> 24);
                        responseLengthBytes[1] = (byte)(responseBytes.Length >> 16);
                        responseLengthBytes[2] = (byte)(responseBytes.Length >> 8);
                        responseLengthBytes[3] = (byte)(responseBytes.Length);
                        
                        await stream.WriteAsync(responseLengthBytes, 0, 4, token);
                        await stream.WriteAsync(responseBytes, 0, responseBytes.Length, token);
                        await stream.FlushAsync(token);
                    }
                    catch (Exception e)
                    {
                        if (_enableDebugLogs && !token.IsCancellationRequested)
                            Debug.LogWarning($"[OptimizerBridge] Client error: {e.Message}");
                        break;
                    }
                }
            }
            
            if (_enableDebugLogs)
                Debug.Log("[OptimizerBridge] Client disconnected");
        }
        
        private async Task<int> ReadExactAsync(NetworkStream stream, byte[] buffer, int offset, int count, CancellationToken token)
        {
            int totalRead = 0;
            while (totalRead < count)
            {
                int bytesRead = await stream.ReadAsync(buffer, offset + totalRead, count - totalRead, token);
                if (bytesRead == 0) return totalRead;
                totalRead += bytesRead;
            }
            return totalRead;
        }
        
        private async Task<string> ProcessRequestAsync(string jsonRequest)
        {
            try
            {
                var request = JsonUtility.FromJson<RpcRequest>(jsonRequest);
                
                switch (request.method)
                {
                    case "ping":
                        return CreateSuccessResponse("pong");
                    
                    case "set_parameters":
                        return await HandleSetParametersAsync(request.params_json);

                    case "get_parameters":
                        return await HandleGetParametersAsync();
                    
                    case "reset_simulation":
                        return await HandleResetSimulationAsync(request.params_json);
                    
                    case "reset_episode":
                        return await HandleResetEpisodeAsync(request.params_json);
                    
                    case "send_cmd_vel":
                        return HandleSendCmdVel(request.params_json);
                    
                    case "get_current_odom":
                        return HandleGetCurrentOdom();
                    
                    case "run_scenario":
                        return await HandleRunScenarioAsync(request.params_json);
                    
                    default:
                        return CreateErrorResponse($"Unknown method: {request.method}");
                }
            }
            catch (Exception e)
            {
                return CreateErrorResponse($"Parse error: {e.Message}");
            }
        }
        
        [System.Serializable]
        private class RpcRequest
        {
            public string method;
            public string params_json;
        }
        
        [System.Serializable]
        private class RpcResponse
        {
            public bool success;
            public string message;
            public string data_json;
        }
        
        private string CreateSuccessResponse(string message, string dataJson = "")
        {
            var response = new RpcResponse { success = true, message = message, data_json = dataJson };
            return JsonUtility.ToJson(response);
        }
        
        private string CreateErrorResponse(string message)
        {
            var response = new RpcResponse { success = false, message = message };
            return JsonUtility.ToJson(response);
        }
        
        // ===== リクエストハンドラ =====
        
        private async Task<string> HandleSetParametersAsync(string paramsJson)
        {
            try
            {
                var p = JsonUtility.FromJson<ParameterSet>(paramsJson);
                var tcs = new TaskCompletionSource<bool>();
                
                EnqueueMainThread(() => 
                {
                    ApplyParameters(p);
                    tcs.SetResult(true);
                });
                
                await tcs.Task;
                return CreateSuccessResponse("Parameters applied");
            }
            catch (Exception e)
            {
                return CreateErrorResponse($"SetParameters error: {e.Message}");
            }
        }
        
        private async Task<string> HandleGetParametersAsync()
        {
            try
            {
                var tcs = new TaskCompletionSource<ParameterSet>();
                EnqueueMainThread(() => tcs.SetResult(ReadParameters()));
                var p = await tcs.Task;
                return CreateSuccessResponse("OK", JsonUtility.ToJson(p));
            }
            catch (Exception e)
            {
                return CreateErrorResponse($"GetParameters error: {e.Message}");
            }
        }

        /// <summary>
        /// Read the values currently used by the vehicle (inverse of ApplyParameters).
        /// </summary>
        private ParameterSet ReadParameters()
        {
            var p = new ParameterSet();
            if (_accelVehicle == null)
                return p;

            var type = _accelVehicle.GetType();
            var flags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance;
            p.skidding_cancel_rate = GetFieldValue(type, "_skiddingCancelRate", flags, p.skidding_cancel_rate);
            p.front_wheel_skidding_rate = GetFieldValue(type, "_frontWheelSkiddingRate", flags, p.front_wheel_skidding_rate);
            p.forward_slip_multiplier = GetFieldValue(type, "_forwardSlipMultiplier", flags, p.forward_slip_multiplier);
            p.wheel_kp = GetFieldValue(type, "_wheelKp", flags, p.wheel_kp);
            p.wheel_ki = GetFieldValue(type, "_wheelKi", flags, p.wheel_ki);
            p.wheel_kd = GetFieldValue(type, "_wheelKd", flags, p.wheel_kd);
            p.wheel_integral_clamp = GetFieldValue(type, "_wheelIntegralClamp", flags, p.wheel_integral_clamp);
            p.air_drag = GetFieldValue(type, "_airDrag", flags, p.air_drag);
            p.rolling_resistance = GetFieldValue(type, "_rollingResistance", flags, p.rolling_resistance);
            p.frontal_area = GetFieldValue(type, "_frontalArea", flags, p.frontal_area);
            if (_vehicleRigidbody != null)
            {
                p.com_offset_x = _vehicleRigidbody.centerOfMass.x;
                p.com_offset_y = _vehicleRigidbody.centerOfMass.y;
                p.com_offset_z = _vehicleRigidbody.centerOfMass.z;
            }
            return p;
        }

        private float GetFieldValue(Type type, string fieldName, System.Reflection.BindingFlags flags, float fallback)
        {
            var field = type.GetField(fieldName, flags);
            return field != null ? (float)field.GetValue(_accelVehicle) : fallback;
        }

        private void ApplyParameters(ParameterSet p)
        {
            if (_accelVehicle == null)
            {
                Debug.LogError("[OptimizerBridge] AccelVehicle is null!");
                return;
            }
            
            // リフレクションでprivateフィールドにアクセス
            var type = _accelVehicle.GetType();
            var flags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance;
            
            SetFieldValue(type, "_skiddingCancelRate", p.skidding_cancel_rate, flags);
            SetFieldValue(type, "_frontWheelSkiddingRate", p.front_wheel_skidding_rate, flags);
            SetFieldValue(type, "_forwardSlipMultiplier", p.forward_slip_multiplier, flags);
            SetFieldValue(type, "_wheelKp", p.wheel_kp, flags);
            SetFieldValue(type, "_wheelKi", p.wheel_ki, flags);
            SetFieldValue(type, "_wheelKd", p.wheel_kd, flags);
            SetFieldValue(type, "_wheelIntegralClamp", p.wheel_integral_clamp, flags);
            SetFieldValue(type, "_airDrag", p.air_drag, flags);
            SetFieldValue(type, "_rollingResistance", p.rolling_resistance, flags);
            SetFieldValue(type, "_frontalArea", p.frontal_area, flags);
            
            // Rigidbody設定（重心のみ、質量は AccelVehicle.Initialize で固定）
            if (_vehicleRigidbody != null)
            {
                _vehicleRigidbody.centerOfMass = new Vector3(p.com_offset_x, p.com_offset_y, p.com_offset_z);
            }
            
            if (_enableDebugLogs)
                Debug.Log("[OptimizerBridge] Parameters applied");
        }
        
        private void SetFieldValue(Type type, string fieldName, float value, System.Reflection.BindingFlags flags)
        {
            var field = type.GetField(fieldName, flags);
            if (field != null)
            {
                field.SetValue(_accelVehicle, value);
            }
        }
        
        private async Task<string> HandleResetSimulationAsync(string paramsJson)
        {
            try
            {
                var p = JsonUtility.FromJson<ResetRequest>(paramsJson);
                var tcs = new TaskCompletionSource<bool>();
                
                EnqueueMainThread(() => 
                {
                    ResetSimulation(p);
                    tcs.SetResult(true);
                });
                
                await tcs.Task;
                return CreateSuccessResponse("Simulation reset");
            }
            catch (Exception e)
            {
                return CreateErrorResponse($"ResetSimulation error: {e.Message}");
            }
        }
        
        /// <summary>
        /// Episode reset for reinforcement learning (rl/whill_env/data_manager.py): optional start pose
        /// and goal (map frame), then EnvironmentReset.ResetEpisode on the main thread; answers when the
        /// reset is done (synchronous), with the simulation time of the reset in data_json.
        /// </summary>
        [Serializable]
        private class EpisodeResetRequest
        {
            public bool has_start; public double start_x; public double start_y; public double start_yaw;
            public bool has_start_z; public double start_z;
            public bool has_goal; public double goal_x; public double goal_y;
            public bool has_goal_z; public double goal_z;
        }
        
        [Serializable]
        private class EpisodeResetResult
        {
            public double sim_time;
        }
        
        private async Task<string> HandleResetEpisodeAsync(string paramsJson)
        {
            try
            {
                var p = JsonUtility.FromJson<EpisodeResetRequest>(paramsJson ?? "{}");
                var tcs = new TaskCompletionSource<double>();
                EnqueueMainThread(() =>
                {
                    try
                    {
                        var env = FindFirstObjectByType<Awsim.DreamerV3.EnvironmentReset>();
                        if (env == null)
                            throw new Exception("no EnvironmentReset in the scene");
                        tcs.SetResult(env.ResetEpisode(
                            p.has_start, p.start_x, p.start_y, p.start_yaw, p.has_start_z ? p.start_z : double.NaN,
                            p.has_goal, p.goal_x, p.goal_y, p.has_goal_z ? p.goal_z : double.NaN));
                    }
                    catch (Exception e)
                    {
                        tcs.SetException(e);
                    }
                });
                double simTime = await tcs.Task;
                return CreateSuccessResponse("Episode reset", JsonUtility.ToJson(new EpisodeResetResult { sim_time = simTime }));
            }
            catch (Exception e)
            {
                return CreateErrorResponse($"reset_episode error: {e.Message}");
            }
        }
        
        private void ResetSimulation(ResetRequest p)
        {
            if (_vehicleRigidbody != null)
            {
                _vehicleRigidbody.linearVelocity = Vector3.zero;
                _vehicleRigidbody.angularVelocity = Vector3.zero;
            }
            
            if (_vehicleTransform != null)
            {
                // ユーザー要望により、Unity再生開始時の初期位置にリセットする
                // Pythonからのリクエスト値(p.position_xなど)は無視する
                _vehicleTransform.position = _initialPosition;
                _vehicleTransform.rotation = _initialRotation;
                
                if (_enableDebugLogs)
                    Debug.Log($"[OptimizerBridge] Reset to initial pose: {_initialPosition}");
                    
                // Rigidbodyがある場合は、そちらの座標も更新しないと物理演算で戻されることがある
                if (_vehicleRigidbody != null)
                {
                    _vehicleRigidbody.position = _initialPosition;
                    _vehicleRigidbody.rotation = _initialRotation;
                    _vehicleRigidbody.linearVelocity = Vector3.zero;
                    _vehicleRigidbody.angularVelocity = Vector3.zero;
                    // 物理演算を再開させる
                    _vehicleRigidbody.WakeUp();
                }
            }
            
            // cmd_vel入力をリセット
            if (_accelVehicle != null)
            {
                _accelVehicle.TargetLinearVelocity = 0f;
                _accelVehicle.TargetYawRate = 0f;
                _accelVehicle.GearInput = Gear.Drive; 
            }
            
            _isScenarioRunning = false;
            _scenarioCompleted = false;
            _recordedOdoms.Clear();
            
            if (_enableDebugLogs)
                Debug.Log("[OptimizerBridge] Simulation reset");
        }
        
        private string HandleSendCmdVel(string paramsJson)
        {
            try
            {
                var cmd = JsonUtility.FromJson<CmdVelRequest>(paramsJson);
                
                EnqueueMainThread(() => 
                {
                    if (_accelVehicle != null)
                    {
                        _accelVehicle.TargetLinearVelocity = cmd.linear_x;
                        _accelVehicle.TargetYawRate = cmd.angular_z;
                    }
                });
                
                return CreateSuccessResponse("CmdVel sent");
            }
            catch (Exception e)
            {
                return CreateErrorResponse($"SendCmdVel error: {e.Message}");
            }
        }
        
        private string HandleGetCurrentOdom()
        {
            var odom = GetCurrentOdom();
            string odomJson = JsonUtility.ToJson(odom);
            return CreateSuccessResponse("OK", odomJson);
        }
        
        private OdomData GetCurrentOdom()
        {
            var odom = new OdomData();
            
            // センサーTransformが設定されていればそれを使用、なければ車両Transformを使用
            Transform targetTransform = _sensorTransform != null ? _sensorTransform : _vehicleTransform;
            
            if (targetTransform != null)
            {
                // GroundTruthPublisherと同じ方式で位置を取得
                Vector3 sensorPosition = targetTransform.position;
                
                // Unity座標系 -> ROS座標系変換 (GroundTruthPublisher準拠)
                odom.position_x = sensorPosition.z;   // Unity Z -> ROS X
                odom.position_y = -sensorPosition.x;  // Unity X -> ROS -Y
                odom.position_z = sensorPosition.y;   // Unity Y -> ROS Z
                
                // 姿勢もセンサーの姿勢を使用 (GroundTruthPublisher準拠)
                odom.orientation_x = -targetTransform.rotation.z;
                odom.orientation_y = targetTransform.rotation.x;
                odom.orientation_z = -targetTransform.rotation.y;
                odom.orientation_w = targetTransform.rotation.w;
            }
            
            if (_vehicleRigidbody != null && _sensorTransform != null)
            {
                // GroundTruthPublisherと同じ方式で速度を計算
                Vector3 cogVelocity = _vehicleRigidbody.linearVelocity;
                Vector3 angularVel = _vehicleRigidbody.angularVelocity;
                
                // センサー位置での速度 = 重心の速度 + (角速度 × 重心からセンサーへのオフセット)
                Vector3 cogToSensor = _sensorTransform.position - _vehicleRigidbody.transform.position;
                Vector3 sensorVelocity = cogVelocity + Vector3.Cross(angularVel, cogToSensor);
                
                // センサーのローカル座標系に変換
                Vector3 localVel = _sensorTransform.InverseTransformDirection(sensorVelocity);
                
                odom.linear_velocity_x = localVel.z;   // 前方速度
                odom.linear_velocity_y = -localVel.x;  // 横方向速度
                odom.linear_velocity_z = localVel.y;   // 上下方向速度
                
                // 角速度もセンサーのローカル座標系に変換
                Vector3 localAngVel = _sensorTransform.InverseTransformDirection(angularVel);
                odom.angular_velocity_x = -localAngVel.z;
                odom.angular_velocity_y = localAngVel.x;
                odom.angular_velocity_z = -localAngVel.y;
            }
            else if (_vehicleRigidbody != null)
            {
                // フォールバック: 従来の方式
                var vel = _vehicleRigidbody.linearVelocity;
                odom.linear_velocity_x = vel.z;
                odom.linear_velocity_y = -vel.x;
                odom.linear_velocity_z = vel.y;
                
                var angVel = _vehicleRigidbody.angularVelocity;
                odom.angular_velocity_x = angVel.z;
                odom.angular_velocity_y = -angVel.x;
                odom.angular_velocity_z = angVel.y;
            }
            
            odom.timestamp = Time.timeAsDouble;
            
            return odom;
        }
        
        private async Task<string> HandleRunScenarioAsync(string paramsJson)
        {
            try
            {
                var scenario = JsonUtility.FromJson<ScenarioRequest>(paramsJson);
                float timeScale = scenario.time_scale > 0 ? scenario.time_scale : 1f;
                float duration = scenario.duration_seconds;
                
                // シナリオ準備をメインスレッドで実行
                var prepareTask = new TaskCompletionSource<bool>();
                
                EnqueueMainThread(() => 
                {
                    try
                    {
                        // パラメータ適用
                        if (!string.IsNullOrEmpty(scenario.parameters_json))
                        {
                            var p = JsonUtility.FromJson<ParameterSet>(scenario.parameters_json);
                            ApplyParameters(p);
                        }
                        
                        // リセット
                        var resetReq = new ResetRequest();
                        ResetSimulation(resetReq);
                        
                        // cmd_velシーケンスをパース
                        _cmdVelSequence.Clear();
                        if (!string.IsNullOrEmpty(scenario.cmd_vel_sequence_json))
                        {
                            var wrapper = JsonUtility.FromJson<CmdVelSequenceWrapper>("{\"items\":" + scenario.cmd_vel_sequence_json + "}");
                            if (wrapper.items != null)
                            {
                                _cmdVelSequence.AddRange(wrapper.items);
                            }
                        }
                        
                        // シナリオ実行開始
                        _recordedOdoms.Clear();
                        _cmdVelIndex = 0;
                        _scenarioStartTime = Time.time;
                        _scenarioDuration = duration;
                        _scenarioCompleted = false;
                        
                        Time.timeScale = timeScale;
                        _isScenarioRunning = true;
                        
                        if (_accelVehicle != null)
                        {
                            // ギア設定はROS2経由で行うため不要だが、念のため初期化として残すか、
                            // あるいはROS2のgear_cmdと競合しないように削除するのが無難。
                            // しかしユーザーは「replay_bag_on_awsim.pyと同様」と言っているため、
                            // ROS側からgear_cmdが来るのを待つべき。
                            // ここでの強制Drive設定は削除する。
                        }
                        
                        if (_enableDebugLogs)
                            Debug.Log($"[OptimizerBridge] Scenario started: duration={duration}s, timeScale={timeScale}x");
                        
                        prepareTask.SetResult(true);
                    }
                    catch (Exception e)
                    {
                        prepareTask.SetException(e);
                    }
                });
                
                await prepareTask.Task;
                
                // シナリオ完了を待機 (System時間を使用 - スレッドセーフ)
                float realTimeTimeout = duration / timeScale + 30f;
                var startWaitTime = DateTime.Now;
                
                while (!_scenarioCompleted)
                {
                    var elapsed = (DateTime.Now - startWaitTime).TotalSeconds;
                    if (elapsed >= realTimeTimeout)
                    {
                        break;
                    }
                    await Task.Delay(50);
                }
                
                // シナリオ終了をメインスレッドで実行
                var finishTask = new TaskCompletionSource<ScenarioResult>();
                
                EnqueueMainThread(() => 
                {
                    try
                    {
                        _isScenarioRunning = false;
                        Time.timeScale = _defaultTimeScale;
                        
                        if (_accelVehicle != null)
                        {
                            _accelVehicle.TargetLinearVelocity = 0f;
                            _accelVehicle.TargetYawRate = 0f;
                        }
                        
                        var result = new ScenarioResult
                        {
                            success = true,
                            message = _scenarioCompleted ? "Scenario completed" : "Scenario timeout",
                            actual_duration = Time.time - _scenarioStartTime,
                            odom_trajectory_json = JsonHelper.ToJson(_recordedOdoms.ToArray())
                        };
                        
                        if (_enableDebugLogs)
                            Debug.Log($"[OptimizerBridge] Scenario finished: {_recordedOdoms.Count} odom samples");
                        
                        finishTask.SetResult(result);
                    }
                    catch (Exception e)
                    {
                        finishTask.SetException(e);
                    }
                });
                
                var scenarioResult = await finishTask.Task;
                return CreateSuccessResponse(scenarioResult.message, JsonUtility.ToJson(scenarioResult));
            }
            catch (Exception e)
            {
                return CreateErrorResponse($"RunScenario error: {e.Message}");
            }
        }
        
        private void ProcessScenario()
        {
            float elapsed = Time.time - _scenarioStartTime;
            
            // 終了チェック
            if (elapsed >= _scenarioDuration)
            {
                _scenarioCompleted = true;
                return;
            }
            
            // cmd_vel適用ループは削除 (ROS2経由で制御するため)
            /*
            while (_cmdVelIndex < _cmdVelSequence.Count)
            {
                // ...
            }
            */
            
            // Odomを記録
            var odom = GetCurrentOdom();
            odom.timestamp = elapsed;
            _recordedOdoms.Add(odom);
        }
        
        // ===== データクラス =====
        
        [System.Serializable]
        public class ParameterSet
        {
            public float skidding_cancel_rate = 10f;
            public float front_wheel_skidding_rate = 0.1f;
            public float forward_slip_multiplier = 1f;
            public float wheel_kp = 10f;
            public float wheel_ki = 0.1f;
            public float wheel_kd = 0f;
            public float wheel_integral_clamp = 0.5f;
            public float air_drag = 0.2f;
            public float rolling_resistance = 0.02f;
            public float frontal_area = 0.8f;
            public float com_offset_x = 0f;
            public float com_offset_y = 0f;
            public float com_offset_z = 0f;
        }
        
        [System.Serializable]
        public class ResetRequest
        {
            public float position_x = 0f;
            public float position_y = 0f;
            public float position_z = 0f;
            public float rotation_x = 0f;
            public float rotation_y = 0f;
            public float rotation_z = 0f;
        }
        
        [System.Serializable]
        public class CmdVelRequest
        {
            public float linear_x;
            public float linear_y;
            public float linear_z;
            public float angular_x;
            public float angular_y;
            public float angular_z;
        }
        
        [System.Serializable]
        public class ScenarioRequest
        {
            public string parameters_json;
            public string cmd_vel_sequence_json;
            public float duration_seconds;
            public float time_scale = 1f;
            public float odom_sample_interval = 0.02f;
        }
        
        [System.Serializable]
        public class ScenarioResult
        {
            public bool success;
            public string message;
            public float actual_duration;
            public string odom_trajectory_json;
        }
        
        [System.Serializable]
        public class CmdVelSequenceWrapper
        {
            public TimestampedCmdVel[] items;
        }
    }
    
    /// <summary>
    /// JSONヘルパークラス
    /// </summary>
    public static class JsonHelper
    {
        public static T[] FromJson<T>(string json)
        {
            string wrappedJson = "{\"items\":" + json + "}";
            Wrapper<T> wrapper = JsonUtility.FromJson<Wrapper<T>>(wrappedJson);
            return wrapper.items;
        }
        
        public static string ToJson<T>(T[] array)
        {
            Wrapper<T> wrapper = new Wrapper<T>();
            wrapper.items = array;
            string json = JsonUtility.ToJson(wrapper);
            // {"items":[...]} から [...] を抽出
            int start = json.IndexOf('[');
            int end = json.LastIndexOf(']');
            if (start >= 0 && end > start)
            {
                return json.Substring(start, end - start + 1);
            }
            return "[]";
        }
        
        [System.Serializable]
        private class Wrapper<T>
        {
            public T[] items;
        }
    }
}

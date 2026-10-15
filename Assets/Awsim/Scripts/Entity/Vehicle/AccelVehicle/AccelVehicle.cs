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
using UnityEngine;
using Awsim.Common;

namespace Awsim.Entity
{
    public class AccelVehicle : MonoBehaviour, IReadOnlyAccelVehicle
    {
        /// <summary>
        /// WheelCollider wrapper and update visual of Wheel. Use only WheelCollider collision and suspension.
        /// </summary>
        [Serializable]
        public class Wheel
        {
            /// <summary>
            /// Wheel contact with ground? Updated by UpdateGroundHit().
            /// </summary>
            public bool IsGrounded { get; private set; } = true;

            /// <summary>
            /// Data on the wheel hits. Updated by UpdateGroundHit().
            /// </summary>
            public WheelHit WheelHit
            {
                get { return _wheelHit; }
            }

            /// <summary>
            /// Tire angle (degree). Apply to front Axle only.
            /// </summary>
            /// NOTE: It might be nice to have Axle flag Steerable as well. And reflect angle on all Steerable.
            public float SteerTireAngle
            {
                get
                {
                    return _tierYawAngle;
                }

                set
                {
                    _tierYawAngle = value;

                    if (_tierYawAngle == 0)
                        SteerTireAngle = 0.00001f;

                    if (_wheelCollider.steerAngle != SteerTireAngle)
                        _wheelCollider.steerAngle = _tierYawAngle;
                }
            }

            /// <summary>
            /// Load on wheel (kg).
            /// </summary>
            public float SprungMass => _wheelCollider.sprungMass;

            public WheelCollider WheelCollider => _wheelCollider;

            [SerializeField] WheelCollider _wheelCollider;
            [SerializeField] Transform _wheelVisualTransform;
            WheelHit _wheelHit;
            Rigidbody _vehicleRigidbody;
            float _tierYawAngle = 0f;
            float _wheelPitchAngle = 0f;

            /// <summary>
            /// Initlaize wheel.
            /// </summary>
            public void Initialize()
            {
                _vehicleRigidbody = _wheelCollider.attachedRigidbody;

                // Initialize WheelCollider appropriately.
                // AccelerationBasedVehicle uses only WheelCollider's suspension and collision.
                // Turn off other functions.
                WheelFrictionCurve friction = new WheelFrictionCurve();
                friction.stiffness = 1.0f; // 0.001 -> 1.0 に変更して最低限の物理グリップを確保

                _wheelCollider.forwardFriction = friction;
                _wheelCollider.sidewaysFriction = friction;

                _wheelCollider.ConfigureVehicleSubsteps(1000.0f, 1, 1);
                _wheelCollider.motorTorque = 0.00001f;
                
                // デバッグ: 初期化時の質量情報を出力
                // Debug.Log($"[Wheel.Initialize] {_wheelCollider.name} | Rigidbody.mass={_vehicleRigidbody.mass:F1}kg | WheelCollider.sprungMass={_wheelCollider.sprungMass:F1}kg");
            }

            /// <summary>
            /// Update wheel hit and grounded flag.
            /// </summary>
            public void UpdateWheelHit()
            {
                IsGrounded = _wheelCollider.GetGroundHit(out _wheelHit);
            }

            /// <summary>
            /// Update wheel visual object.
            /// </summary>
            public void UpdateWheelVisual(float forwardSlipMultiplier)
            {
                if (_wheelVisualTransform == null)
                    return;

                var vehicleVelocity = _vehicleRigidbody.linearVelocity;
                var localSpeed = _vehicleRigidbody.transform.InverseTransformDirection(vehicleVelocity);

                // Wheel position.
                _wheelCollider.GetWorldPose(out var pos, out _);
                _wheelVisualTransform.position = pos;

                // Wheel forward rotation (pitch).
                var addtionalPitchAngle = localSpeed.z / forwardSlipMultiplier * Time.deltaTime / _wheelCollider.radius * Mathf.Rad2Deg;
                _wheelPitchAngle += addtionalPitchAngle;
                _wheelPitchAngle %= 360f;

                _wheelVisualTransform.localEulerAngles = new Vector3(_wheelPitchAngle, SteerTireAngle, 0);
            }
        }

        /// <summary>
        /// Axle with wheels on both sides (left and right).
        /// </summary>
        [Serializable]
        public class Axle
        {
            [SerializeField] Wheel _leftWheel;
            [SerializeField] Wheel _rightWheel;

            /// <summary>
            /// Left wheel.
            /// </summary>
            public Wheel LeftWheel => _leftWheel;

            /// <summary>
            /// Right wheel.
            /// </summary>
            public Wheel RightWheel => _rightWheel;
        }

        // ----- Vehicle input properties -----

        /// <summary>
        /// Acceleration input (m/s^2).
        /// </summary>
        public float AccelerationInput { get; set; } = 0f;

        /// <summary>
        /// Steering tire angle input (degree).
        /// </summary>
    public float SteerTireAngleInput
        {
            get
            {
                return _steerTireAngle;
            }

            set
            {
                _steerTireAngle = Mathf.Clamp(value, _maxSteerTireAngle * -1.0f, _maxSteerTireAngle);
            }
        }

        /// <summary>
        /// Gear input (AT).
        /// </summary>
    public Gear GearInput { get; set; } = Gear.Parking;

        /// <summary>
        /// Turn indicators input.
        /// </summary>
    public TurnIndicators TurnIndicatorsInput { get; set; } = TurnIndicators.None;

        /// <summary>
        /// Hazard lights input.
        /// </summary>
    public HazardLights HazardLightsInput { get; set; } = HazardLights.Disable;

        /// <summary>
        /// Change the slip rate of the wheel in the forward direction of this car. 1 no slip, 0 is full slip.
        /// </summary>
        public float ForwardSlipMultiplier
        {
            get
            {
                return _forwardSlipMultiplier;
            }

            set
            {
                _forwardSlipMultiplier = Mathf.Max(0.0001f, value);
            }
        }

        /// <summary>
        /// Change the slip rate of the wheel in the sideway direction of this car. 1 no slip, 0 is full slip.
        /// NOTE: 差動二輪ロボットでは横滑りは_skiddingCancelRateで制御するため、このプロパティは互換性のために残されています。
        /// </summary>
        public float SidewaySlipMultiplier
        {
            get
            {
                return 1.0f; // 差動二輪では使用しないため固定値を返す
            }

            set
            {
                // 差動二輪では横滑りは_skiddingCancelRateで制御するため、何もしない
                // 互換性のためにプロパティのみ残す
            }
        }

        // ----- Vehicle state properties -----

        /// <summary>
        /// Gear (AT).
        /// </summary>
        public Gear Gear => GearInput;

        /// <summary>
        /// Turn indicators.
        /// </summary>
        public TurnIndicators TurnIndicators => TurnIndicatorsInput;

        /// <summary>
        /// Hazard lights.
        /// </summary>
        public HazardLights HazardLights => HazardLightsInput;

        /// <summary>
        /// Steering tire angle (degree).
        /// </summary>
        public float SteerTireAngle => _firstOrderLaggedSteerTireAngle.Value;

        /// <summary>
        /// Local acceleration (m/s^2).
        /// </summary>
        public Vector3 LocalAcceleration { get; private set; } = Vector3.zero;

        /// <summary>
        /// Vehicle angular velocity in the local coordinate system of the vehicle (rad/s).
        /// </summary>
        public Vector3 LocalAngularVelocity => _transform.InverseTransformDirection(AngularVelocity);

        /// <summary>
        /// Speed (m/s).
        /// </summary>
        public float Speed { get; private set; } = 0f;

        /// <summary>
        /// Velocity (m/s).
        /// </summary>
        public Vector3 Velocity => _rigidbody.linearVelocity;

        /// <summary>
        /// Local velcoity (m/s).
        /// </summary>
        public Vector3 LocalVelocity => _transform.InverseTransformDirection(Velocity);

        /// <summary>
        /// Angular velocity (rad/s).
        /// </summary>
        public Vector3 AngularVelocity { get; private set; } = Vector3.zero;

        public Vector3 LocalAngularAcceleration => _transform.InverseTransformDirection(AngularAcceleration);

        /// <summary>
        /// Angular acceleration (ras/s^2). 
        /// </summary>
        public Vector3 AngularAcceleration { get; private set; } = Vector3.zero;

        public Wheel[] Wheels => _wheels;

        public float MaxSteerTireAngle => _maxSteerTireAngle;

        /// <summary>
        /// 最大加速度（差動二輪では各輪PIDで制御されるため、参考値として最高速度から概算）
        /// </summary>
        public float MaxAcceleration => 3.0f; // [m/s²] 参考値（WHILL相当の加速性能）

        public float SteerTireAngleNormalized => SteerTireAngle / _maxSteerTireAngle;

        // 実測車輪速度 [m/s]
        public float LeftRearWheelVelocity { get; private set; } = 0f;
        public float RightRearWheelVelocity { get; private set; } = 0f;
        
        // 目標車輪速度 [m/s] (内部変数 _leftTargetVelocity などを公開)
        public float LeftTargetVelocity => _leftTargetVelocity;
        public float RightTargetVelocity => _rightTargetVelocity;

        [SerializeField] Rigidbody _rigidbody;
        [SerializeField, Tooltip("後輪の横滑り抵抗係数（速度減衰モデル: 大きいほど横滑りしにくい）")] 
        float _skiddingCancelRate = 10f; // 通常タイヤレベル（速度減衰係数）
        [SerializeField, Tooltip("前輪の横滑り抵抗係数（オムニホイール: 後輪の1/10）")] 
        float _frontWheelSkiddingRate = 0.1f; // オムニホイール効果（速度減衰係数）
        [SerializeField] Axle _frontAxle;
        [SerializeField] Axle _rearAxle;
        [SerializeField] float sleepVelocityThreshold = 0.02f;
        [SerializeField] float _accelerationTimeConstant = 0f;
        [SerializeField] float _steerTireAngleTimeConstant = 0f;
        FirstOrderLaggedFloat _firstOrderLaggedAcceleration = null;
        FirstOrderLaggedFloat _firstOrderLaggedSteerTireAngle = null;
        [SerializeField] float _maxSteerTireAngle = 35f;
        float _steerTireAngle = 0f;

        [Tooltip("Deceleration curve when throttle is off (like engine braking). Time axis represents velocity (m/s), Value axis represents deceleration (m/s^2).")]
        [SerializeField] AnimationCurve _decelerationCurve;

        // Slip multiplier
        [SerializeField] float _forwardSlipMultiplier = 1f;

    [Header("Differential Drive - Inverse Kinematics")]
    [SerializeField, Tooltip("トレッド幅 [m] (WHILL Model CRの実測値)")]
    float _treadWidth = 0.55f;
    [SerializeField, Tooltip("各ホイール速度制御用PIDゲイン Kp（加速度出力版）")]
    float _wheelKp = 10f; // 振動抑制のため低めに設定
    [SerializeField, Tooltip("各ホイール速度制御用PIDゲイン Ki")]
    float _wheelKi = 0.1f; // 積分による振動を抑制
    [SerializeField, Tooltip("各ホイール速度制御用PIDゲイン Kd（ノイズ対策のため初期値0）")]
    float _wheelKd = 0.0f; // D項はノイズを拾いやすいため無効化
    [SerializeField, Tooltip("PID積分上限値 [m/s]")]
    float _wheelIntegralClamp = 0.5f; // 積分ワインドアップを厳しく制限
    
    // 最大駆動力: 200N/輪 (登坂10°・最大167kg対応、パラメータ探索対象外)
    private const float _maxForcePerWheel = 200f;
    [SerializeField, Tooltip("Enable debug logs for AccelVehicle (verbose)")]
    bool _enableDebugLogs = true; // デバッグログを有効化

        // 左右後輪の目標速度と制御状態
        private float _leftTargetVelocity = 0f;
        private float _rightTargetVelocity = 0f;
        private float _leftIntegral = 0f;
        private float _rightIntegral = 0f;
        private float _leftPrevError = 0f;
        private float _rightPrevError = 0f;
        
        [Header("Speed Control")]
        [SerializeField] float _maxSpeed = 2.22f; // 最高速度 8km/h = 2.22m/s
        
        [Header("Drag Forces")]
        [SerializeField] float _airDrag = 0.2f; // 空気抵抗係数（大幅削減）
        [SerializeField] float _rollingResistance = 0.02f; // 転がり抵抗係数（大幅削減）
        [SerializeField] float _frontalArea = 0.8f; // 前面投影面積 (m^2) - WHILL相当
        
        private const float airDensity = 1.225f; // 空気密度 (kg/m^3)
        private const float gravity = 9.81f; // 重力加速度 (m/s^2)

    // ----- Velocity control (v/ω targets -> internal wheel velocities) -----
    [Header("Velocity Control (v/ω targets)")]
    [Tooltip("v/ωターゲットから各輪の目標速度を生成（未使用なら0のまま）")]
    [SerializeField] bool _useVelocityControl = false;
    [Tooltip("目標前進速度 [m/s]（+は前進）")]
    public float TargetLinearVelocity { get; set; } = 0f;
    [Tooltip("目標角速度 [rad/s]（+は右回頭）")]
    public float TargetYawRate { get; set; } = 0f;
    [SerializeField, Tooltip("自動有効化のしきい値")]
    float _velCtrlEnableEps = 1e-4f;

    Gear _lastGear = Gear.Parking;

        // Cache components.
        Transform _transform = null;
        Wheel[] _wheels = null;

        // Cache previous frame values.
        Vector3 _lastVelocity = Vector3.zero;
        Quaternion _lastRotation = new Quaternion();
        Vector3 _lastAngularVelocity = Vector3.zero;

        /// <summary>
        /// Initialize vehicle.
        /// </summary>
        public void Initialize()
        {
            _transform = transform;
            _wheels = new Wheel[] { _frontAxle.LeftWheel, _frontAxle.RightWheel, _rearAxle.LeftWheel, _rearAxle.RightWheel };
            foreach (var wheel in _wheels)
                wheel.Initialize();

            _firstOrderLaggedAcceleration = new FirstOrderLaggedFloat(_accelerationTimeConstant, AccelerationInput);
            _firstOrderLaggedSteerTireAngle = new FirstOrderLaggedFloat(_steerTireAngleTimeConstant, SteerTireAngleInput);
            
            // Rigidbodyの設定を最適化（停止時回転のため）
            if (_rigidbody != null)
            {
                _rigidbody.mass = 55f; // 車両質量を55kgに固定
                _rigidbody.maxAngularVelocity = 30f; // さらに引き上げ
                // 回頭立ち上がり改善のため減衰を極小に
                _rigidbody.angularDamping = 0.0f;
            }
            
            // 横摩擦係数の確認ログ（デバッグ用）
            // Debug.Log($"[AccelVehicle.Initialize] 後輪横摩擦係数={_skiddingCancelRate:F1}, 前輪横摩擦係数={_frontWheelSkiddingRate:F1}");
        }

        /// <summary>
        /// Update wheel visuals.
        /// Assumption called by Update() of MonoBehaviour.
        /// </summary>
        public void OnUpdate()
        {
            foreach (var wheel in _wheels)
                wheel.UpdateWheelVisual(ForwardSlipMultiplier);
        }

        /// <summary>
        /// Calculate vehicle states, apply first order lag, determine sleep physics components, and apply tire forces.
        /// Assumption called by Update() of MonoBehaviour.
        /// </summary>
        public void OnFixedUpdate()
        {
            // --- Calculate vehicle states ---
            // Speed.
            Speed = Vector3.Dot(Velocity, _transform.forward);

            // Local acceleration（物理更新は fixedDeltaTime を使用）
            Vector3 acceleration = (Velocity - _lastVelocity) / Time.fixedDeltaTime;
            LocalAcceleration = _transform.InverseTransformDirection(acceleration);
            _lastVelocity = Velocity;

            // Angular velocity and angular acceleration.
            _lastAngularVelocity = AngularVelocity;
            // Rigidbody から角速度 [rad/s]（ワールド座標）を直接取得
            AngularVelocity = _rigidbody.angularVelocity;
            AngularAcceleration = (AngularVelocity - _lastAngularVelocity) / Time.fixedDeltaTime;
            _lastRotation = _transform.rotation;

            // --- 逆運動学による各輪目標速度の計算 ---
            {
                // ギア変化・Parkingでは積分をリセット
                if (_lastGear != GearInput || GearInput == Gear.Parking)
                {
                    _leftIntegral = _rightIntegral = 0f;
                    _leftPrevError = _rightPrevError = 0f;
                    _lastGear = GearInput;
                }

                // 明示ON もしくは 非ゼロ指令で自動ON
                bool targetsNonZero = (Mathf.Abs(TargetLinearVelocity) > _velCtrlEnableEps) || (Mathf.Abs(TargetYawRate) > _velCtrlEnableEps);
                bool enableCtrl = _useVelocityControl || targetsNonZero;

                if (enableCtrl && (GearInput == Gear.Drive || GearInput == Gear.Reverse || GearInput == Gear.Neutral))
                {
                    // 逆運動学: V_L = V - (d/2)*ω, V_R = V + (d/2)*ω
                    float halfTread = _treadWidth * 0.5f;
                    _leftTargetVelocity = TargetLinearVelocity - halfTread * TargetYawRate;
                    _rightTargetVelocity = TargetLinearVelocity + halfTread * TargetYawRate;

                    // if (_enableDebugLogs)
                    // {
                    //     Debug.Log($"[InverseKinematics] V_target={TargetLinearVelocity:F3} ω_target={TargetYawRate:F3} → V_L={_leftTargetVelocity:F3} V_R={_rightTargetVelocity:F3}");
                    // }
                }
                else if (!enableCtrl)
                {
                    // 制御OFF時は目標速度をゼロにリセット
                    _leftTargetVelocity = _rightTargetVelocity = 0f;
                    _leftIntegral = _rightIntegral = 0f;
                    _leftPrevError = _rightPrevError = 0f;
                }
            }

            // --- Apply first order lag ---
            _firstOrderLaggedAcceleration.DesiredValue = AccelerationInput;
            _firstOrderLaggedSteerTireAngle.DesiredValue = 0f; // 差動二輪では舵角は常に0
            
            // ステアリング角度を0に固定（タイヤは常に直進方向）
            _frontAxle.LeftWheel.SteerTireAngle = 0f;
            _frontAxle.RightWheel.SteerTireAngle = 0f;

            // --- Determine sleep physics ---
            foreach (var wheel in Wheels)
            {
                wheel.UpdateWheelHit();
            }

            var canSleep = CanSleep();

            if (canSleep)
            {
                Sleep();
                return;
            }
            else
            {
                WakeUp();
            }

            // --- Apply tire forces ---
            var a = _firstOrderLaggedAcceleration.Value;

            // Debug: log velocity control diagnostics
            // if (_enableDebugLogs)
            // {
            //     float wNowDbg = Vector3.Dot(AngularVelocity, _transform.up);
            //     Debug.Log($"[AccelVehicle] Gear={Gear} TargetLinearVel={TargetLinearVelocity:F3} TargetYawRate={TargetYawRate:F3} vNow={LocalVelocity.z:F3} wNow={wNowDbg:F3} leftTarget={_leftTargetVelocity:F3} rightTarget={_rightTargetVelocity:F3} canSleep={canSleep}");
            // }

            // 空気抵抗と転がり抵抗を適用
            ApplyDragForces();

            if (Gear == Gear.Drive)
            {
                ApplyWheelForce(a);
            }
            else if (Gear == Gear.Reverse)
            {
                ApplyWheelForce(-a);
            }
            else if (Gear == Gear.Neutral)
            {
                if (a >= 0)
                    ApplyWheelForce(0);
                else if (Speed > 0)
                    ApplyWheelForce(a);
                else if (Speed < 0)
                    ApplyWheelForce(-a);
            }

            bool CanSleep()
            {
                var isAllWheelGrounded = true;
                foreach (var e in Wheels)
                {
                    if (e.IsGrounded == false)
                    {
                        isAllWheelGrounded = false;
                        break;
                    }
                }
                var isParkingShift = Gear == Gear.Parking;
                var isNeutralShift = Gear == Gear.Neutral;
                var isDriveShift = Gear == Gear.Drive;
                var isReverseShift = Gear == Gear.Reverse;
                var isUnderSleepVelocity = Mathf.Abs(LocalVelocity.z) < sleepVelocityThreshold;
                var isZeroOrNegativeAcceleration = AccelerationInput <= 0;
                var isMinusSpeed = Speed < 0;
                var isPlusSpeed = Speed > 0;
                // 左右の目標速度がゼロ付近かチェック（信地旋回を許可）
                var hasWheelTargets = (Mathf.Abs(_leftTargetVelocity) > 0.01f) || (Mathf.Abs(_rightTargetVelocity) > 0.01f);

                // Can sleep?
                var canSleepParkingShift = isAllWheelGrounded && isParkingShift && !hasWheelTargets;
                var canSleepDriveShift = isAllWheelGrounded && isDriveShift && isZeroOrNegativeAcceleration && (isMinusSpeed || isUnderSleepVelocity) && !hasWheelTargets;
                var canSleepReverseShift = isAllWheelGrounded && isReverseShift && isZeroOrNegativeAcceleration && (isPlusSpeed || isUnderSleepVelocity) && !hasWheelTargets;
                var canSleepNeutralShift = isAllWheelGrounded && isNeutralShift && isUnderSleepVelocity && !hasWheelTargets;
                var canSleep = canSleepParkingShift || canSleepDriveShift || canSleepNeutralShift || canSleepReverseShift;

                return canSleep;
            }

            /// <summary>
            /// 現実的な空気抵抗と転がり抵抗を適用
            /// </summary>
            void ApplyDragForces()
            {
                // 速度ベクトルを取得
                var velocity = _rigidbody.linearVelocity;
                var speed = velocity.magnitude;
                
                if (speed < 0.01f) return; // 極低速時は抵抗を適用しない
                
                // 転がり抵抗（重量に比例、速度に依存しない)
                var rollingResistanceForce = -velocity.normalized * (_rollingResistance * _rigidbody.mass * gravity);
                _rigidbody.AddForce(rollingResistanceForce, ForceMode.Force);
                
                // 空気抵抗（速度の2乗に比例、現実的な計算）
                var airDragForce = -velocity.normalized * (0.5f * airDensity * _airDrag * _frontalArea * speed * speed);
                _rigidbody.AddForce(airDragForce, ForceMode.Force);
                
                // 最高速度での強制制限（安全装置）- より緩やかな制限
                if (speed > _maxSpeed * 1.1f) // 最高速度の110%を超えた場合のみ制限
                {
                    var excessSpeedForce = -velocity.normalized * ((speed - _maxSpeed * 1.1f) * _rigidbody.mass * 5f); // 制限力も半減
                    _rigidbody.AddForce(excessSpeedForce, ForceMode.Force);
                }
            }

            void Sleep()
            {
                _rigidbody.linearVelocity = Vector3.zero;
                _rigidbody.angularVelocity = Vector3.zero;
                _rigidbody.Sleep();
                _rigidbody.constraints = RigidbodyConstraints.FreezeAll;
                
                // PID積分項をリセット（停止時のドリフト防止）
                _leftIntegral = _rightIntegral = 0f;
                _leftPrevError = _rightPrevError = 0f;
            }

            void WakeUp()
            {
                _rigidbody.constraints = RigidbodyConstraints.None;
                _rigidbody.WakeUp();
            }

            void ApplyWheelForce(float acceleration)
            {
                var eachWheelDeceleration = -1 * Mathf.Sign(Speed) * _decelerationCurve.Evaluate(Mathf.Abs(Speed)) / Wheels.Length;
                if (Mathf.Abs(AccelerationInput) > 0)
                {
                    eachWheelDeceleration = 0;
                }

                float dt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);

                foreach (var wheel in Wheels)
                {
                    if (wheel.IsGrounded == false)
                        continue; // 非接地輪はスキップ

                    var pointVelocity = _rigidbody.GetPointVelocity(wheel.WheelHit.point);
                    var wheelVelocity = pointVelocity - Vector3.Project(pointVelocity, wheel.WheelHit.normal);
                    var localWheelVelocity = Vector3.zero;
                    // WheelColliderのforwardDirは進行方向を向いているので、そのまま内積を取る
                    // 前進時に正の値になる（符号反転は不要）
                    localWheelVelocity.y = Vector3.Dot(wheel.WheelHit.forwardDir, wheelVelocity); // 前進方向速度 [m/s]
                    localWheelVelocity.x = Vector3.Dot(wheel.WheelHit.sidewaysDir, wheelVelocity); // 横方向速度 [m/s]

                    bool isFrontWheel = (wheel == _frontAxle.LeftWheel || wheel == _frontAxle.RightWheel);
                    bool isLeftRearWheel = (wheel == _rearAxle.LeftWheel);
                    bool isRightRearWheel = (wheel == _rearAxle.RightWheel);

                    // --- 横滑りキャンセル力の適用 ---
                    // 後輪: 強いグリップ（差動駆動の基盤）
                    // 前輪: オムニホイールの軽い抵抗（完全ゼロではなく、現実的な摩擦）
                    // 
                    // 粘性減衰モデル: F = -c × v (c: 減衰係数、v: 横速度)
                    // 質量に比例させることで安定したダンピング力を実現
                    float lateralCancelCoeff = isFrontWheel ? _frontWheelSkiddingRate : _skiddingCancelRate;
                    float lateralVelocity = localWheelVelocity.x;
                    
                    // 質量ベースのダンピング力
                    float lateralForce = -lateralCancelCoeff * lateralVelocity * wheel.SprungMass;
                    Vector3 lateralCancelForce = wheel.WheelHit.sidewaysDir * lateralForce;
                    
                    _rigidbody.AddForceAtPosition(lateralCancelForce, wheel.WheelHit.point, ForceMode.Force);

                    // if (_enableDebugLogs)
                    // {
                    //     string wheelName = isFrontWheel ? "Front" : "Rear";
                    //     Debug.Log($"[Wheel][{wheelName}] latVel={lateralVelocity:F3} coeff={lateralCancelCoeff:F1} force={lateralForce:F1}N (SprungMass={wheel.SprungMass:F1}kg)");
                    // }

                    // --- 後輪差動駆動: PID速度制御 ---
                    if (isLeftRearWheel || isRightRearWheel)
                    {
                        // 現在の前進速度 [m/s]
                        float currentVelocity = localWheelVelocity.y;
                        
                        // 目標速度を取得
                        float targetVelocity = isLeftRearWheel ? _leftTargetVelocity : _rightTargetVelocity;
                        
                        // デバッグ: 速度の符号確認
                        // if (_enableDebugLogs)
                        // {
                        //     string side = isLeftRearWheel ? "Left" : "Right";
                        //     Debug.Log($"[WheelVel][{side}] localWheelVelocity.y={localWheelVelocity.y:F3} | wheelVelocity={wheelVelocity.magnitude:F3} forwardDir={wheel.WheelHit.forwardDir}");
                        // }

                        float wheelSpeedMps = localWheelVelocity.y; // 回転速度から計算した車輪接地面速度 [m/s]
                        
                        if(isLeftRearWheel)  LeftRearWheelVelocity = wheelSpeedMps;
                        else RightRearWheelVelocity = wheelSpeedMps;

                        // PID制御で「加速度」[m/s²] を計算
                        float error = targetVelocity - currentVelocity;
                        
                        // 積分項（アンチワインドアップ付き）
                        float integral = isLeftRearWheel ? _leftIntegral : _rightIntegral;
                        integral += error * dt;
                        integral = Mathf.Clamp(integral, -_wheelIntegralClamp, _wheelIntegralClamp);
                        
                        // 微分項（スパイク防止のため制限付き）
                        float prevError = isLeftRearWheel ? _leftPrevError : _rightPrevError;
                        float derivative = 0f;
                        
                        // dtが異常に小さい場合は微分を使わない（スパイク防止）
                        if (dt > 0.001f) // 1ms以上の場合のみ微分を計算
                        {
                            derivative = (error - prevError) / dt;
                            // 微分項の異常値を制限（±50 m/s²以内）
                            derivative = Mathf.Clamp(derivative, -50f, 50f);
                        }
                        
                        // PID出力は「加速度」[m/s²]
                        float pidAcceleration = _wheelKp * error + _wheelKi * integral + _wheelKd * derivative;
                        
                        // 加速度を力 [N] に変換（F = m * a）
                        // 駆動輪は2つなので、車両質量の半分を各輪に割り当て
                        float wheelMass = _rigidbody.mass * 0.5f; // 後輪2つで質量を分割
                        float driveForce = pidAcceleration * wheelMass;
                        
                        // 力を制限（安全装置）
                        driveForce = Mathf.Clamp(driveForce, -_maxForcePerWheel, _maxForcePerWheel);
                        
                        // 力を適用
                        Vector3 driveForceVector = wheel.WheelHit.forwardDir * driveForce * ForwardSlipMultiplier;
                        _rigidbody.AddForceAtPosition(driveForceVector, wheel.WheelHit.point, ForceMode.Force);
                        
                        // 状態を保存
                        if (isLeftRearWheel)
                        {
                            _leftIntegral = integral;
                            _leftPrevError = error;
                        }
                        else
                        {
                            _rightIntegral = integral;
                            _rightPrevError = error;
                        }
                        
                        // if (_enableDebugLogs)
                        // {
                        //     string side = isLeftRearWheel ? "Left" : "Right";
                        //     Debug.Log($"[WheelPID][{side}] target={targetVelocity:F3} current={currentVelocity:F3} error={error:F3} dt={dt:F4} | P={_wheelKp*error:F3} I={_wheelKi*integral:F3} D={_wheelKd*derivative:F3} | accel={pidAcceleration:F3} force={driveForce:F3}N");
                        // }
                    }

                    // 抵抗力を適用（全輪共通）
                    var resistForce = eachWheelDeceleration * (_rigidbody.mass / Wheels.Length) * wheel.WheelHit.forwardDir * ForwardSlipMultiplier;
                    _rigidbody.AddForceAtPosition(resistForce, wheel.WheelHit.point, ForceMode.Force);
                }
            }
        }
    }
}
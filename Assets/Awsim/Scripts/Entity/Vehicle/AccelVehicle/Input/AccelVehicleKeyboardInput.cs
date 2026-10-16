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

using UnityEngine;

namespace Awsim.Entity
{
    /// <summary>
    /// Controlling vehicle input via keyboard.
    /// For the differential-drive WHILL the arrow keys give velocity targets (as cmd_vel does):
    /// held = drive, released = stop. Pressing an arrow key switches the control mode to Manual
    /// (cmd_vel is ignored); C switches back to Autonomous (ROS 2).
    /// </summary>
    /// ----- Default key binds -----
    /// up / down arrow     : forward / backward at _linearVelocity
    /// left / right arrow  : turn left / right at _yawRate (combine with up/down for curves)
    /// left shift (held)   : x _fastMultiplier
    /// C                   : back to Autonomous (ROS 2 cmd_vel)
    /// D                   : Drive gear
    /// P                   : Parking gear
    /// R                   : Reverse gear
    /// N                   : Neutral gear
    /// 1                   : Left turn signal
    /// 2                   : Right turn signal
    /// 3                   : Hazard
    /// 4                   : Turn signal off
    public class AccelVehicleKeyboardInput : MonoBehaviour, IAccelVehicleInput
    {
        public string Name => "Keyboard";
        public float AccelerationInput { get; private set; }
        public float SteerAngleInput { get; private set; }
        // Drive by default: in Manual mode the vehicle takes this gear, and Parking (the enum default)
        // disables the velocity control of the differential drive.
        public Gear GearInput { get; private set; } = Gear.Drive;
        public TurnIndicators TurnIndicatorsInput { get; private set; }
        public HazardLights HazardLightsInput { get; private set; }
        public bool SwitchAutonomous { get; private set; }
        public float TargetLinearVelocity { get; private set; }   // [m/s]
        public float TargetYawRate { get; private set; }          // [rad/s], positive = left (counter-clockwise)

        [Header("Velocity targets (differential drive)")]
        [SerializeField, Tooltip("Linear velocity while up/down is held [m/s]")] float _linearVelocity = 0.5f;
        [SerializeField, Tooltip("Yaw rate while left/right is held [rad/s]")] float _yawRate = 0.5f;
        [SerializeField, Tooltip("Factor while the fast key is held")] float _fastMultiplier = 2f;
        [SerializeField] KeyCode _fastKey = KeyCode.LeftShift;

        [SerializeField] KeyCode _accelerationKey = KeyCode.UpArrow;
        [SerializeField] KeyCode _decelerationKey = KeyCode.DownArrow;
        [SerializeField] KeyCode _leftSteeringKey = KeyCode.LeftArrow;
        [SerializeField] KeyCode _rightSteeringKey = KeyCode.RightArrow;
        [SerializeField] KeyCode _driveGearKey = KeyCode.D;
        [SerializeField] KeyCode _parkingGearKey = KeyCode.P;
        [SerializeField] KeyCode _reverseGearKey = KeyCode.R;
        [SerializeField] KeyCode _neutralGearKey = KeyCode.N;
        [SerializeField] KeyCode _leftTurnSignalKey = KeyCode.Alpha1;
        [SerializeField] KeyCode _rightTurnSignalKey = KeyCode.Alpha2;
        [SerializeField] KeyCode _hazardSignalKey = KeyCode.Alpha3;
        [SerializeField] KeyCode _noneSignalKey = KeyCode.Alpha4;
        [SerializeField] KeyCode _switchAutonomousKey = KeyCode.C;
        //[SerializeField] Component _readonlyVehicleComponent = null;
        //IReadOnlyAccelVehicle _readonlyVehicle = null;

        public void Initialize()
        {
            //_readonlyVehicle = _readonlyVehicleComponent as IReadOnlyAccelVehicle;
        }

        public bool UpdateInputs()
        {
            bool isOverridden = false;


            // Get switch autonomous.
            SwitchAutonomous = false;
            if (Input.GetKey(_switchAutonomousKey))
                SwitchAutonomous = true;

            // Velocity targets. Acceleration / steering inputs are not used by the differential
            // drive; they stay 0 so that the vehicle behaves as with cmd_vel.
            AccelerationInput = 0f;
            SteerAngleInput = 0f;
            float scale = Input.GetKey(_fastKey) ? _fastMultiplier : 1f;
            float v = 0f, w = 0f;
            if (Input.GetKey(_accelerationKey))
                v += _linearVelocity;
            if (Input.GetKey(_decelerationKey))
                v -= _linearVelocity;
            if (Input.GetKey(_leftSteeringKey))
                w += _yawRate;
            if (Input.GetKey(_rightSteeringKey))
                w -= _yawRate;
            TargetLinearVelocity = v * scale;
            TargetYawRate = w * scale;
            isOverridden = Input.GetKey(_accelerationKey) || Input.GetKey(_decelerationKey)
                || Input.GetKey(_leftSteeringKey) || Input.GetKey(_rightSteeringKey);

            // Get shift.
            if (Input.GetKey(_driveGearKey))
                GearInput = Gear.Drive;
            else if (Input.GetKey(_parkingGearKey))
                GearInput = Gear.Parking;
            else if (Input.GetKey(_reverseGearKey))
                GearInput = Gear.Reverse;
            else if (Input.GetKey(_neutralGearKey))
                GearInput = Gear.Neutral;
            //else
                //GearInput = _readonlyVehicle.GearInput;

            // Get turn signal.
            if (Input.GetKey(_leftTurnSignalKey))               // Left turn signal.
                TurnIndicatorsInput = TurnIndicators.Left;
            else if (Input.GetKey(_rightTurnSignalKey))         // Right turn signal.
                TurnIndicatorsInput = TurnIndicators.Right;
            else if (Input.GetKey(_noneSignalKey))              // None.
                TurnIndicatorsInput = TurnIndicators.None;
            //else
                //TurnIndicatorsInput = _readonlyVehicle.TurnIndicatorsInput;

            // Get hazard signal.
            if (Input.GetKeyDown(_hazardSignalKey))
            {
                if (HazardLightsInput == HazardLights.Enable)
                    HazardLightsInput = HazardLights.Disable;
                else
                    HazardLightsInput = HazardLights.Enable;
            }
            //else
                //HazardLightsInput = _readonlyVehicle.HazardLightsInput;

            return isOverridden;
        }
    }
}
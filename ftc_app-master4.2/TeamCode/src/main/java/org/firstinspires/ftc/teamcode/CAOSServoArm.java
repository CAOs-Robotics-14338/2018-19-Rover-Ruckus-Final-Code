/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

 /**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Servo Arm", group = "Linear Opmode")
@Disabled
public class CAOSServoArm extends LinearOpMode {

    private static final double INCREMENT   = 0.05;       // amount to slew servo each CYCLE_MS cycle
    // private static final int    CYCLE_MS    =   50;        // period of each cycle
    private static final double BASE_SET_POS  =  0.55;//.6      // Minimum rotational position
    private static final double LOWER_ARM_SET_POS  =  0.9; // Minimum rotational position
    private static final double UPPER_ARM_SET_POS  =  0.2; // Minimum rotational position
    private static final double WRIST_SET_POS  =  0.0;     // Minimum rotational position

    private static final double MAX_BASE_POS     =  0.65;                 // Maximum rotational position
    private static final double MIN_BASE_POS     =  0.45;        // Minimum rotational position
    private static final double MAX_LOW_ARM_POS  =  0.7;                 // Maximum rotational position
    private static final double MIN_LOW_ARM_POS  =  0.2;   // Minimum rotational position
    private static final double MAX_UPPER_ARM_POS  =  0.7;               // Maximum rotational position
    private static final double MIN_UPPER_ARM_POS  =  0.2; // Minimum rotational position
    private static final double MAX_WRIST_POS  =  1.0;                   // Maximum rotational position
    private static final double MIN_WRIST_POS  =  0;         // Minimum rotational position


     // Define class members
    private Servo armBase;
    private Servo lowerArm;
    private Servo upperArm;
    private Servo wrist;
    private double  basePosition = BASE_SET_POS;     //(MAX_BASE_POS - MIN_BASE_POS) / 2; Start at halfway position
    private double  lowerArmPosition = LOWER_ARM_SET_POS; //(MAX_LOW_ARM_POS - MAX_LOW_ARM_POS) / 2;
    private double  upperArmPosition = UPPER_ARM_SET_POS; //(MAX_UPPER_ARM_POS - MAX_UPPER_ARM_POS) / 2;
    private double  wristPosition = WRIST_SET_POS;    //(MAX_WRIST_POS - MAX_WRIST_POS) / 2;
    // boolean rampUp = true;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        armBase =  hardwareMap.get(Servo.class, "arm_base");
        lowerArm = hardwareMap.get(Servo.class, "lower_arm");
        upperArm = hardwareMap.get(Servo.class, "upper_arm");
        wrist =    hardwareMap.get(Servo.class, "wrist");


        armBase.setPosition(basePosition);
        lowerArm.setPosition(lowerArmPosition);
        upperArm.setPosition(upperArmPosition);
        wrist.setPosition(wristPosition);


        // Wait for the start button
     //   telemetry.addData(">", "Press Start to scan Servo." );
     //   telemetry.update();
     //   waitForStart();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the (direction) variable.
            //basePosition += INCREMENT ;
            //lowerArmPosition += INCREMENT ;
            telemetry.addData("Arm Base: ",armBase.getPosition());
            telemetry.addData("Lower Arm: ", lowerArm.getPosition());
            telemetry.addData("Upper Arm: ", upperArm.getPosition());
            telemetry.addData("Wrist: ",  wrist.getPosition());
            telemetry.update();


            if (gamepad1.left_stick_x >= 0.1) {
                basePosition += INCREMENT;
                if (basePosition >= MAX_BASE_POS ) {
                   basePosition = MAX_BASE_POS;
                }
            } else {
                if (gamepad1.left_stick_x < -0.1 ) {
                    basePosition -= INCREMENT;
                    if (basePosition <= MIN_BASE_POS ) {
                        basePosition = MIN_BASE_POS;
                    }
                }
            }
            armBase.setPosition(basePosition);

            if (gamepad1.left_stick_y >= 0.1) {
                lowerArmPosition -= INCREMENT;
                if (lowerArmPosition <= MAX_LOW_ARM_POS ) {
                    lowerArmPosition = MAX_LOW_ARM_POS;
                }
            } else {
                if ( gamepad1.left_stick_y < -0.1 ) {
                    lowerArmPosition += INCREMENT;
                    if (lowerArmPosition >= MIN_LOW_ARM_POS ) {
                       lowerArmPosition = MIN_LOW_ARM_POS;
                    }
                }
            }
            lowerArm.setPosition(lowerArmPosition);

            if (gamepad1.right_stick_x >= 0.1) {
                upperArmPosition += INCREMENT;
                if (upperArmPosition >= MAX_UPPER_ARM_POS ) {
                    upperArmPosition = MAX_UPPER_ARM_POS;
                }
            } else {
                if ( gamepad1.right_stick_x < -0.1 ) {
                    upperArmPosition -= INCREMENT;
                    if (upperArmPosition <= MIN_UPPER_ARM_POS ) {
                      upperArmPosition = MIN_UPPER_ARM_POS;
                    }
                }
            }
            upperArm.setPosition(upperArmPosition);

            if (gamepad1.right_stick_y >= 0.1) {
                wristPosition += INCREMENT;
                if (wristPosition >= MAX_WRIST_POS ) {
                   wristPosition = MAX_WRIST_POS;
                }
            } else {
                if ( gamepad1.right_stick_y < -0.1 ) {
                    wristPosition -= INCREMENT;
                    if (wristPosition <= MIN_WRIST_POS ) {
                       wristPosition = MIN_WRIST_POS;
                    }
                }
            }
            wrist.setPosition(wristPosition);

/*
            position += INCREMENT ;

            if (position >= MAX_POS ) {
                position = MAX_POS;
                rampUp = !rampUp;   // Switch ramp direction
            }
        }
            else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }
*/
            // Display the current value
       //     telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
          //  sleep(CYCLE_MS);
           // idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

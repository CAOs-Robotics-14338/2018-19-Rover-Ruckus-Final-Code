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
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Marker Deployment", group="Linear Opmode")
@Disabled
public class MarkerDeploymentANNOTATED extends LinearOpMode {

    // Declare OpMode members.

    Servo deploy;


    @Override
    /*This is a Teleop Op Mode that allows us to test the Marker Deployment System
     *The elements of this opMode are used in Autonomous and the Main teleop OpModes.
     */
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //gets the servo position of the servo for marker deployment
        deploy = hardwareMap.servo.get("deploy_marker");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        sleep(5);
        deployMarker();//run the deploy marker method

        /*This section is also in the main teleop section
         * It allows to get the marker off the robot if a problem occurs in teleop as well
         * as mess with Handy Manny.
         */
        while (opModeIsActive()) { //while teleop is running
            if (gamepad1.y) { //if the y button is pressed
                deploy.setPosition(0); //deploy marker
            } else if (gamepad1.a) { //if the a button is pressed return to folded position
                deploy.setPosition(.75);
            } else {
                deploy.setPosition(0.75); //if nothing pressed return to folded position

            }
        }
    }

    public void deployMarker() { //method for the sequencing of deploying the marker
        deploy.setPosition(0.75); //set deploy servo to home position
        sleep(1000);//4000
        deploy.setPosition(0);// set deploy servo to deloy marker location
        sleep(1000);//4000
        deploy.setPosition(0.75); //set deploy servo to home position
    }

    //method accessed by the other classes in order for marker deployment
    public void deployMarker(Servo servo) { //this method allows for the other classes to access this method. Uses same sequence as above just calling the servo that was inputted to do the actions.
        servo.setPosition(0.75);//set servo to home position
        sleep(1000);//4000
        servo.setPosition(0); //set servo to deploy marker position
        sleep(1000);//4000
        servo.setPosition(0.75);//set servo back to home position
    }

    public void setMarkerPosition(double pos) {
        deploy.setPosition(pos);// set the deploy marker position of servo
    }
    public void setMarkerPosition(Servo servo, double pos){
        servo.setPosition(pos);
    } //sets servo to position to easily identify purpose of positioning.

    public void setMarkerPositionToDeploy(Servo servo) {
        servo.setPosition(0); // set the position of the servo to the deploy location
    }

    public void setMarkerPositionToHome(Servo servo) {
        servo.setPosition(0.75); // set the position of the servo to the home location
    }
}
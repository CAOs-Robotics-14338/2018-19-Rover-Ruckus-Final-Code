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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="CAOS Teleop Slide V5", group="Linear Opmode")
@Disabled
public class CAOSSlideLinearV5 extends LinearOpMode {

    // Declare OpMode members.
    //Declare Objects to be able to use later
    private ElapsedTime runtime = new ElapsedTime();

    //Declare motor objects in order drive systems
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor centerDrive = null;
    private DcMotor centerRise = null;

    private DcMotor climbMotor = null;

    //declare limit switch
    DigitalChannel digitalTouch;
// declare marker deployment servo
    Servo marker;

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    //COLLECTING ARM
    // Define objects for the collecting system
    private DcMotor collector = null;
    private DcMotor armLift = null;
    private CRServo reel;
    private double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    private double  reelPower = 0; // Start at halfway position
    private double  collectorPower = 0; // Start at halfway position
    private double  armLiftPower = 0; // Start at halfway position


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //getting motor position values from the Expansion Hub
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");
        centerRise = hardwareMap.get(DcMotor.class, "center_rise");

        climbMotor = hardwareMap.get(DcMotor.class, "climb_motor");

        // get a reference to our digitalTouch object.
        //get position and values from Expansion hub
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input. This sets the limit switch to get input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        //gets the reference and the position/value of servo controlling the marker deployment
        marker  = hardwareMap.servo.get("deploy_marker");

        //COLLECTING ARM

        //getting the position values for the collecting system.
        reel = hardwareMap.get(CRServo.class, "reel_servo");
        collector  = hardwareMap.get(DcMotor.class, "collector");
        armLift = hardwareMap.get(DcMotor.class, "armLift");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        //Set the direction of the motors in order to improve control.
        /*
        The motor direction of certain motors needs to be reversed due to their position on the robot.
        For example, with the drive system one of the motors needs to be reversed in order for both motors to go the same direction.
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        centerDrive.setDirection(DcMotor.Direction.REVERSE);
        centerRise.setDirection(DcMotor.Direction.REVERSE);

        climbMotor.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double centerPower;
            double centerRisePower;
            double climbPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn = -gamepad1.right_stick_y;
            double drive  =  -(gamepad1.right_stick_y);
            double turn  =  -(gamepad1.right_stick_x);
            double slide  =  -(gamepad1.left_stick_x);
            //double rise  =  -(gamepad1.left_stick_y);


            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            centerPower   = Range.clip(slide, -1.0, 1.0) ;
            //centerRisePower   = Range.clip(rise, -1.0, 1.0);
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            ///SPECIAL CONTROL FOR ARM CONTROL
            if((gamepad1.left_bumper || gamepad1.right_bumper)){
                leftPower    = Range.clip(drive + turn, -.5, 0.5) ;
                rightPower   = Range.clip(drive - turn, -0.50, .50) ;
                centerPower   = Range.clip(slide, -0.50, 0.50) ;
            }

            leftDrive.setPower(-leftPower);
            rightDrive.setPower(-rightPower);
            centerDrive.setPower(-centerPower);



            if (gamepad1.dpad_up && gamepad1.b) {
                centerRisePower = 0.8;
            } else {
                if ( gamepad1.dpad_down && gamepad1.b )
                    centerRisePower = -0.8;
                else
                    centerRisePower = 0.0;
            }
            centerRise.setPower(centerRisePower);

            if(gamepad1.dpad_left ){//going down
                climbPower = 0.8;
            }else{
                if(gamepad1.dpad_right&& !digitalTouch.getState()) {////digitalTouch.getState() == false//going up
                    climbPower = -0.8;
                }
                else
                    climbPower = 0.0;
            }
            climbMotor.setPower(climbPower);

            if(gamepad1.y){
                marker.setPosition(0);
                sleep(1000);
            }else if (gamepad1.a){
                marker.setPosition(.75);
            }else{
                marker.setPosition(0.75);
            }

            //COLLECTING ARM
            //if (gamepad2.a) {
            reelPower = Range.clip(-gamepad2.left_stick_y, -1.0, 1.0) ;
            if (reelPower > 0){
                position = 1.0;
            } else {
                // if ( gamepad2.b ) {
                if (reelPower < 0){
                    position = -1.0;
                }
                else
                    position = 0.0;
            }
            reel.setPower(position);

            if (gamepad2.right_trigger > 0){
                collectorPower = 0.6;
            } else {
                if (gamepad2.left_trigger > 0){
                    collectorPower = -0.6;
                }
                else
                    collectorPower = 0.0;
            }
            collector.setPower(collectorPower);

            if(gamepad2.left_bumper == true){//was -.1 as min
                armLiftPower = Range.clip(-gamepad2.right_stick_y, -0.15, .40); //min was -.1 //.30
                armLift.setPower(armLiftPower);//reversed power to reverse control was previously --> -armLiftPower   //
            }
            else{    //was =-0.1 as min
                armLiftPower = Range.clip(-gamepad2.right_stick_y, -0.15, .70); //min was -.7 //.30
                armLift.setPower(armLiftPower);//reversed power to reverse control was previously --> -armLiftPower   //
            }



            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

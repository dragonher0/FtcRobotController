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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tank Mode with Mechanisms", group="Iterative Opmode")

public class TankMode_WithMechanism extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private Servo wrist = null;
    private Servo claw = null;
    private Servo launcher = null;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public double buttonPressToPower (boolean buttonPress) {
        double buttonPower = 0.0;
        if (buttonPress) {
            buttonPower = 90;
        }
        return buttonPower;
    }


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        launcher = hardwareMap.get(Servo.class, "launcher");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(270);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(0);
        launcher.setDirection(Servo.Direction.REVERSE);
        launcher.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        initAprilTag();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double armPower;
        boolean wristUp;
        boolean wristDown;
        boolean clawButtonOpen;
        boolean clawButtonClose;
        boolean defaultAutoButton;
        boolean scoreButton;
        double x = claw.getPosition();
        double y = wrist.getPosition();
        boolean launchButton;
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower  = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;
        armPower = -gamepad2.left_stick_y;
        wristUp = gamepad2.dpad_up;
        wristDown = gamepad2.dpad_down;
        clawButtonOpen = gamepad2.a;
        clawButtonClose = gamepad2.b;
        defaultAutoButton = gamepad2.y;
        scoreButton = gamepad2.x;
        launchButton = gamepad2.right_bumper;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        telemetryAprilTag();

        if (clawButtonOpen) {
            claw.setPosition(0);
            if (x == 0) {
                if (!clawButtonOpen) {
                    claw.setPosition(0);
                } else if (clawButtonClose) {
                    claw.setPosition(180);
                }
            }
        } else if (clawButtonClose) {
            claw.setPosition(180);
            if (x >= 180) {
                if (!clawButtonClose) {
                    claw.setPosition(180);
                } else if (clawButtonOpen) {
                    claw.setPosition(0);
                }
            }
        }
        if (wristDown) {
            wrist.setPosition(0);
            if (y == 0) {
                if (!wristDown) {
                    wrist.setPosition(0);
                } else if (wristUp) {
                    claw.setPosition(180);
                }
            }
        } else if (wristUp) {
            wrist.setPosition(180);
            if (y >= 180) {
                if (!wristUp) {
                    claw.setPosition(180);
                } else if (wristDown) {
                    claw.setPosition(0);
                }
            }
        }
        if (defaultAutoButton) {
            leftArm.setPower(1);
            leftArm.setTargetPosition(0);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setPower(1);
            rightArm.setTargetPosition(0);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(0);
        } else if (scoreButton) {
            leftArm.setPower(1);
            leftArm.setTargetPosition(550);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setPower(1);
            rightArm.setTargetPosition(550);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(180);
        } else if (armPower == 0){
            leftArm.setTargetPosition(leftArm.getCurrentPosition());
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setTargetPosition(rightArm.getCurrentPosition());
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftArm.setPower(armPower/2);
            rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightArm.setPower(armPower/2);
        }

        launcher.setPosition(buttonPressToPower(launchButton));

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Arm", leftArm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }
}

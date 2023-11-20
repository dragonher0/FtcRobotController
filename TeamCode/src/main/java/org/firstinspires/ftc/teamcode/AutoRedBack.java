package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import android.util.Size;
import org.firstinspires.ftc.teamcode.FirstPipelineRevised;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous(name="Auto Red Backstage")
public class AutoRedBack extends LinearOpMode {
    private FirstPipelineRevised firstPipelineRevised; //Create an object of the VisionProcessor Class
    private VisionPortal portal;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo claw = null;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        firstPipelineRevised = new FirstPipelineRevised();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(firstPipelineRevised)
                .setCameraResolution(new Size(1280, 720))
                .build();
        imu.resetYaw();
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        claw = hardwareMap.get(Servo.class, "claw");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //claw.setPosition(0);
        sleep(1000);
        claw.setPosition(180);
        waitForStart();
        portal.setProcessorEnabled(firstPipelineRevised, true);
        while (opModeIsActive()) {
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.update();
            sleep(5000);
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.update();
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();
            double selection = firstPipelineRevised.getSelection();
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            if (selection == 1) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                while (leftDrive.getCurrentPosition() < 2400) {}
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            } else if (selection == 3) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                while (leftDrive.getCurrentPosition() < 2400) {}
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            if (selection == 1) {
                while (Yaw < 55 && opModeIsActive()){
                    rightDrive.setPower(0.5);
                    telemetry.addData("Yaw", Yaw);
                    telemetry.update();
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(500);
                while (Yaw > 0 && opModeIsActive()){
                    rightDrive.setPower(-0.5);
                    telemetry.addData("Yaw", Yaw);
                    telemetry.update();
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                }
            } else if (selection == 3) {
                while (Yaw > -45 && opModeIsActive()){
                    leftDrive.setPower(0.5);
                    telemetry.addData("Yaw", Yaw);
                    telemetry.update();
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                }
                leftDrive.setPower(-0.5);
                rightDrive.setPower(0);
                while (Yaw < 0 && opModeIsActive()){
                    leftDrive.setPower(-0.5);
                    telemetry.addData("Yaw", Yaw);
                    telemetry.update();
                    robotOrientation = imu.getRobotYawPitchRollAngles();
                    Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                }
            } else if (selection == 2) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                while (leftDrive.getCurrentPosition() < 3700) {
                    telemetry.addLine(String.valueOf(leftDrive.getCurrentPosition()));
                    telemetry.update();
                }
                leftDrive.setPower(-0.5);
                rightDrive.setPower(-0.5);
                while (leftDrive.getCurrentPosition() > 1900) {
                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            break;
        }



    }
}

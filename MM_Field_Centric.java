package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Field Centric Nav", group="Linear OpMode")
public class MM_Field_Centric extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu = null;

    double flPower = 0;
    double frPower = 0;
    double blPower = 0;
    double brPower = 0;

    double maxPower;

    private YawPitchRollAngles orientation = null;
    double angle = 0;

    double strafePower = 0; // robot-centric, used to calculate x and y power
    double drivePower = 0; // same as above
    double rotatePower = 0;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            orientation = imu.getRobotYawPitchRollAngles();
            angle = orientation.getYaw(AngleUnit.DEGREES);

            calculateAndSetDrivePowers();

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            telemetry.addLine("GYRO TELEMETRY:\n");
            telemetry.addData("Angle", "%.2f Deg. (Heading)\n", orientation.getYaw(AngleUnit.DEGREES));

            telemetry.addLine("DRIVE TELEMETRY:\n");
            telemetry.addData("Front left power", "%4.2f", flPower);
            telemetry.addData("Front right power", "%4.2f", frPower);
            telemetry.addData("Back left power", "%4.2f", blPower);
            telemetry.addData("Back right power", "%4.2f", brPower);
            telemetry.update();
        }
    }

    private void calculateAndSetDrivePowers() {
        drivePower = (Math.sin(Math.toRadians(angle)) * -gamepad1.left_stick_y) + (Math.cos(Math.toRadians(angle)) * gamepad1.left_stick_x);
        strafePower = (-Math.cos(Math.toRadians(angle)) * -gamepad1.left_stick_y) + (Math.sin(Math.toRadians(angle)) * gamepad1.left_stick_x);
        rotatePower = gamepad1.right_stick_x;

        flPower = drivePower + strafePower + rotatePower;
        frPower = drivePower - strafePower - rotatePower;
        blPower = drivePower - strafePower + rotatePower;
        brPower = drivePower + strafePower - rotatePower;

        //normalize
        maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    private void initializeIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot)); // if you choose two conflicting directions, this initialization will cause a code exception.
    }
}
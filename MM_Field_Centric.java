package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sin;

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
    double angle = 0; //TODO get angle
    double strafeAngleError = 0;
    double driveAngleError = 0;

    double strafeRatio = 0; //TODO add comment
    double driveRatio = 0; // same as above

    double strafeVector = 0; // robot-centric, used to calculate x and y power
    double driveVector = 0; // same as above

    double yPower = 0; // pushing stick forward gives negative value
    double xPower = 0;
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
            strafeVector *= gamepad1.left_stick_x;
            driveVector *= -gamepad1.left_stick_y;
            rotatePower = gamepad1.right_stick_x;

            orientation = imu.getRobotYawPitchRollAngles();
            angle = orientation.getYaw(AngleUnit.DEGREES);

            getAngleError();
            calculateVectors();
            calculateDrivePowers();
            setDrivePowers();

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

    private void getAngleError() {
        if (angle >= 0 && angle < 90) { // quadrant 1
            strafeAngleError = -(90 - angle);
            driveAngleError = angle;
        } else if (angle >= 90 && angle < 180) { // quadrant 2
            strafeAngleError = -(90 - angle);
            driveAngleError = 180 - angle;
        } else if (angle >= -180 && angle < -90) { // quadrant 3
            strafeAngleError = -90 - angle;
            driveAngleError = -180 - angle;
        } else if (angle >= -90 && angle < 0) { // quadrant 4
            strafeAngleError = -90 - angle;
            driveAngleError = angle;
        }
    }

    private void calculateDrivePowers() {
        flPower = driveVector + strafeVector + rotatePower;
        frPower = driveVector - strafeVector - rotatePower;
        blPower = driveVector - strafeVector + rotatePower;
        brPower = driveVector + strafeVector - rotatePower;

        //normalize
        maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }
    }

    private void setDrivePowers() {
        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    private void calculateVectors() {
        strafeRatio = sin(strafeAngleError);
        driveRatio = sin(driveAngleError);

        strafeVector = strafeRatio / (strafeRatio + driveRatio);
        driveVector = driveRatio / (strafeRatio + driveRatio);
    }

    private void initializeIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot)); // if you choose two conflicting directions, this initialization will cause a code exception.
    }
}
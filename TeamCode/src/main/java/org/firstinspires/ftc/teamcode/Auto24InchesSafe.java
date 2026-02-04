package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto 24in Forward & Back - Decode", group = "Autonomous")
public class Auto24InchesSafe extends LinearOpMode {

    // Motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    // Distance Sensor
    private DistanceSensor frontDistance;

    // Encoder constants (adjust for your robot)
    static final double COUNTS_PER_MOTOR_REV = 537.7; // goBILDA 312 RPM
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH =
            COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    // Safety distance (in inches)
    static final double MIN_DISTANCE_INCHES = 8.0;

    @Override
    public void runOpMode() {

        // Initialize hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();

        telemetry.addLine("Initialized. Ready for Auto.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Move forward 24 inches
            encoderDriveSafe(0.4, 24);

            sleep(500);

            // Move backward 24 inches
            encoderDriveSafe(0.4, -24);

            stopRobot();
        }
    }

    // Encoder-based drive with distance safety
    private void encoderDriveSafe(double speed, double inches) {

        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        int leftTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
        leftTarget += leftBackDrive.getCurrentPosition() + moveCounts;
        int rightTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
        rightTarget += rightBackDrive.getCurrentPosition() + moveCounts;




        leftFrontDrive.setTargetPosition(leftTarget);
        leftBackDrive.setTargetPosition(leftTarget);
        rightFrontDrive.setTargetPosition(rightTarget);
        rightBackDrive.setTargetPosition(rightTarget);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        while (opModeIsActive()
                && leftFrontDrive.isBusy()
                && leftBackDrive.isBusy()
                && rightFrontDrive.isBusy()
                && rightBackDrive.isBusy()) {

            double distance = frontDistance.getDistance(DistanceUnit.INCH);

            // Safety stop if too close
            if (distance < MIN_DISTANCE_INCHES) {
                telemetry.addLine("Obstacle detected! Stopping.");
                telemetry.update();
                stopRobot();
                return;
            }

            telemetry.addData("Distance (in)", distance);
            telemetry.update();
        }

        stopRobot();
        resetEncoders();
    }

    private void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopRobot() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}

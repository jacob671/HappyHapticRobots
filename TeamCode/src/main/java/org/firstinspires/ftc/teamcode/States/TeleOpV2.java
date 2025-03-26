package org.firstinspires.ftc.teamcode.States;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name = "TeleOpV2", group = "Linear Opmode")
public class TeleOpV2 extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, UpRightMotor,UpLeftMotor;
    private ClawSubsystem clawSubsystem;
    private Timer timer = new Timer();
    private Follower follower;
    private Servo ExtendServo, SpinServo;
    private boolean liftControlActive = false;
    private boolean bucketControlActive = false;
    private boolean isNavigating = false;
    private boolean isGrabbing = false;
    private boolean specimenInGrabber;
    private double divisor = 1;
    private static final double SERVO_MIN = 0 ;
    private static final double SERVO_MAX = 0.4;
    private final Pose scorePose = new Pose(5.4, 17.8, 2.32);
    private final Pose parkPose = new Pose(-55, 10, Math.toRadians(270));
    private double lastPosition = 0.2;
    final double SERVO_INCREMENT = 0.4; // Increment for smooth movement
    final double LOOP_DELAY = 0.05;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        ExtendServo = hardwareMap.get(Servo.class, "ExtendServo");
        SpinServo = hardwareMap.get(Servo.class, "ExtendServo");
        UpRightMotor = hardwareMap.get(DcMotor.class, "UpRightMotor");
        UpLeftMotor = hardwareMap.get(DcMotor.class, "UpLeftMotor");

        clawSubsystem = new ClawSubsystem(hardwareMap);
        follower = new Follower(hardwareMap);
        ExtendServo.setPosition(lastPosition);
        // Reverse the right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (!isNavigating) {
                if (!isGrabbing) {
                    // Manual driving control
                    lastPosition = 0.2;
                    double y = gamepad1.left_stick_y;
                    double x = -gamepad1.left_stick_x ;
                    double rx = gamepad1.right_stick_x;
                    double y2 = gamepad2.left_stick_y;
                    double denominator = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rx)) * divisor, 1 * divisor);

                    double frontLeftPower = (y + x + rx) * 1 / denominator;
                    double ExtendAmount = (y2) / denominator / 10;
                    double backLeftPower = (y - x + rx) * 1/ denominator;
                    double frontRightPower = (y - x - rx) * 1 / denominator;
                    double backRightPower = (y + x - rx) * 1 / denominator;

                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                } else if (isGrabbing) {

                    double y = gamepad1.left_stick_y;
                    double x = -gamepad1.left_stick_x ;
                    double rx = gamepad1.right_stick_x;
                    double y2 = gamepad2.left_stick_y;
                    double denominator = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rx)) * divisor, 1 * divisor);

                    double frontLeftPower = (y + x + rx) * 1 / denominator;
                    double ExtendAmount = (y2) / denominator / 10;
                    double backLeftPower = (y - x + rx) * 1/ denominator;
                    double frontRightPower = (y - x - rx) * 1 / denominator;
                    double backRightPower = (y + x - rx) * 1 / denominator;

                    // Assuming you want to use the left stick y-axis
                    if (gamepad1.right_bumper) {
                        // Move forward
                        lastPosition += 1 * SERVO_INCREMENT;
                    } else if (gamepad1.left_bumper) {
                        // Move backward
                        lastPosition += 1 * -SERVO_INCREMENT;
                    }

                    // Clamp position to valid servo range
                    lastPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, lastPosition));

                    // Update servo position
                    ExtendServo.setPosition(lastPosition);

                    // Save the last valid position

                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                }}


            // Navigation control for X and Y buttons
            if (gamepad2.x) {
                //navigateTo(scorePose);
                //sleep(1000);
                clawSubsystem.bucket();
            }
            if (gamepad2.y && !isNavigating) {
                //navigateTo(parkPose);
            }
            if(gamepad2.y){

               //clawSubsystem.moveBit();

            }


            if (gamepad1.dpad_right){
                clawSubsystem.specReadyForGrab();
            }
            if (gamepad1.dpad_left){
                clawSubsystem.specReadyForHang();
            }
            if (gamepad1.dpad_up){
                clawSubsystem.specHang();
            }


            if (gamepad1.left_stick_button){
                clawSubsystem.releaseSpec();
            }

            if (gamepad1.right_stick_button){
                clawSubsystem.grabSpec();

            }

            // Additional gamepad controls
            if (gamepad2.left_trigger > 0.4 && !liftControlActive) {
                clawSubsystem.extendOriginal();
                clawSubsystem.moveDown();

            } else if (gamepad2.right_trigger > 0.4 && !liftControlActive) {
                clawSubsystem.moveUp();




            }
            if (gamepad1.b){
                isGrabbing = true;
                divisor=4;
                clawSubsystem.reset();
            }
            if (gamepad1.right_trigger > 0.51) {
                isGrabbing = true;
                clawSubsystem.grabReady();
                divisor = 2;
                isGrabbing = true;

            } else if (gamepad1.left_trigger > 0.51) {

               //
                // sleep(300);
                divisor = 1;
                ExtendServo.setPosition(0);
                clawSubsystem.release();

                isGrabbing = false;
            }
            if (gamepad1.y) {
                isGrabbing = true;
                clawSubsystem.grab();
            }

            if (gamepad2.dpad_left) {
                clawSubsystem.grabVertical();
                clawSubsystem.bucketHold();
            }



            if (gamepad2.a){
                UpRightMotor.setPower(-1);
                UpLeftMotor.setPower(1);

            }
            if (gamepad2.b){

                UpRightMotor.setPower(0);
                UpLeftMotor.setPower(0);
            }

            if (gamepad2.right_bumper) {

                clawSubsystem.moveUpSp();
            }

            if (gamepad2.left_bumper) {

                clawSubsystem.extendOriginal();

                clawSubsystem.moveDownSpToRelease();
                specimenInGrabber = false;
            }
            if (gamepad2.dpad_down){
                    clawSubsystem.extendOriginal();

                    clawSubsystem.moveToPickingReady();
                    specimenInGrabber = true;
                }





            if (gamepad2.y) {
                clawSubsystem.sweep();

            }


            telemetry.addData("Front Left Power", frontLeftMotor.getPower());
            telemetry.addData("Back Left Power", backLeftMotor.getPower());
            telemetry.addData("Front Right Power", frontRightMotor.getPower());
            telemetry.addData("Back Right Power", backRightMotor.getPower());
            telemetry.update();
        }
    }}

/*
    private void navigateTo(Pose targetPose) {
        // Build a path to the target pose
        Pose currentPose = follower.getPose();
        Path path = new Path(new BezierLine(
                new Point(currentPose.getX(), currentPose.getY()),
                new Point(targetPose.getX(), targetPose.getY())
        ));
        path.setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading());

        // Start following the path
        follower.followPath(path, true);
        isNavigating = true;

        // Monitor path completion
        new Thread(() -> {
            while (opModeIsActive()) {
                follower.update();

                // Check if the robot is close enough to the target pose
                Pose currentPosition = follower.getPose();
                double distanceToTarget = Math.hypot(
                        targetPose.getX() - currentPosition.getX(),
                        targetPose.getY() - currentPosition.getY()
                );
                double angleDifference = Math.abs(targetPose.getHeading() - currentPosition.getHeading());

                // Stop navigation if within tolerance
                if (distanceToTarget < 2.0 && angleDifference < Math.toRadians(5)) { // Adjust thresholds as needed
                    break;
                }
            }
            isNavigating = false;
        }).start();


    }
    }

 */

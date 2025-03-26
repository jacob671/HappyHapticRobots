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

@TeleOp(name = "WheelTest", group = "Linear Opmode")
public class WheelTest extends LinearOpMode {

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
    private double lastPosition = 0.3;
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


                // Manual driving control
                lastPosition = 0.2;
                double y = gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;
                double y2 = gamepad2.right_stick_y;
                double denominator = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rx)) * divisor, 1 * divisor);

                double frontLeftPower = (y ) * 1 / denominator;
                //double ExtendAmount = (y2) / denominator / 10;
                double backLeftPower = (x) * 1 / denominator;
                double frontRightPower = (rx) * 1 / denominator;
                double backRightPower = (y2) * 1 / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

        }}
}
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

package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {

    // Elevator motor
    private DcMotor upLeftMotor;
    private DcMotor upRightMotor;
    // Claw servos
    private Servo spinServo;
    private Servo turnServo;
    private Servo extendServo;
    private Servo bucketServo;
    private Servo sweepServo;
    private Servo specTurnServo;
    private Servo specGrabServo;
    // Constants for elevator power
    private static final double ELEVATOR_POWER = 1.0;
    private static final int TICKS_PER_INCH = 100; // Adjust based on your motor and gearing
    private static final int FULL_LIFT_TICKS = 2150; // Total ticks for full lift height
    private static final int SMALL_LIFT_TICKS = 1800; // Ticks for small lift height

    private static final int New_SPICIMENT_PICKING_READY_TICKS = 340;
    private static final int New_SPICIMENT_READY_TO_RELEASE_TICKS = 1800; // Ticks for small lift height
    private static final int New_SPICIMENT_RELESED_TICKS = 1100; // release spiciment at new attachemnt

    private static final int BIT_LIFT_TICKS = 400; // Ticks for small adjustments
    private static final int SPICIMENT_RELEASE_TICKS = 5; // Ticks for small adjustments
    private static final int SPICIMENT_AFTER_RELEASE_TICKS = 1150; // Ticks for small adjustments

    private static final int STARTING_POSITION_TICKS = 30; // Define the starting position

    // Servo positions
    private static final double TURN_GRAB_POSITION = 0.22;
    private static final double TURN_RELEASE_POSITION = 0.7;
    private static final double TURN_VERTICAL_POSITION = 0.53;
    private static final double TURN_READY_POSITION = 0.3;
//skibdi toilet


    private static final double SPIN_GRAB_POSITION = .29;
    private static final double SPIN_RELEASE_POSITION = .8;
    private static final double SPIN_complete_POSITION = .15;

    private static final double BUCKET_DUMP_POSITION = 0.13;
    private static final double BUCKET_NEUTRAL_POSITION = 0.59;
    private static final double EXTEND_ORIGINAL_POSITION = 0;

    // Current state tracking
    private boolean liftControlActive = false;
    public int height = 0;
    // 0 = down, 1 = specimen height, 2 = full elevator height

    public ClawSubsystem(HardwareMap hardwareMap) {
        // Initialize elevator motor
        upLeftMotor = hardwareMap.get(DcMotor.class, "UpLeftMotor");
        upLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upRightMotor = hardwareMap.get(DcMotor.class, "UpRightMotor");
        upRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize claw servos
        spinServo = hardwareMap.get(Servo.class, "SpinServo");
        turnServo = hardwareMap.get(Servo.class, "TurnServo");
        extendServo = hardwareMap.get(Servo.class, "ExtendServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        sweepServo = hardwareMap.get(Servo.class, "sweepServo");
        specTurnServo = hardwareMap.get(Servo.class, "specTurnServo");
        specGrabServo = hardwareMap.get(Servo.class, "specGrabServo");
    }


    // Elevator Controls




    public void grabSpec(){
        specGrabServo.setPosition(0.5);
    }

    public void releaseSpec(){
        specGrabServo.setPosition(0.9);

    }
    public void specReadyForGrab(){
        extendServo.setPosition(0);
        specTurnServo.setPosition(0);
        specGrabServo.setPosition(0.9);
    }
    public void specReadyForHang(){
        specTurnServo.setPosition(.6);
        extendServo.setPosition(0);


    }

    public void specHang(){
        specTurnServo.setPosition(.5);

    }




    public void moveUp() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevator(-FULL_LIFT_TICKS, 2); // Set height to 2 when fully up
        }
    }

    public void moveDown() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(STARTING_POSITION_TICKS, 0); // Set height to 0 when fully down
        }
    }
    public void smallExtend(){
        extendServo.setPosition(0.2);
    }

    public void moveUpSp() {
        if (!liftControlActive) {
            liftControlActive = true;
            //moveElevator(-SMALL_LIFT_TICKS, 1); // Set height to 1 for small lift
            moveElevatorToPosition(-SMALL_LIFT_TICKS, 1);
        }
    }


    public void moveToPickingReady() {
        // at ready to pick, target height is 100.
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(-New_SPICIMENT_PICKING_READY_TICKS, 100);
        }
    }

    public void moveUpSpReadyToRelease() {
        // at ready to release, target height is 200
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(-New_SPICIMENT_READY_TO_RELEASE_TICKS, 200);
        }
    }


    public void moveDownSpToRelease() {
        // after release, target height is 300
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(-New_SPICIMENT_RELESED_TICKS, 300);
        }
    }

    public void moveUpSp_withMoveElevator() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevator(-SMALL_LIFT_TICKS, 1); // Set height to 1 for small lift
            //moveElevatorToPosition(-SMALL_LIFT_TICKS, 1);
        }
    }

    public void moveUpSpDownBit() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(-SPICIMENT_RELEASE_TICKS, 9); // Set height to 1 for small lift

            // moveElevator(500, 9); // Set height to 1 for small lift
            //moveElevatorToPosition(-SMALL_LIFT_TICKS + BIT_LIFT_TICKS, 9);

        }
    }


    public void moveUpSpDownBitThenUp() {
        if (!liftControlActive) {
            liftControlActive = true;
            //moveElevator(-SMALL_LIFT_TICKS, 11); // Set height to 1 for small lift
            // moveElevator(-100, 11); // Set height to 1 for small lift
            //moveElevatorToPosition(-SMALL_LIFT_TICKS, 11);
            moveElevatorToPosition(-SPICIMENT_AFTER_RELEASE_TICKS, 11);

        }
    }


    public void moveDownSp() {
        if (!liftControlActive) {
            liftControlActive = true;
            moveElevatorToPosition(STARTING_POSITION_TICKS, 0); // Set height to 0 when back to start
        }
    }
    public void extend() {
        specTurnServo.setPosition(0);
        extendServo.setPosition(0.5);

    }
    public void retract() {
        extendServo.setPosition(0.);

    }


    public void moveBit() {
        int currentPosition = (upLeftMotor.getCurrentPosition()+upRightMotor.getCurrentPosition())/2;
        int targetDownPosition = currentPosition + BIT_LIFT_TICKS;

        moveElevatorToPosition(targetDownPosition, height);

        // After reaching the lowered position, return to the original position
        new Thread(() -> {
            while (upLeftMotor.isBusy()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            moveElevatorToPosition(currentPosition, height);
        }).start();
    }
    public int getElevatorPosition(){
        return(upLeftMotor.getCurrentPosition());
    }

    private void moveElevator(int ticks, int targetHeight) {
        int targetPosition = (upLeftMotor.getCurrentPosition() +upRightMotor.getCurrentPosition())/2 + ticks;
        upLeftMotor.setTargetPosition(targetPosition);
        upLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        upRightMotor.setTargetPosition(-targetPosition);
        upRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upRightMotor.setPower(-ELEVATOR_POWER);
        upLeftMotor.setPower(ELEVATOR_POWER);

        new Thread(() -> {
            while (upLeftMotor.isBusy()) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            // Check if the current position is close enough to the target position
            if (Math.abs((upLeftMotor.getCurrentPosition() ) - targetPosition) <= 10) {
                height = targetHeight; // Update height only if the target position is reached
            } else {
                // Handle or log failure to reach target position
                System.out.println("Failed to reach the target position.");
            }

            stopElevator();
        }).start();
    }



    private void moveElevatorToPosition(int targetPosition, int targetHeight) {
        upLeftMotor.setTargetPosition(targetPosition);
        upLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upRightMotor.setTargetPosition(-targetPosition);
        upRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upRightMotor.setPower(-ELEVATOR_POWER);
        upLeftMotor.setPower(ELEVATOR_POWER);

        new Thread(() -> {
            while (upLeftMotor.isBusy()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            // Check if the current position matches the target position
            if (Math.abs(upLeftMotor.getCurrentPosition() - targetPosition) <= 10) {
                height = targetHeight; // Update height only if the target position is reached
            } else {
                // Log or handle a failure to reach the target position, if needed
                System.out.println("Target position not reached!");
            }

            stopElevator();
        }).start();
    }


    public void stopElevator() {
        upLeftMotor.setPower(0);
        upLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upRightMotor.setPower(0);
        upRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftControlActive = false;
    }

    // Claw and bucket controls
    public void bucket() {
        bucketServo.setPosition(BUCKET_DUMP_POSITION);
        delayAction(() -> bucketServo.setPosition(BUCKET_NEUTRAL_POSITION), 1000);
    }

    public void bucketHold() {
        bucketServo.setPosition(0);

    }

    public void grab1() {

       // delayAction(() ->

            delayAction(() -> spinServo.setPosition(SPIN_GRAB_POSITION), 300);

            turnServo.setPosition(TURN_GRAB_POSITION);


            //, 100);


    }
    public void grab() {

        // delayAction(() ->
        turnServo.setPosition(TURN_GRAB_POSITION);
        spinServo.setPosition(SPIN_GRAB_POSITION);




        //, 100);


    }

    public void grabReady() {
        //turnServo.setPosition(TURN_GRAB_POSITION);
       // spinServo.setPosition(SPIN_RELEASE_POSITION);
        turnServo.setPosition(TURN_READY_POSITION);
    }

    public void grabVertical() {
        extendServo.setPosition(0);
        turnServo.setPosition(TURN_VERTICAL_POSITION);
    }

    public void reset() {
        turnServo.setPosition(TURN_GRAB_POSITION);
        spinServo.setPosition(SPIN_RELEASE_POSITION);
    }


    public void sweep(){
        sweepServo.setPosition(0.45);
        delayAction(() -> sweepServo.setPosition(0.05), 500);
    }

    public void sweepBack(){
        sweepServo.setPosition(0.05);
    }

    public void release() {
        turnServo.setPosition(TURN_RELEASE_POSITION);
        delayAction(() -> spinServo.setPosition(SPIN_RELEASE_POSITION), 700);
    }
    public void releaseComplete() {
        turnServo.setPosition(TURN_RELEASE_POSITION);
        delayAction(() -> spinServo.setPosition(SPIN_complete_POSITION), 900);
    }

    public void extendOriginal() {
        extendServo.setPosition(EXTEND_ORIGINAL_POSITION);
        turnServo.setPosition(0.3);
    }


    // Claw and bucket controls
    public void pickandDumpRightSamples() {
        grab();
        delayAction(() -> release(), 500);
        delayAction(() -> grabVertical(), 500);
        delayAction(() -> bucket(), 500);


        //delayAction(() -> bucketServo.setPosition(BUCKET_NEUTRAL_POSITION), 1000);
    }


    // Utility to delay actions
    private void delayAction(Runnable action, long delay) {
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                action.run();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }
}

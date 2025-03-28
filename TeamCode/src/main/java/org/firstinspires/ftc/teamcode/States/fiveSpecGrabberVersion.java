package org.firstinspires.ftc.teamcode.States;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;;

@Autonomous(name = "Get_Shitt_OnV3", group = "Competition")
public class fiveSpecGrabberVersion extends OpMode {
    private int x = 0;
    private int pathState = 0;
    private int highChamberCount = 0;
    private boolean elevatorUp = false;
    private boolean lift = false;

    private TouchSensor touch;
    private Follower follower;
    private Timer opmodeTimer;
    private ClawSubsystem clawSubsystem;
    AnalogInput ranger;
    private Timer yAxisTimer;
    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Pose placeSpecimenPose = new Pose(30, 12, 0);

    // Change push Ready Path as a curve
    //        pushReadyPath , this is for the 1st speciman push with a curve path


    // afterSpecimanPose is the end of the curve
    private Pose placeSpecimenPose1 = new Pose(19, 0, 0);
    private Pose placeSpecimenPose2 = new Pose(20, 2, 0);
    private Pose placeSpecimenPose3 = new Pose(29.5, 14.5, 0);
    private Pose placeSpecimenPose4 = new Pose(29.5, 14, 0);
    //    private final Pose specimenReayControlPose = new Pose(5, -25, 0);
//    private final Pose afterSpecimenPose = new Pose(52, -18, Math.toRadians(180));


    private final Pose afterSpecimenPose = new Pose(12, -10, Math.toRadians(0));

    //private final Pose specimen1ControlPose = new Pose(60, -26, Math.toRadians(0));
    private final Pose specimen1PushReadyPose = new Pose(8.5, -10, Math.toRadians(0));
    private final Pose specimen1Pose = new Pose(42, -13, Math.toRadians(0));
    private final Pose specimen1StrafePose = new Pose(55, -15, Math.toRadians(0));
    private final Pose specimen1PushPose = new Pose(25, -15, Math.toRadians(0));
    private final Pose specimen2Pose = new Pose(44, -10.5, Math.toRadians(0));
    private final Pose specimen2StrafePose = new Pose(53, -14, Math.toRadians(0));
    private final Pose specimen2PushPose = new Pose(25, -14, Math.toRadians(0));


    private final Pose specimen3Pose = new Pose(45, -14.5, Math.toRadians(0));
    private final Pose specimen3StrafePose = new Pose(53, -20, Math.toRadians(0));
    private final Pose specimen3PushPose = new Pose(25, -20, Math.toRadians(0));
    private final Pose readyForHuman1 = new Pose(3, -26, Math.toRadians(0));
    private final Pose humanPlayerZonePose1 = new Pose(0, -25, Math.toRadians(0));
    private final Pose humanPlayerZonebackPose1 = new Pose(0, -40, Math.toRadians(0));
    private final Pose readyForHuman = new Pose(4, -23.8, Math.toRadians(0));
    private final Pose humanPlayerZonePose = new Pose(0, -21.8, Math.toRadians(0));
    private final Pose humanPlayerZonebackPose = new Pose(0, -21.8, Math.toRadians(0));
    private final Pose specHang1Pose = new Pose(0, 12, Math.toRadians(0));
    private final Pose highChamberPose = new Pose(0, 14, 0);
    private final Pose returnHomePose = new Pose(25, 15, 0);
    // Paths
    private Path hang1Path, specimenToHighChamberPath1, specimenToHighChamberPath2, specimenToHighChamberPath3, specimenToHighChamberPath4, placeSpecimenPath, pushReadyPath, specimen1ReadyPushPath,specimen1Path, specimen1StrafePath,
            specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath,specimen3Path, specimen3StrafePath, specimen3PushPath,
            readyForHumanPath1,readyForHumanPath, getSpeciPath, getSpeciBackPath, specimenToHighChamberPath, returnHomePath, readyForHumanPath2,getSpeciPath1, getSpeciBackPath1;
    private PathChain pushPath, pushSpecimen1Path;

    @Override
    public void init() {
        clawSubsystem = new ClawSubsystem(hardwareMap);
        follower = new Follower(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower.setStartingPose(startPose);
        //clawSubsystem.height = 0;
        buildPaths();
        clawSubsystem.moveDownSp();
        yAxisTimer = new Timer();
        opmodeTimer.resetTimer();
        yAxisTimer.resetTimer();
        //touch = hardwareMap.get(DigitalChannel.class, "touch");
        //touch.setMode(DigitalChannel.Mode.INPUT);
        ranger = hardwareMap.get(AnalogInput.class, "ranger");
        touch = hardwareMap.get(TouchSensor.class, "touch");



    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        if (clawSubsystem.height != 0) {
            clawSubsystem.moveDownSp();
        }

    }

    @Override




    public void loop() {

        // Update the Y coordinate only every 3 seconds.
        if ((ranger.getVoltage() * 48.7 - 4.9) > 7 &&
                ((ranger.getVoltage() * 48.7 - 4.9) < 25) &&
                (follower.getPose().getHeading() == 0)) {

            if (yAxisTimer.getElapsedTimeSeconds() >= 3.0) {
                Pose currentPose = follower.getPose();
                // Calculate the new Y coordinate using the range sensor value.
                Pose newPose = new Pose(
                        currentPose.getX(),
                        (((ranger.getVoltage() * 48.7 - 4.9) - 35.7) + currentPose.getY()) / 2,
                        currentPose.getHeading());
                follower.setStartingPose(newPose);
                telemetry.addData("Reset Y", "Y coordinate set to: " + (ranger.getVoltage() * 48.7 - 4.9));
                yAxisTimer.resetTimer();
            }
        }
      if (touch.isPressed()) {
            Pose currentPose = follower.getPose();
            Pose newPose = new Pose(0, currentPose.getY(), currentPose.getHeading());
            follower.setStartingPose(newPose);
            telemetry.addData("Touch Override", "Reset X coordinate to 0");
        }
/*
        if (!touch.getState()) {
            // Retrieve the current pose
            Pose currentPose = follower.getPose();
            // Create a new Pose with the x coordinate reset to zero,
            // while preserving the y coordinate and heading.
            Pose newPose = new Pose(0, currentPose.getY(), currentPose.getHeading());
            // Update the Follower's pose (this method should update the internal odometry coordinate)
            follower.setStartingPose(newPose);
            telemetry.addData("Reset X", "PedroPathing X coordinate set to 0");
        }

 */

        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        Telemetry.Item elapsedTime = telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());

        telemetry.addData("elevator Height", clawSubsystem.height);
        telemetry.addData("elevator Current Positon", clawSubsystem.getElevatorPosition());
        telemetry.addData("isNearPose(follower.getPose(), placeSpecimenPose, 1)", isNearPose(follower.getPose(), placeSpecimenPose, 1));
        telemetry.update();
    }

    private void buildPaths() {

        placeSpecimenPath = new Path(new BezierLine(new Point(startPose), new Point(placeSpecimenPose)));
        placeSpecimenPath.setLinearHeadingInterpolation(startPose.getHeading(), placeSpecimenPose.getHeading());

        pushReadyPath = new Path(new BezierLine(new Point(placeSpecimenPose1), new Point(afterSpecimenPose)));
        pushReadyPath.setLinearHeadingInterpolation(placeSpecimenPose1.getHeading(), afterSpecimenPose.getHeading());


// Change push Ready Path as a curve

//        pushReadyPath = new Path(new BezierCurve(new Point(placeSpecimenPose), new Point(specimenReayControlPose), new Point(afterSpecimenPose)));
//        pushReadyPath.setLinearHeadingInterpolation(placeSpecimenPose.getHeading(), afterSpecimenPose.getHeading());


       // specimen1ReadyPushPath = new Path(new BezierCurve(new Point(afterSpecimenPose), new Point(specimen1ControlPose), new Point(specimen1PushReadyPose)));
       // specimen1ReadyPushPath.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1PushReadyPose.getHeading());


//        specimen1Path = new Path(new BezierLine(new Point(afterSpecimenPose), new Point(specimen1Pose)));
//        specimen1Path.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1Pose.getHeading());

        specimen1Path = new Path(new BezierLine(new Point(afterSpecimenPose), new Point(specimen1Pose)));
        specimen1Path.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1Pose.getHeading());

        hang1Path = new Path(new BezierLine(new Point(placeSpecimenPose), new Point(specHang1Pose)));
        hang1Path.setLinearHeadingInterpolation(placeSpecimenPose.getHeading(), specHang1Pose.getHeading());


        specimen1StrafePath = new Path(new BezierLine(new Point(specimen1Pose), new Point(specimen1StrafePose)));
        specimen1StrafePath.setLinearHeadingInterpolation(specimen1Pose.getHeading(), specimen1StrafePose.getHeading());

        specimen1PushPath = new Path(new BezierLine(new Point(specimen1StrafePose), new Point(specimen1PushPose)));
        specimen1PushPath.setLinearHeadingInterpolation(specimen1StrafePose.getHeading(), specimen1PushPose.getHeading());

        specimen2Path = new Path(new BezierLine(new Point(specimen1PushPose), new Point(specimen2Pose)));
        specimen2Path.setLinearHeadingInterpolation(specimen1PushPose.getHeading(), specimen2Pose.getHeading());

        specimen2StrafePath = new Path(new BezierLine(new Point(specimen2Pose), new Point(specimen2StrafePose)));
        specimen2StrafePath.setLinearHeadingInterpolation(specimen2Pose.getHeading(), specimen2StrafePose.getHeading());

        specimen2PushPath = new Path(new BezierLine(new Point(specimen2StrafePose), new Point(specimen2PushPose)));
        specimen2PushPath.setLinearHeadingInterpolation(specimen2StrafePose.getHeading(), specimen2PushPose.getHeading());

        specimen3Path = new Path(new BezierLine(new Point(specimen2PushPose), new Point(specimen3Pose)));
        specimen3Path.setLinearHeadingInterpolation(specimen2PushPose.getHeading(), specimen3Pose.getHeading());

        specimen3StrafePath = new Path(new BezierLine(new Point(specimen3Pose), new Point(specimen3StrafePose)));
        specimen3StrafePath.setLinearHeadingInterpolation(specimen3Pose.getHeading(), specimen3StrafePose.getHeading());

        specimen3PushPath = new Path(new BezierLine(new Point(specimen3StrafePose), new Point(specimen3PushPose)));
        specimen3PushPath.setLinearHeadingInterpolation(specimen3StrafePose.getHeading(), specimen3PushPose.getHeading());

        readyForHumanPath = new Path(new BezierLine(new Point(placeSpecimenPose1), new Point(readyForHuman)));
        readyForHumanPath.setLinearHeadingInterpolation(placeSpecimenPose1.getHeading(), readyForHuman.getHeading());


        readyForHumanPath1 = new Path(new BezierLine(new Point(specimen3PushPose), new Point(readyForHuman)));
        readyForHumanPath1.setLinearHeadingInterpolation(specimen3PushPose.getHeading(), readyForHuman.getHeading());

        getSpeciPath = new Path(new BezierLine(new Point(readyForHuman), new Point(humanPlayerZonePose)));
        getSpeciPath.setLinearHeadingInterpolation(readyForHuman.getHeading(), humanPlayerZonePose.getHeading());

        getSpeciBackPath = new Path(new BezierLine(new Point(humanPlayerZonePose), new Point(humanPlayerZonebackPose)));
        getSpeciBackPath.setLinearHeadingInterpolation(humanPlayerZonePose.getHeading(), humanPlayerZonebackPose.getHeading());


        readyForHumanPath2 = new Path(new BezierLine(new Point(specimen3PushPose), new Point(readyForHuman1)));
        readyForHumanPath2.setLinearHeadingInterpolation(specimen3PushPose.getHeading(), readyForHuman1.getHeading());

        getSpeciPath1 = new Path(new BezierLine(new Point(readyForHuman1), new Point(humanPlayerZonePose1)));
        getSpeciPath1.setLinearHeadingInterpolation(readyForHuman1.getHeading(), humanPlayerZonePose1.getHeading());

        getSpeciBackPath1 = new Path(new BezierLine(new Point(humanPlayerZonePose1), new Point(humanPlayerZonebackPose1)));
        getSpeciBackPath1.setLinearHeadingInterpolation(humanPlayerZonePose1.getHeading(), humanPlayerZonebackPose1.getHeading());


        specimenToHighChamberPath1 = new Path(new BezierLine(new Point(startPose), new Point(placeSpecimenPose1)));
        specimenToHighChamberPath1.setLinearHeadingInterpolation(startPose.getHeading(), highChamberPose.getHeading());

        specimenToHighChamberPath2 = new Path(new BezierLine(new Point(humanPlayerZonebackPose1), new Point(placeSpecimenPose2)));
        specimenToHighChamberPath2.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose2.getHeading());

        specimenToHighChamberPath3 = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose3)));
        specimenToHighChamberPath3.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose3.getHeading());

        specimenToHighChamberPath4 = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose4)));
        specimenToHighChamberPath4.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose4.getHeading());

        returnHomePath = new Path(new BezierLine(new Point(highChamberPose), new Point(returnHomePose)));
        returnHomePath.setLinearHeadingInterpolation(highChamberPose.getHeading(), returnHomePose.getHeading());

        //working to before first push part
        //pushPath = new PathChain(specimen1Path, specimen1StrafePath);

       // pushPath = new PathChain(specimen1Path);
        pushPath = new PathChain(specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath, specimen3Path, specimen3StrafePath, specimen3PushPath, readyForHumanPath1);
        //pushSpecimen1Path = new PathChain(specimen1ReadyPushPath, specimen1Path);
        //ww    pushSpecimen1Path = new PathChain(specimen1ReadyPushPath);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial state

                if (lift){
                    follower.followPath(specimenToHighChamberPath1, true);
                    opmodeTimer.resetTimer();
                    setPathState(1);
                  }
                else {
                    clawSubsystem.grabSpec();
                    clawSubsystem.retract();

               //     clawSubsystem.specReadyForHang();
                    if (opmodeTimer.getElapsedTimeSeconds() > 0.1) {
                        clawSubsystem.specReadyForHang();
                        if (opmodeTimer.getElapsedTimeSeconds() > .5) {

                            lift = true;
                        }
                    }
                }




                break;
            case 1: // Place specimen
                if (isNearPose(follower.getPose(), placeSpecimenPose1, 1)) {









                            clawSubsystem.specHang();
                            opmodeTimer.resetTimer();

                            opmodeTimer.resetTimer();
                            setPathState(2);

                                // elevatorUp = false;
                                // opmodeTimer.resetTimer();
                                // setPathState(-1);


                }
/*
                if (clawSubsystem.height == 300) {
                    // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(pushReadyPath, true);

                    // Move to a point and turn direction
                    setPathState(2);
                }

 */
                break;

            case 2: // Push First sample, and stop
                    if (opmodeTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(pushReadyPath, true);

                        setPathState(3);

                    }
                    break;

            case 3:

                    if (opmodeTimer.getElapsedTimeSeconds()>.8) {
                        clawSubsystem.releaseSpec();
                        //setPathState(3);
                    }

                    if (isNearPose(follower.getPose(), afterSpecimenPose, 3)) {
                    // clawSubsystem.moveDownSp();
                   // clawSubsystem.moveToPickingReady();
                    //clawSubsystem.extend();

                        clawSubsystem.specReadyForGrab();
                        follower.followPath(pushPath, true);
                        opmodeTimer.resetTimer();
                        lift = false;
                        setPathState(4);


                        //opmodeTimer.resetTimer();
                        //setPathState(-1);
                    }
                break;

//  here



            case 4: // Ready for next phase
                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    // clawSubsystem.moveDownSp();

                    if (lift){
                        follower.followPath(specimenToHighChamberPath2, true);
                        opmodeTimer.resetTimer();
                        setPathState(6);
                    }
                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                            clawSubsystem.grabSpec();
                            clawSubsystem.retract();

                            //     clawSubsystem.specReadyForHang();
                        if (opmodeTimer.getElapsedTimeSeconds() > 1.1) {
                            clawSubsystem.specReadyForHang();
                            if (opmodeTimer.getElapsedTimeSeconds() > 1.5) {

                                lift = true;
                            }
                        }
                        }
                    }// follower.followPath(specimen2Path);
                }


                break;

            // Stop at the back position

            case 6:
                if (isNearPose(follower.getPose(), placeSpecimenPose2, 1)) {
                    // Ready for human interaction



                    opmodeTimer.resetTimer();
                    if (opmodeTimer.getElapsedTimeSeconds()>1) {
                        setPathState(7);

                    }
                    else{
                        clawSubsystem.specHang();
                    }
                }
                break;
            case 7: // Push First sample, and stop
                if (opmodeTimer.getElapsedTimeSeconds()>0.5) {
                    follower.followPath(readyForHumanPath, true);

                    setPathState(8);

                }
                break;

            case 8:

                if (opmodeTimer.getElapsedTimeSeconds()>.8) {
                    clawSubsystem.releaseSpec();
                    clawSubsystem.specReadyForGrab();
                    //setPathState(3);
                }

                if (isNearPose(follower.getPose(), readyForHuman1, 1)) {
                    clawSubsystem.grab();
                    setPathState(-1);
                }

            case 9: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 0) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath1, true);
                        x += 3;
                        clawSubsystem.moveUpSp();
                        follower.followPath(specimenToHighChamberPath2, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 0.6) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -500) {
                                telemetry.addData("Status", "Elevator moving up");
                                clawSubsystem.moveUpSp();
                            } else {
                                telemetry.addData("Status", "Following back path");
                                follower.followPath(getSpeciBackPath, true);

                                //follower.followPath(getSpeciBackPath, true);
                                x += 3;
//                            placeSpecimenPose1 = new Pose(32, 23 - x, 0);
                                specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                                specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                                follower.followPath(specimenToHighChamberPath2, true);
                                opmodeTimer.resetTimer();
                                setPathState(13);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 13: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose2, 1.3)) {
                    if (opmodeTimer.getElapsedTimeSeconds()>0) {
                        clawSubsystem.moveDownSpToRelease();
                    }


                    // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                    if (opmodeTimer.getElapsedTimeSeconds()>3) {
                        //clawSubsystem.moveToPickingReady();

                        follower.followPath(readyForHumanPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(14);


                    }

                }
                break;

            case 14: // Ready for human interaction


                if (isNearPose(follower.getPose(), readyForHuman, 20)){
                    clawSubsystem.moveToPickingReady();
                }

                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    clawSubsystem.moveToPickingReady();

                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(16);
                }
                break;
            case 16: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 0 ) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath, true);
                        x += 3;
                        clawSubsystem.moveUpSp();
                        follower.followPath(specimenToHighChamberPath3, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 0.3) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -400) {
                                telemetry.addData("Status", "Elevator moving up");
                                clawSubsystem.moveUpSp();
                            } else {
                                telemetry.addData("Status", "Following back path");
                                follower.followPath(getSpeciBackPath, true);

                                //follower.followPath(getSpeciBackPath, true);
                                x += 3;
//                            placeSpecimenPose1 = new Pose(32, 23 - x, 0);
                                specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                                specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                                follower.followPath(specimenToHighChamberPath3, true);
                                opmodeTimer.resetTimer();
                                setPathState(18);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 18: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose3, 1.3)) {
                    clawSubsystem.moveDownSpToRelease();
                    if (opmodeTimer.getElapsedTimeSeconds()>3){

                        // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                        //clawSubsystem.moveToPickingReady();

                        follower.followPath(readyForHumanPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(20);


                    }

                }






                break;
            case 20:
                if (isNearPose(follower.getPose(), readyForHuman, 20)){
                    clawSubsystem.moveToPickingReady();
                }

                // Ready for human interaction
                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    //    clawSubsystem.moveToPickingReady();
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(21);
                }
                break;
            case 21: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 0.8) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath, true);
                        x += 3;
                        clawSubsystem.moveUpSp();
                        follower.followPath(specimenToHighChamberPath4, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 0.2) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -400) {
                                telemetry.addData("Status", "Elevator moving up");
                                clawSubsystem.moveUpSp();
                            } else {
                                telemetry.addData("Status", "Following back path");
                                follower.followPath(getSpeciBackPath, true);

                                //follower.followPath(getSpeciBackPath, true);
                                x += 3;
//                            placeSpecimenPose1 = new Pose(32, 23 - x, 0);
                                specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                                specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                                follower.followPath(specimenToHighChamberPath4, true);
                                opmodeTimer.resetTimer();
                                setPathState(22);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 22: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose4, 1.3)) {
                    clawSubsystem.moveDownSpToRelease();


                    //clawSubsystem.extend();
                    //clawSubsystem.grabReady();



                    opmodeTimer.resetTimer();
                    setPathState(23);







                }






                break;




            case 23:
                if (opmodeTimer.getElapsedTimeSeconds()>0.5) {
                    follower.followPath(returnHomePath, true);

                /*
                if (clawSubsystem.getElevatorPosition() >= -100) {


                    follower.followPath(returnHomePath, true);
                    //follower.followPath(returnHomePath, true);
                    clawSubsystem.moveDown();
                }
                else{
                    follower.followPath(returnHomePath, true);
                }
            }

                */
                    setPathState(25); // End
                }

                break;
            case 25:
                if (isNearPose(follower.getPose(), returnHomePose, 3)){
                    clawSubsystem.grabVertical();
                    clawSubsystem.moveDown();
                    //setPathState(-1);

                }

            case -1: // Autonomous complete
                telemetry.addData("Status", "Autonomous Complete");
                break;

        }
    }

    private boolean isNearPose(Pose current, Pose target,    double tolerance) {
        return Math.abs(current.getX() - target.getX()) < tolerance &&
                Math.abs(current.getY() - target.getY()) < tolerance;
    }

    private void setPathState(int state) {
        pathState = state;
    }
}

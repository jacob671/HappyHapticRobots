package org.firstinspires.ftc.teamcode.States;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Autonomous(name = "Get_Shitt_OnV3", group = "Competition")
public class fiveSpecGrabberVersion extends OpMode {
    private int x = 0;
    private int pathState = 0;
    private int highChamberCount = 0;
    private boolean elevatorUp = false;
    private boolean lift = false;

    private Follower follower;
    private Timer opmodeTimer;
    private ClawSubsystem clawSubsystem;

    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Pose placeSpecimenPose = new Pose(30, 12, 0);

    // Change push Ready Path as a curve
    //        pushReadyPath , this is for the 1st speciman push with a curve path


    // afterSpecimanPose is the end of the curve
    private Pose placeSpecimenPose1 = new Pose(29, 0, 0);
    private Pose placeSpecimenPose2 = new Pose(30.4, 14, 0);
    private Pose placeSpecimenPose3 = new Pose(30.4, 14.5, 0);
    private Pose placeSpecimenPose4 = new Pose(30.4, 15, 0);
    private Pose placeSpecimenPose5 = new Pose(30.4, 15.5, 0);

    //    private final Pose specimenReayControlPose = new Pose(5, -25, 0);
//    private final Pose afterSpecimenPose = new Pose(52, -18, Math.toRadians(180));


    private final Pose afterSpecimenPose = new Pose(10, -20, Math.toRadians(0));

    //private final Pose specimen1ControlPose = new Pose(60, -26, Math.toRadians(0));
    private final Pose specimen1PushReadyPose = new Pose(8.5, -14.2, Math.toRadians(0));
    private final Pose specimen1Pose = new Pose(40, -22, Math.toRadians(0));
    private final Pose specimen1StrafePose = new Pose(52, -29, Math.toRadians(0));
    private final Pose specimen1PushPose = new Pose(15, -32, Math.toRadians(0));
    private final Pose specimen2Pose = new Pose(41, -30, Math.toRadians(0));
    private final Pose specimen2StrafePose = new Pose(52, -37, Math.toRadians(0));
    private final Pose specimen2PushPose = new Pose(15, -37, Math.toRadians(0));


    private final Pose specimen3Pose = new Pose(41, -37, Math.toRadians(0));
    private final Pose specimen3StrafePose = new Pose(52, -42, Math.toRadians(0));
    private final Pose specimen3PushPose = new Pose(15, -42, Math.toRadians(0));
    private final Pose readyForHuman1 = new Pose(3, -25, Math.toRadians(0));
    private final Pose humanPlayerZonePose1 = new Pose(0, -48, Math.toRadians(0));
    private final Pose humanPlayerZonebackPose1 = new Pose(0, -40, Math.toRadians(0));
    private final Pose readyForHuman = new Pose(3, -30, Math.toRadians(0));
    private final Pose humanPlayerZonePose = new Pose(0, -21.8, Math.toRadians(0));
    private final Pose humanPlayerZonebackPose = new Pose(0, -21.8, Math.toRadians(0));
    private final Pose specHang1Pose = new Pose(0, 12, Math.toRadians(0));
    private final Pose highChamberPose = new Pose(0, 14, 0);
    private final Pose returnHomePose = new Pose(25, 15, 0);
    // Paths
    private Path hang1Path, specimenToHighChamberPath1, specimenToHighChamberPath2, specimenToHighChamberPath3, specimenToHighChamberPath4,specimenToHighChamberPath5, placeSpecimenPath, pushReadyPath, specimen1ReadyPushPath,specimen1Path, specimen1StrafePath,
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

        readyForHumanPath = new Path(new BezierLine(new Point(placeSpecimenPose3), new Point(readyForHuman1)));
        readyForHumanPath.setLinearHeadingInterpolation(placeSpecimenPose3.getHeading(), readyForHuman1.getHeading());


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
        specimenToHighChamberPath1.setLinearHeadingInterpolation(startPose.getHeading(), placeSpecimenPose.getHeading());

        specimenToHighChamberPath2 = new Path(new BezierLine(new Point(humanPlayerZonebackPose1), new Point(placeSpecimenPose2)));
        specimenToHighChamberPath2.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), highChamberPose.getHeading());

        specimenToHighChamberPath3 = new Path(new BezierLine(new Point(readyForHuman1), new Point(placeSpecimenPose3)));
        specimenToHighChamberPath3.setLinearHeadingInterpolation(readyForHuman1.getHeading(), highChamberPose.getHeading());

        specimenToHighChamberPath4 = new Path(new BezierLine(new Point(readyForHuman1), new Point(placeSpecimenPose4)));
        specimenToHighChamberPath4.setLinearHeadingInterpolation(readyForHuman1.getHeading(), highChamberPose.getHeading());

        specimenToHighChamberPath5 = new Path(new BezierLine(new Point(readyForHuman1), new Point(placeSpecimenPose5)));
        specimenToHighChamberPath5.setLinearHeadingInterpolation(readyForHuman1.getHeading(), highChamberPose.getHeading());

        returnHomePath = new Path(new BezierLine(new Point(readyForHuman1), new Point(returnHomePose)));
        returnHomePath.setLinearHeadingInterpolation(readyForHuman1.getHeading(), returnHomePose.getHeading());

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
                        if (opmodeTimer.getElapsedTimeSeconds() > 0.2) {

                            lift = true;
                        }
                    }
                }




                break;
            case 1: // Place specimen
                if (isNearPose(follower.getPose(), placeSpecimenPose1, 1.5)) {









                            clawSubsystem.specHang();
                            opmodeTimer.resetTimer();
                            follower.followPath(pushReadyPath, true);
                            opmodeTimer.resetTimer();
                            lift = false;
                            setPathState(3);

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



            case 3:
                    if (opmodeTimer.getElapsedTimeSeconds()>0.4) {
                        clawSubsystem.releaseSpec();

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
                        //setPathState(4);
                    }
                break;

// Stop here
            case 4:
                if (isNearPose(follower.getPose(), specimen3Pose, 2.8)) {
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }



            case 5: // Ready for next phase
                if (isNearPose(follower.getPose(), readyForHuman, 2.6)) {
                    // clawSubsystem.moveDownSp();

                    if (lift){
                        follower.followPath(specimenToHighChamberPath2, true);
                        opmodeTimer.resetTimer();
                        setPathState(6);
                    }
                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
                            clawSubsystem.grabSpec();
                            clawSubsystem.retract();
                        }
                        //     clawSubsystem.specReadyForHang();
                        if (opmodeTimer.getElapsedTimeSeconds() > 4.3) {
                            clawSubsystem.specReadyForHang();
                            if (opmodeTimer.getElapsedTimeSeconds() > 4.4) {

                                lift = true;
                            }
                        }

                    }// follower.followPath(specimen2Path);
                }


                break;

            // Stop at the back position

            case 6:
                if (isNearPose(follower.getPose(), placeSpecimenPose2, 2)) {
                    clawSubsystem.specHang();
                    opmodeTimer.resetTimer();
                    follower.followPath(readyForHumanPath, true);
                    opmodeTimer.resetTimer();
                    lift = false;



                    setPathState(7);



                }
                break;
            case 7:
                if (opmodeTimer.getElapsedTimeSeconds()>0.7){
                    clawSubsystem.releaseSpec();
                }
                if (isNearPose(follower.getPose(), readyForHuman1, 20)) {

                    clawSubsystem.specReadyForGrab();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(8);
                }
                break;
            case 8:
                if (isNearPose(follower.getPose(), readyForHuman1, 4)) {
                    // clawSubsystem.moveDownSp();

                    if (lift){
                        follower.followPath(specimenToHighChamberPath3, true);
                        opmodeTimer.resetTimer();
                        setPathState(9);
                    }
                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 3.2) {
                            clawSubsystem.grabSpec();
                            clawSubsystem.retract();
                        }
                        //     clawSubsystem.specReadyForHang();
                        if (opmodeTimer.getElapsedTimeSeconds() > 3.4) {
                            clawSubsystem.specReadyForHang();
                            if (opmodeTimer.getElapsedTimeSeconds() > 3.9) {

                                lift = true;
                            }
                        }

                    }// follower.followPath(specimen2Path);
                }


                break;
            case 9:
                if (isNearPose(follower.getPose(), placeSpecimenPose3, 2)) {
                    clawSubsystem.specHang();
                    opmodeTimer.resetTimer();
                    follower.followPath(readyForHumanPath, true);
                    opmodeTimer.resetTimer();
                    lift = false;



                    setPathState(10);







                }
                break;
            case 10:
                if (opmodeTimer.getElapsedTimeSeconds()>0.7){
                    clawSubsystem.releaseSpec();
                }
                if (isNearPose(follower.getPose(), readyForHuman1, 20)) {

                    clawSubsystem.specReadyForGrab();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(11);
                }
                break;
            case 11:
                if (isNearPose(follower.getPose(), readyForHuman1, 4)) {
                    // clawSubsystem.moveDownSp();

                    if (lift){
                        follower.followPath(specimenToHighChamberPath4, true);
                        opmodeTimer.resetTimer();
                        setPathState(12);
                    }
                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 3) {
                            clawSubsystem.grabSpec();
                            clawSubsystem.retract();
                        }
                        //     clawSubsystem.specReadyForHang();
                        if (opmodeTimer.getElapsedTimeSeconds() > 3.1) {
                            clawSubsystem.specReadyForHang();
                            if (opmodeTimer.getElapsedTimeSeconds() > 3.9) {

                                lift = true;
                            }
                        }

                    }// follower.followPath(specimen2Path);
                }


                break;
            case 12:
                if (isNearPose(follower.getPose(), placeSpecimenPose4, 2)) {
                    clawSubsystem.specHang();
                    opmodeTimer.resetTimer();
                    follower.followPath(readyForHumanPath, true);
                    opmodeTimer.resetTimer();
                    lift = false;



                    setPathState(13);







                }
                break;
            case 13:
                if (opmodeTimer.getElapsedTimeSeconds()>0.7){
                    clawSubsystem.releaseSpec();
                }
                if (isNearPose(follower.getPose(), readyForHuman1, 20)) {

                    clawSubsystem.specReadyForGrab();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(14);
                }
                break;
            case 14:
                if (isNearPose(follower.getPose(), readyForHuman1, 4)) {
                    // clawSubsystem.moveDownSp();

                    if (lift){
                        follower.followPath(specimenToHighChamberPath5, true);
                        opmodeTimer.resetTimer();
                        setPathState(15);
                    }
                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 3) {
                            clawSubsystem.grabSpec();
                            clawSubsystem.retract();
                        }
                        //     clawSubsystem.specReadyForHang();
                        if (opmodeTimer.getElapsedTimeSeconds() > 3.1) {
                            clawSubsystem.specReadyForHang();
                            if (opmodeTimer.getElapsedTimeSeconds() > 3.9) {

                                lift = true;
                            }
                        }

                    }// follower.followPath(specimen2Path);
                }


                break;
            case 15:
                if (isNearPose(follower.getPose(), placeSpecimenPose5, 2)) {
                    clawSubsystem.specHang();
                    opmodeTimer.resetTimer();
                   follower.followPath(readyForHumanPath, true);
                    opmodeTimer.resetTimer();
                    lift = false;



                    setPathState(16);







                }
                break;
            case 16:
                if (opmodeTimer.getElapsedTimeSeconds()>1){
                    clawSubsystem.releaseSpec();
                }

                if (isNearPose(follower.getPose(), readyForHuman1, 20)) {

                    clawSubsystem.specReadyForGrab();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(-1);
                }
                break;



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

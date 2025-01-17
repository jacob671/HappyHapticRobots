// FTC Autonomous Code: autoRightSide
// This code handles robot actions on the right side of the gameboard using PedroPathi
//1/1 11:49 specimen works


// FTC Autonomous Code: autoRightSide
// This code handles robot actions on the right side of the gameboard using PedroPathi
/*
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.jvm.Code;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;


@Autonomous(name = "AutoR Side", group = "Competition")
public class AutoRightSide extends OpMode {

    private Follower follower;
    private int x = 0;
    private Timer opmodeTimer;
    private int pathState = 0;
    private int highChamberCount = 0;
    private boolean elevatorUp = false;
    private ClawSubsystem clawSubsystem;
    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Pose placeSpecimenPose = new Pose(29, 12+x, 0);
    private final Pose afterSpecimenPose = new Pose(12,-17.4 , 0);
    private final Pose specimen1Pose = new Pose(50, -19.4, Math.toRadians(0));
    private final Pose specimen1StrafePose = new Pose(50, -24, Math.toRadians(0));
    private final Pose specimen1PushPose = new Pose(18, -28, Math.toRadians(0));
    private final Pose specimen2Pose = new Pose(50, -28, Math.toRadians(0));
    private final Pose specimen2StrafePose = new Pose(50, -38, Math.toRadians(0));

    private final Pose specimen2PushPose = new Pose(18, -38, Math.toRadians(0));
    private final Pose specimen3Pose = new Pose(50, -48, Math.toRadians(0));
    private final Pose specimen3StrafePose = new Pose(50, -48, Math.toRadians(0));
    private final Pose specimen3PushPose = new Pose(20, -48, Math.toRadians(0));
    private final Pose readyForHuman = new Pose(10, -20.4, Math.toRadians(180));
    private final Pose humanPlayerZonePose = new Pose(-6, -20.4, Math.toRadians(180));
    private final Pose humanPlayerZonebackPose = new Pose(3, -20.4, Math.toRadians(180));

    private final Pose highChamberPose = new Pose(28, 14+x, Math.toRadians(0));
    private final Pose returnHomePose = new Pose(0, 0, Math.toRadians(0));

    // Paths
    private Path placeSpecimenPath, specimen1Path, specimen1StrafePath, specimen1PushPath,
            specimen2Path, specimen2StrafePath, specimen2PushPath,pushReadyPath,
            specimen3Path, specimen3StrafePath, specimen3PushPath,
            humanPlayerZonePath, specimenToHighChamberPath, returnHomePath, returnToPlay,readyForHumanPath,readyForHumanPath1, getSpeciPath, getSpeciBackPath;
    private PathChain PushPath;

    @Override
    public void init() {
        clawSubsystem = new ClawSubsystem(hardwareMap);

        follower = new Follower(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    private void buildPaths() {
        placeSpecimenPath = new Path(new BezierLine(new Point(startPose), new Point(placeSpecimenPose)));
        placeSpecimenPath.setLinearHeadingInterpolation(startPose.getHeading(), placeSpecimenPose.getHeading());

        pushReadyPath = new Path(new BezierLine(new Point(placeSpecimenPose), new Point(afterSpecimenPose)));
        pushReadyPath.setLinearHeadingInterpolation(placeSpecimenPose.getHeading(), afterSpecimenPose.getHeading());


        specimen1Path = new Path(new BezierLine(new Point(afterSpecimenPose), new Point(specimen1Pose)));
        specimen1Path.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1Pose.getHeading());

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

        humanPlayerZonePath = new Path(new BezierLine(new Point(specimen3PushPose), new Point(humanPlayerZonePose)));
        humanPlayerZonePath.setLinearHeadingInterpolation(specimen3PushPose.getHeading(), humanPlayerZonePose.getHeading());

        specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose)));
        specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose.getHeading());

        returnToPlay = new Path(new BezierLine(new Point(highChamberPose), new Point(humanPlayerZonePose)));
        returnToPlay.setLinearHeadingInterpolation(highChamberPose.getHeading(), humanPlayerZonePose.getHeading());
        readyForHumanPath1 = new Path(new BezierLine(new Point(specimen2PushPose), new Point(readyForHuman)));
        readyForHumanPath1.setLinearHeadingInterpolation(specimen2PushPose.getHeading(), readyForHuman.getHeading());

        readyForHumanPath = new Path(new BezierLine(new Point(highChamberPose), new Point(readyForHuman)));
        readyForHumanPath.setLinearHeadingInterpolation(highChamberPose.getHeading(), readyForHuman.getHeading());


        getSpeciPath = new Path(new BezierLine(new Point(readyForHuman), new Point(humanPlayerZonePose)));
        getSpeciPath.setLinearHeadingInterpolation(readyForHuman.getHeading(), humanPlayerZonePose.getHeading());
        getSpeciBackPath = new Path(new BezierLine(new Point(humanPlayerZonePose), new Point(humanPlayerZonebackPose)));
        getSpeciBackPath.setLinearHeadingInterpolation(humanPlayerZonePose.getHeading(), humanPlayerZonebackPose.getHeading());

        returnHomePath = new Path(new BezierLine(new Point(highChamberPose), new Point(returnHomePose)));

        returnHomePath.setLinearHeadingInterpolation(highChamberPose.getHeading(), returnHomePose.getHeading());
        PushPath = new PathChain(specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath, readyForHumanPath1);

    }


    private void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                // Check if follower's position is greater than the threshold (startPose - 1) for X and Y
                if (follower.getPose().getX() > (startPose.getX() - 1) && follower.getPose().getY() > (startPose.getY() - 1)) {
                    follower.followPath(placeSpecimenPath, true);

                    if (!elevatorUp) {
                        clawSubsystem.moveUpSp();
                        clawSubsystem.extendOriginal();// Move elevator up
                        elevatorUp = true;
                        opmodeTimer.resetTimer();
                    } else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 0) {
                            setPathState(1);// Move to the next path after completing the current one
                        }
                    }
                }

                break;

            case 1:
                // Check position before following the next path
                if (follower.getPose().getX() > (placeSpecimenPose.getX() - 1) && follower.getPose().getY() > (placeSpecimenPose.getY() - 1)) {
                    if (elevatorUp) {
                        //clawSubsystem.reset();
                        opmodeTimer.resetTimer();
                        clawSubsystem.moveDownBit(); // Move elevator up
                        elevatorUp = false;
                    }
                    if(!elevatorUp){

                        if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                            clawSubsystem.moveUpBit();
                            follower.followPath(pushReadyPath, true);

                            opmodeTimer.resetTimer();
                            setPathState(3);

                        }
                    }


                }
                break;








            case 3:
                // Check position before following the next path

                if (follower.getPose().getX() < (afterSpecimenPose.getX() + 1) && follower.getPose().getY() < (afterSpecimenPose.getY() + 1)) {
                    clawSubsystem.moveDownSp();
                    //follower.followPath(specimen1StrafePath, true);



                        follower.followPath(PushPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(5);



                }
                break;






            case 5:

                if (follower.getPose().getX() < (readyForHuman.getX() + 1) && follower.getPose().getY() > (readyForHuman.getY() - 1)) {
                    if (opmodeTimer.getElapsedTimeSeconds()>0) {

                        follower.followPath(getSpeciPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(12);//goes to twelve
                    }
                }
            break;

            case 6:

                if (opmodeTimer.getElapsedTimeSeconds()>0.2) {

                    follower.followPath(readyForHumanPath, true);
                    clawSubsystem.moveDownSp();

                    if (opmodeTimer.getElapsedTimeSeconds()>0.2) {
                        setPathState(5);
                        opmodeTimer.resetTimer();
                    }
                }
                break;



            case 12:
                // Check position before following the next path
                if (opmodeTimer.getElapsedTimeSeconds() > 0.5){

                    if (!elevatorUp) {

                        follower.followPath(getSpeciBackPath, true);

                        clawSubsystem.moveUpSp(); // Move elevator up
                        elevatorUp = true;
                        opmodeTimer.resetTimer();


                    }
                    else{

                            x = x + 3;

                            placeSpecimenPose = new Pose(29, 12+x, 0);
                            specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose)));
                            specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose.getHeading());

                            follower.followPath(specimenToHighChamberPath, true);
                            setPathState(13);

                       }
                }
                break;
            case 13:
                if (follower.getPose().getX() > (placeSpecimenPose.getX() - 0.5) && follower.getPose().getY() < (placeSpecimenPose.getY() + 0.5)) {
                    if (elevatorUp) {
                        clawSubsystem.moveDownBit(); // Move elevator up
                        elevatorUp = false;
                        opmodeTimer.resetTimer();
                    } // Move to the next path after completing the current one
                    else {
                        if (opmodeTimer.getElapsedTimeSeconds()>0.5){
                            clawSubsystem.moveUpBit();
                            highChamberCount++;
                            if (highChamberCount < 3) {
                                setPathState(6);  // Repeat depositing until 4 specimens are placed
                            } else {
                                setPathState(14);


                            }
                        }
                    }
                }
                break;

            case 14:
                // Check position before following the next path
                if (follower.getPose().getX() > (highChamberPose.getX() - 1) && follower.getPose().getY() > (highChamberPose.getY() - 1)) {
                    follower.followPath(returnHomePath, true);
                    setPathState(-1);  // End the autonomous routine
                }
                break;

            case -1:
                telemetry.addData("Status", "Autonomous Complete");
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        opmodeTimer.resetTimer();
    }


}

*/
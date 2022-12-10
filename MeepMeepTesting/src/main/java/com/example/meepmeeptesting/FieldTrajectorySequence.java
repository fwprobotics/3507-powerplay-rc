package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.EmptySequenceException;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class FieldTrajectorySequence {
    public double border;
    public static double coneoffset = 8;
    public static int clawlength = 4;

    public enum sides {
        UP,
        LEFT,
        RIGHT,
        DOWN;
    }


    public double width;
    public double length;
    public Field.autoZones autoZone;
    public Pose2d lastPose;
    public TrajectorySequenceBuilder trajectory;

    public FieldTrajectorySequence(TrajectorySequenceBuilder t, Pose2d startPose, double rW, double rL, Field.autoZones aZ) {
      //  super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);;
        width = rW;
        length = rL;
        autoZone = aZ;
        trajectory = t;
        lastPose = startPose;
        border = (24-getDimension())/2;
    }



    public FieldTrajectorySequence toLocation(Pose2d toPose, boolean xfirst) {
        Pose2d startPose;
        try {
            startPose = trajectory.build().end();
        } catch (EmptySequenceException e) {
            startPose = lastPose;
        }
        if (doesIntersects(startPose, getStreetNum(toPose.getX()), 0, false) && doesIntersects(startPose, 0, getStreetNum(toPose.getY()), true))
        {

            Pose2d startStreet = new Pose2d(getXStreet(startPose, startPose), getYStreet(startPose, startPose), startPose.getHeading());
            double startc;
            if (!xfirst) {
                if ((doesIntersects(startPose, false))) {
                    startc = startPose.getY();
                    trajectory.lineToLinearHeading(new Pose2d(startStreet.getX(), startPose.getY(), startPose.getHeading()));
                } else {
                    startc = startPose.getY();
                }
                if (getYStreet(toPose, startStreet) != startc || (!inStreet(toPose, getStreetNum(startStreet.getY()), true)) && (!inStreet(toPose, getStreetNum(startStreet.getX()), false))) {
                    trajectory.lineToLinearHeading(new Pose2d(startStreet.getX(), getYStreet(toPose, startStreet), startPose.getHeading()));
                }
                if (startStreet.getX() != getXStreet(toPose, startStreet) & (!inStreet(toPose, getStreetNum(getYStreet(toPose, startStreet)), true))) {
                    trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), getYStreet(toPose, startStreet), toPose.getHeading()));
                }
            } else {
                if (doesIntersects(startPose, true)) {
                    startc = startPose.getX();
                    trajectory.lineToLinearHeading(new Pose2d(startPose.getX(), startStreet.getY(), startPose.getHeading()));
                } else {
                    startc = startPose.getX();
                }
                if ((startc != getXStreet(toPose, startStreet)) & (!inStreet(toPose, getStreetNum(startStreet.getY()), true)) & (!inStreet(toPose, getStreetNum(startStreet.getX()), false))) {
                    trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), startStreet.getY(), startStreet.getHeading()));
                }
                if ((getYStreet(toPose, startStreet) != startStreet.getY()) & (!inStreet(toPose, getStreetNum(getXStreet(toPose, startStreet)), false))) {
                    trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), getYStreet(toPose, startStreet), toPose.getHeading()));
                }

            }
        }
        lastPose = toPose;
        trajectory.lineToLinearHeading(toPose);
        return this;

    }
    public FieldTrajectorySequence toPole(int poleX, int poleY, sides side, boolean backwardsDrop, boolean xfirst) {
        Pose2d targetPole = getTargetPole(poleX, poleY, side, backwardsDrop);
        return toLocation(targetPole, xfirst);
    }

    public FieldTrajectorySequence toSignalZone(int zone) {
        double x;
        double y;
        switch (this.autoZone) {
            case REDRIGHT:
                x = (zone*24)-12;
                y = -((getDimension()/2)+border);
                toLocation( new Pose2d(x, y, lastPose.getHeading()), false);
                break;
            case REDLEFT:
                x = -(((4-zone)*24)-12);
                y = -((getDimension()/2)+border);
                toLocation(new Pose2d(x, y, lastPose.getHeading()), false);
                break;
            case BLUERIGHT:
                x = -((zone*24)-12);
                y = (getDimension()/2)+border;
                toLocation(new Pose2d(x, y, lastPose.getHeading()), false);
                break;
            case BLUELEFT:
                x = (((4-zone)*24)-12);
                y = (getDimension()/2)+border;
                toLocation(new Pose2d(x, y, lastPose.getHeading()), false);
                break;

        }
        return this;
    }

    public FieldTrajectorySequence toStack(boolean xfirst) {
        double x = 72-(this.length/2+18.5);
        double y = 12;

        switch (autoZone) {
            case REDRIGHT:
                toLocation(new Pose2d(x, -y, Math.toRadians(0)), xfirst);
                break;
            case REDLEFT:
                toLocation(new Pose2d(-x, -y, Math.toRadians(180)), xfirst);
                break;
            case BLUERIGHT:
                toLocation(new Pose2d(-x, y, Math.toRadians(180)), xfirst);
                break;
            case BLUELEFT:
                toLocation(new Pose2d(x, y, Math.toRadians(0)), xfirst);
                break;
        }
        return  this;
    }

    public TrajectorySequence build() {
        return trajectory.build();
    }

    public boolean doesIntersects(Pose2d start, boolean x) {
        double minX;
        double maxX;
        double minY;
        double maxY;
        if (Math.toDegrees(start.getHeading()) == 0 || Math.toRadians(start.getHeading()) == 180) {
            minX = start.getX()-length/2;
            maxX = start.getX()+length/2;
            minY = start.getY()-width/2;
            maxY = start.getY()+width/2;
        } else {
            minX = start.getX()-width/2;
            maxX = start.getX()+width/2;
            minY = start.getY()-length/2;
            maxY = start.getY()+length/2;
        }
        double xStreet = getStreetNum(getXStreet(start, start));
        double yStreet = getStreetNum(getYStreet(start, start));
        double tileStartX = (xStreet*24)+1;
        double tileEndX = (xStreet*24)+(23);
        double tileStartY = (yStreet*24)+1;
        double tileEndY = (yStreet*24)+(23);
        if ((minX >= tileStartX) & (maxX <= tileEndX) & (!x)) {
            return false;
        } else if ( (x) & (minY >= tileStartY) & (maxY <= tileEndY)){
            return false;
        } else {
            return true;
        }
//        if ((Math.abs(start % 24) > (getDimension()/2)-6 ) & Math.abs(start % 24) <  (24 - ((getDimension() /2)-6))) {
//            return false;
//        } else {
//            return true;
//        }
    }

    public boolean doesIntersects(Pose2d start, double xStreet, double yStreet, boolean x) {
        double minX;
        double maxX;
        double minY;
        double maxY;
        if (Math.toDegrees(start.getHeading()) == 0 || Math.toRadians(start.getHeading()) == 180) {
            minX = start.getX()-(length/2+clawlength);
            maxX = start.getX()+(length/2+clawlength);
            minY = start.getY()-width/2;
            maxY = start.getY()+width/2;
        } else {
            minX = start.getX()-width/2;
            maxX = start.getX()+width/2;
            minY = start.getY()-(length/2-clawlength);
            maxY = start.getY()+(length/2-clawlength);
        }
        double tileStartX = (xStreet*24)+1;
        double tileEndX = (xStreet*24)+(23);
        double tileStartY = (yStreet*24)+1;
        double tileEndY = (yStreet*24)+(23);
        if ((minX >= tileStartX) & (maxX <= tileEndX) & (!x)) {
            return false;
        } else if ( (x) & (minY >= tileStartY) & (maxY <= tileEndY)){
            return false;
        } else {
            return true;
        }
//        if ((Math.abs(start % 24) > (getDimension()/2)-6 ) & Math.abs(start % 24) <  (24 - ((getDimension() /2)-6))) {
//            return false;
//        } else {
//            return true;
//        }
    }



    public boolean inStreet(Pose2d loc, double street, boolean x ) {
        double coord;
        if (x) {
            coord = loc.getY();
        } else {
            coord = loc.getX();
        }
            if ((coord > (street*24)+(getDimension()/2)) & (coord < (((street)*24))+(24-getDimension()/2) )) {
                return true;
            } else {
                return false;
            }
            }
    public double getDimension() {
        if (this.length > this.width) {
            return this.length;
        } else {
            return this.width;
        }
    }

    public double getStreetNum(double lane) {
        return Math.floor(lane/24);
    }

    public double getXStreet(Pose2d pos, Pose2d start) {
        if (pos.getX()%24 == 0) {
            double streetNum = pos.getX()/24;
            double lane1 = (streetNum * 24) + border + getDimension() / 2;
            double lane2 = ((streetNum-1) * 24) + border + getDimension() / 2;
            if (Math.abs(lane1-start.getX()) <= Math.abs(lane2-start.getX())) {
                return lane1;
            } else {
                return lane2;
            }
        } else {
            double streetNum = Math.floor(pos.getX() / 24);
            double lane = (streetNum * 24) + border + getDimension() / 2;
            return lane;
        }
    }
    public double getYStreet(Pose2d pos, Pose2d start) {
        if (pos.getY()%24 == 0) {
            double streetNum = pos.getY()/24;
            double lane1 = (streetNum * 24) + border + getDimension() / 2;
            double lane2 = ((streetNum-1) * 24) + border + getDimension() / 2;
            if (Math.abs(lane1-start.getY()) <= Math.abs(lane2-start.getY())) {
                return lane1;
            } else {
                return lane2;
            }
        } else {
            double streetNum = Math.floor(pos.getY() / 24);
            double lane = (streetNum * 24) + border + getDimension() / 2;
            return lane;
        }
    }
    public Pose2d getTargetPole(int poleX, int poleY, sides side, boolean backwardsDrop) {
        poleX *= 24;
        poleY *= 24;

        int heading = 0;
        switch (side) {
            case LEFT:
                poleX -= this.length/2+coneoffset;
                heading = 0;
                if (backwardsDrop) heading = 180;
                break;
            case RIGHT:
                poleX += this.length/2+coneoffset;
                heading = 180;
                if (backwardsDrop) heading = 0;
                break;
            case UP:
                poleY += this.length/2+coneoffset;
                heading = -90;
                if (backwardsDrop) heading = 90;
                break;
            case DOWN:
                poleY -= this.length/2+coneoffset;
                heading = 90;
                if (backwardsDrop) heading = -90;
                break;

        }
        return new Pose2d(poleX, poleY, Math.toRadians(heading));
    }

}

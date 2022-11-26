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
    public static int border = 6;

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
    }



    public FieldTrajectorySequence toLocation(Pose2d toPose, boolean xfirst) {
        Pose2d startPose;
        try {
            startPose = trajectory.build().end();
        } catch (EmptySequenceException e) {
            startPose = lastPose;
        }

        Pose2d startStreet = new Pose2d(getXStreet(startPose, startPose), getYStreet(startPose, startPose), startPose.getHeading());
        double startc;
        if (!xfirst) {
            if (doesIntersects(startPose.getX())) {
                startc = startStreet.getY();
                trajectory.lineToLinearHeading(new Pose2d(startStreet.getX(), startPose.getY(), startPose.getHeading()));
            } else {
                startc = startPose.getY();
            }
            if (getYStreet(toPose, startStreet) != startc) {
                trajectory.lineToLinearHeading(new Pose2d(startStreet.getX(), getYStreet(toPose, startStreet), toPose.getHeading()));
            }
            if (startStreet.getX() != getXStreet(toPose, startStreet) &(!inStreet(toPose, getStreetNum(getYStreet(toPose, startStreet)), true))) {
               trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), getYStreet(toPose, startStreet), toPose.getHeading()));
            }
        } else {
            if (doesIntersects(startPose.getY())) {
                startc = startStreet.getX();
                trajectory.lineToLinearHeading(new Pose2d(startPose.getX(), startStreet.getY(), startPose.getHeading()));
            } else {
                startc = startPose.getX();
            }
            if (startc != getXStreet(toPose, startStreet)) {
                trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), startStreet.getY(), toPose.getHeading()));
            }
            if ((getYStreet(toPose, startStreet) != startStreet.getY())  &(!inStreet(toPose, getStreetNum(getXStreet(toPose, startStreet)), false))) {
                trajectory.lineToLinearHeading(new Pose2d(getXStreet(toPose, startStreet), getYStreet(toPose, startStreet), toPose.getHeading()));
            }

        }
        lastPose = toPose;
        trajectory.lineToLinearHeading(toPose);
        return this;

    }
    public FieldTrajectorySequence toPole(int poleX, int poleY, sides side, boolean xfirst) {
        Pose2d targetPole = getTargetPole(poleX, poleY, side);
        return toLocation(targetPole, xfirst);
    }

    public FieldTrajectorySequence toSignalZone(int zone) {
        double x;
        double y;
        switch (this.autoZone) {
            case REDRIGHT:
                x = (zone*24)-12;
                y = -((getDimension()/2)+border);
                toLocation( new Pose2d(x, y, Math.toRadians(90)), false);
                break;
            case REDLEFT:
                x = ((4-zone)*24)-12;
                y = -((getDimension()/2)+border);
                toLocation(new Pose2d(x, y, Math.toRadians(90)), false);
                break;
            case BLUERIGHT:
                x = ((4-zone)*24)-12;
                y = (getDimension()/2)+border;
                toLocation(new Pose2d(x, y, Math.toRadians(-90)), false);
                break;
            case BLUELEFT:
                x = (zone*24)-12;
                y = (getDimension()/2)+border;
                toLocation(new Pose2d(x, y, Math.toRadians(-90)), false);
                break;

        }
        return this;
    }

    public FieldTrajectorySequence toStack(boolean xfirst) {
        double x = 72-this.length;
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

    public boolean doesIntersects(double start) {
        if ((Math.abs(start % 24) > getDimension()/2 ) & Math.abs(start % 24) <  (24 - (getDimension() /2))) {
            return false;
        } else {
            return true;
        }
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
    public Pose2d getTargetPole(int poleX, int poleY, sides side) {
        poleX *= 24;
        poleY *= 24;
        int heading = 0;
        switch (side) {
            case LEFT:
                poleX -= this.length;
                heading = 0;
                break;
            case RIGHT:
                poleX += this.length;
                heading = 180;
                break;
            case UP:
                poleY += this.length;
                heading = 270;
                break;
            case DOWN:
                poleY -= this.length;
                heading = 90;
                break;

        }
        return new Pose2d(poleX, poleY, Math.toRadians(heading));
    }

}

package frc.robot.Robot.Drive;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Robot.Constants;

public class MotorController {
    //The physical controller
    public PS4Controller driveController;

    //The PID controller, in order to augment the inputs of the controller
    public PIDController speedAugment;
    public PIDController turnAugment;

    public double[] tuneKpSpeed = new double[]{0.0, 0.0, 0.0};
    public double[] tuneKiSpeed = new double[]{0.0, 0.0, 0.0};
    public double[] tuneKdSpeed = new double[]{0.0, 0.0, 0.0};
    public double[] tuneKpTurn = new double[]{0.0, 0.0, 0.0};
    public double[] tuneKiTurn = new double[]{0.0, 0.0, 0.0};
    public double[] tuneKdTurn = new double[]{0.0, 0.0, 0.0};

    public double[][] tuneStuff = new double[][]{tuneKpSpeed, tuneKiSpeed, tuneKdSpeed, tuneKpTurn, tuneKiTurn, tuneKdTurn};
    public int curr = 0;
    public boolean prevBad = false;
    public boolean first = true;

    public MotorController(PS4Controller driveController, String augmentFile) {
        this.driveController = driveController;

        try {
            Scanner augmentReader = new Scanner(new File(augmentFile));
            double kpSpeed = Double.valueOf(augmentReader.nextLine());
            double kiSpeed = Double.valueOf(augmentReader.nextLine());
            double kdSpeed = Double.valueOf(augmentReader.nextLine());
            double kpTurn = Double.valueOf(augmentReader.nextLine());
            double kiTurn = Double.valueOf(augmentReader.nextLine());
            double kdTurn = Double.valueOf(augmentReader.nextLine());

            speedAugment = new PIDController(kpSpeed, kiSpeed, kdSpeed);
            turnAugment = new PIDController(kpTurn, kiTurn, kdTurn);

            tuneKpSpeed[0] = kpSpeed;
            tuneKiSpeed[0] = kiSpeed;
            tuneKdSpeed[0] = kdSpeed;
            tuneKpTurn[0] = kpTurn;
            tuneKiTurn[0] = kiTurn;
            tuneKdTurn[0] = kdTurn;

            speedAugment.setTolerance(Constants.driveSpeedPositionTolerance, Constants.driveSpeedVelocityTolerance);
            turnAugment.setTolerance(Constants.driveTurnPositionTolerance, Constants.driveTurnVelocityTolerance);
            
            augmentReader.close();

        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    public double[] update(double[] last) {
        if(Math.abs(driveController.getLeftY()) <= Constants.driveSquareBelow) {
            speedAugment.setSetpoint(Math.pow(driveController.getLeftY(), 2));
        } else {
            speedAugment.setSetpoint(driveController.getLeftY());
        }

        if(Math.abs(driveController.getRightX()) <= Constants.driveSquareBelow) {
            turnAugment.setSetpoint(Math.pow(driveController.getRightX(), 2));
        } else {
            turnAugment.setSetpoint(driveController.getRightX());
        }

        if(speedAugment.atSetpoint() && Math.abs(speedAugment.getSetpoint()) <= speedAugment.getPositionTolerance()) {
            speedAugment.reset();
        }

        if(turnAugment.atSetpoint() && Math.abs(turnAugment.getSetpoint()) <= turnAugment.getPositionTolerance()) {
            turnAugment.reset();
        }

        return new double[]{speedAugment.calculate(last[0]), turnAugment.calculate(last[1])};
    }

    public void tune(boolean better) {
        switch(curr) {
            case 0:
                if(first) {
                    first = false;
                    tuneKpSpeed[0] *= 2;
                    
                    speedAugment.setP(tuneKpSpeed[0]);
                    break;
                } else {
                    tuneKpSpeed[2] = (speedAugment.getP() - tuneKpSpeed[0]) / 2.0;
                    tuneKpSpeed[0] = speedAugment.getP();
                    tuneKpSpeed[1] += tuneKpSpeed[2];
    
                    if(prevBad) {
                        tuneKpSpeed[1] = 0.0;
                        prevBad = false;
                    }
    
                    if(better) {
                        speedAugment.setP(tuneKpSpeed[0] + tuneKpSpeed[1] + tuneKpSpeed[2]);
                    } else {
                        prevBad = true;
                        tuneKpSpeed[1] = 0.0;
    
                        speedAugment.setP(tuneKpSpeed[0] - tuneKpSpeed[2]);
                    }
    
                    break;
                }
            case 1:
                if(first) {
                    first = false;
                    tuneKiSpeed[0] *= 2;
                    
                    speedAugment.setI(tuneKiSpeed[0]);
                    break;
                } else {
                    tuneKiSpeed[2] = (speedAugment.getI() - tuneKiSpeed[0]) / 2.0;
                    tuneKiSpeed[0] = speedAugment.getI();
                    tuneKiSpeed[1] += tuneKiSpeed[2];
    
                    if(prevBad) {
                        tuneKiSpeed[1] = 0.0;
                        prevBad = false;
                    }
    
                    if(better) {
                        speedAugment.setI(tuneKiSpeed[0] + tuneKiSpeed[1] + tuneKiSpeed[2]);
                    } else {
                        prevBad = true;
                        tuneKiSpeed[1] = 0.0;
    
                        speedAugment.setI(tuneKiSpeed[0] - tuneKiSpeed[2]);
                    }
    
                    break;
                }
            case 2:
                if(first) {
                    first = false;
                    tuneKdSpeed[0] *= 2;
                    
                    speedAugment.setD(tuneKdSpeed[0]);
                    break;
                } else {
                    tuneKdSpeed[2] = (speedAugment.getD() - tuneKdSpeed[0]) / 2.0;
                    tuneKdSpeed[0] = speedAugment.getD();
                    tuneKdSpeed[1] += tuneKdSpeed[2];
    
                    if(prevBad) {
                        tuneKdSpeed[1] = 0.0;
                        prevBad = false;
                    }
    
                    if(better) {
                        speedAugment.setD(tuneKdSpeed[0] + tuneKdSpeed[1] + tuneKdSpeed[2]);
                    } else {
                        prevBad = true;
                        tuneKdSpeed[1] = 0.0;
    
                        speedAugment.setP(tuneKdSpeed[0] - tuneKdSpeed[2]);
                    }
    
                    break;
                }
            case 3:
                if(first) {
                    first = false;
                    tuneKpTurn[0] *= 2;
                    
                    turnAugment.setP(tuneKpTurn[0]);
                    break;
                } else {

                }
                tuneKpTurn[2] = (turnAugment.getP() - tuneKpTurn[0]) / 2.0;
                tuneKpTurn[0] = turnAugment.getP();
                tuneKpTurn[1] += tuneKpTurn[2];

                if(prevBad) {
                    tuneKpTurn[1] = 0.0;
                    prevBad = false;
                }

                if(better) {
                    turnAugment.setP(tuneKpTurn[0] + tuneKpTurn[1] + tuneKpTurn[2]);
                } else {
                    prevBad = true;
                    tuneKpTurn[1] = 0.0;

                    turnAugment.setP(tuneKpTurn[0] - tuneKpTurn[2]);
                }

                break;
            case 4:
                if(first) {
                    first = false;
                    tuneKiTurn[0] *= 2;
                    
                    turnAugment.setI(tuneKiTurn[0]);
                    break;
                } else {
                    tuneKiTurn[2] = (turnAugment.getI() - tuneKiTurn[0]) / 2.0;
                    tuneKiTurn[0] = turnAugment.getI();
                    tuneKiTurn[1] += tuneKiTurn[2];
    
                    if(prevBad) {
                        tuneKiTurn[1] = 0.0;
                        prevBad = false;
                    }
    
                    if(better) {
                        turnAugment.setI(tuneKiTurn[0] + tuneKiTurn[1] + tuneKiTurn[2]);
                    } else {
                        prevBad = true;
                        tuneKiTurn[1] = 0.0;
    
                        turnAugment.setI(tuneKiTurn[0] - tuneKiTurn[2]);
                    }
    
                    break;   
                }
            case 5:
                if(first) {
                    first = false;
                    tuneKdTurn[0] *= 2;
                    
                    turnAugment.setD(tuneKdTurn[0]);
                    break;
                } else {
                    tuneKdTurn[2] = (turnAugment.getD() - tuneKdTurn[0]) / 2.0;
                    tuneKdTurn[0] = turnAugment.getD();
                    tuneKdTurn[1] += tuneKdTurn[2];
    
                    if(prevBad) {
                        tuneKdTurn[1] = 0.0;
                        prevBad = false;
                    }
    
                    if(better) {
                        turnAugment.setD(tuneKdTurn[0] + tuneKdTurn[1] + tuneKdTurn[2]);
                    } else {
                        prevBad = true;
                        tuneKdTurn[1] = 0.0;
    
                        turnAugment.setD(tuneKdTurn[0] - tuneKdTurn[2]);
                    }
    
                    break;   
                }
        }

        if(curr <= 2 && Math.abs(tuneStuff[curr][2]) <= Constants.tuneSpeedPrecision) {
            curr++;
        } else if(curr > 2 && Math.abs(tuneStuff[curr][2]) <= Constants.tuneTurnPrecision) {
            curr++;

            if(curr > 5) {
                curr = 0;
            }
        }
    }

    public void close() {
        speedAugment.close();
        turnAugment.close();
    }
}
package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class LoadTrajectory {
    //linked list to hold data
    LinkedList<Double[]> trajectoryData = new LinkedList<Double[]>();
    //double array for all starting
    //new array for each intermediary
    //final arry for ending

    String path = Filesystem.getDeployDirectory().toString();

    public LoadTrajectory(String path){
        this.path += path;
        loadTrajectoryData();
    }

    public void loadTrajectoryData(){
        String line = "";
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            while((line = br.readLine()) != null){
                String[] values = line.split(",");
                Double[] doubleValues = new Double[values.length];
                if(values.length == 2){
                    for(int i = 0; i <= 1; i++){
                        doubleValues[i] = Double.parseDouble(values[i]);
                    }
                }
                else{
                    for(int i = 0; i < doubleValues.length; i++){
                        doubleValues[i] = Double.parseDouble(values[i]);
                    }
                }
                

                trajectoryData.add(doubleValues);
            }
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } 

        for(Double[] x : trajectoryData){
            System.out.print(x);
        }
        
    }

    public Pose2d getInitialPose(){
        return new Pose2d(trajectoryData.get(0)[0]*1.42, trajectoryData.get(0)[1]*1.42, new Rotation2d(Math.toRadians(trajectoryData.get(0)[2])));
    }

    public double getInitialVelocity(){
        return trajectoryData.get(0)[3];
    }

    public List<Translation2d> getTranslations(){
        List<Translation2d> translations = new ArrayList<Translation2d>();
        for(int i = 1; i < trajectoryData.size()-1; i++){
            translations.add(new Translation2d(trajectoryData.get(i)[0], trajectoryData.get(i)[1]));
        }
        return translations;
    }

    public Pose2d getFinalPose(){
        return new Pose2d(trajectoryData.get(trajectoryData.size()-1)[0]*1.42, trajectoryData.get(trajectoryData.size()-1)[1]*1.42, new Rotation2d(Math.toRadians(trajectoryData.get(trajectoryData.size()-1)[2])));
    }

    public double getFinalVelocity(){
        return trajectoryData.get(trajectoryData.size()-1)[3];
    }
}

package frc.robot;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterData {

    public ShooterData(){
        loadShooterData();
    }

    //LinkedList to read csv files
    LinkedList<Double[]> shooterData = new LinkedList<Double[]>();

    public void loadShooterData(){
        String path = Filesystem.getDeployDirectory().toString() + "/shooter.csv";
        String line = "";
        try {
          try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            while((line = br.readLine()) != null){
                  String[] values = line.split(",");
                  String[] newValues = new String[values.length-1];
                  for(int i = 1; i < values.length; i++){
                    newValues[i-1] = values[i];
                  }
   
                  Double[] newNewValues = new Double[newValues.length];
                  for(int i = 0; i < newNewValues.length; i++){
                    newNewValues[i] = Double.parseDouble(newValues[i]);
                  }
                  
                  
   
                  shooterData.add(newNewValues);
   
                  
              }
        } catch (NumberFormatException e) {
            e.printStackTrace();
        }
        } catch (FileNotFoundException e) {
          DriverStation.reportError("File Not Found", true);
          e.printStackTrace();
        } catch(IOException e){
          DriverStation.reportError("IO Exception", true);
          e.printStackTrace();
        }
     
        
      }



      int lowerBound(double distance){
            for(int i = 0; i < shooterData.get(0).length; i++){
                if(distance <= shooterData.get(0)[i]){
                    return i-1;
                }
            }

            return -1;
      }

      

        public double[] getData(double distance){
        int lBound = lowerBound(distance);
       // SmartDashboar.putNumber("LBOUND", lBound);
        if(lBound == shooterData.get(0).length-1){
            double[] temp = {shooterData.get(1)[shooterData.get(0).length-1], shooterData.get(2)[shooterData.get(0).length-1]};
            return temp;
        }
        else{
            double distanceFraction = 1.0*(distance - shooterData.get(0)[lBound]) / (shooterData.get(0)[lBound+1] - shooterData.get(0)[lBound]);
            double rpmInterp = ((shooterData.get(2)[lBound+1] - shooterData.get(2)[lBound]) * distanceFraction) + shooterData.get(2)[lBound];
            double angleInterp = ((shooterData.get(1)[lBound+1] - shooterData.get(1)[lBound]) * distanceFraction) + shooterData.get(1)[lBound];
            double[] temp1 = {angleInterp, rpmInterp};
            return temp1;
        }
      }

      
}

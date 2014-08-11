1) rosrun kinematic_calibration validation
2) Bagfiles abspielen (Kamera + Messungen)

3) Parameter:
  - validation_data_strategy: split [ | all | others ]
    --> split: optimiert auf den Daten gemäß der IDs, die im Parameter "optimization_ids: <list_of_ids>" angegeben sind
    und validiert auf den Daten gemäß der IDs, die im Parameter "validation_ids: <list_of_ids>" angegeben sind
    [--> all: optimiert auf den Daten gemäß der IDs, die im Parameter "optimization_ids: <list_of_ids>" angegeben sind
    und validiert auf ALLEN Daten]
    [--> others: optimiert auf den Daten gemäß der IDs, die im Parameter "optimization_ids: <list_of_ids>" angegeben sind
    und validiert auf den RESTLICHEN Daten]
    
  - folder_name: <folder_name>
    --> Ordner, in den die Ergebnisse (für EINE Optimierung) geschrieben werden 
    
  - andere Parameter wie bisher (siehe /config/params_general.yaml)  

4) rosservice call /${NAMESPACE}/kinematic_calibration/start_optimization

5) Bei Bedarf: bei 3) weitermachen (Node läuft weiter, dadurch muss man die Bagfiles nicht mehr erneut abspielen), ansonsten Ctrl+C
within VirtualFCS.EMS_experiment.EMS_simulation.RB;

model Vehicle_RB_simulation_charge_only
  extends Modelica.Icons.Example;
  inner Modelica.Fluid.System system annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //Real fuel_consumption(unit = "kg/100km", start = 0) "The total fuel consumption at the end of simulation";
  //Modelica.Units.SI.Mass hydrogen_mass_init(start = 0) "Initial value of hydrogen mass in tank";
  Modelica.Units.SI.Efficiency eta_vehicle "Vehicle efficiency";
  VirtualFCS.EMS_experiment.EMS_simulation.RB.PowerTrain_RB_simulation powerTrain_RB_simulation(C_bat_pack = 180, SOC_init = 0.33753, V_max_bat_pack = 54.75, V_min_bat_pack = 37.5, V_nom_bat_pack = 48)  annotation(
    Placement(visible = true, transformation(origin = {40, 1.77636e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  VirtualFCS.EMS_experiment.DriveCycle_EMS_experiment_charge_only driveCycle_EMS_experiment_charge_only(fileName = "C:/Users/benjamins/OneDrive - SINTEF/Documents/HyOPT/XInTheLoop/XInTheLoop/Resources/Data/DriveProfile.mat", tableName = "X")  annotation(
    Placement(visible = true, transformation(origin = {-41, 1}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
//when time > 0.1 then
//hydrogen_mass_init = powerTrain_RB_simulation.fuelCellSystem.fuelCellSubSystems.subSystemHydrogen.tankHydrogen.m;
//end when;
//when terminal() then
//fuel_consumption = (hydrogen_mass_init - powerTrain_RB_simulation.fuelCellSystem.fuelCellSubSystems.subSystemHydrogen.tankHydrogen.m)/max((vehicleProfile.x*0.00001), 0.01);
//end when;
  if driveCycle_EMS_experiment_charge_only.P > 0 then
    eta_vehicle = max(((driveCycle_EMS_experiment_charge_only.P)/max((driveCycle_EMS_experiment_charge_only.P + (powerTrain_RB_simulation.Power_del_DC_DC*(1 - (powerTrain_RB_simulation.eta_drivetrain*0.01))) + (driveCycle_EMS_experiment_charge_only.P*(1 - driveCycle_EMS_experiment_charge_only.eff_drivetrain))), 0.00001))*100, 0);
  else
    eta_vehicle = max(((abs(driveCycle_EMS_experiment_charge_only.P))/max(abs(driveCycle_EMS_experiment_charge_only.P) + (abs(powerTrain_RB_simulation.Power_del_DC_DC)*(1 - (powerTrain_RB_simulation.eta_drivetrain*0.01))) + (abs(driveCycle_EMS_experiment_charge_only.P)*(1 - driveCycle_EMS_experiment_charge_only.eff_brake)), 0.00001))*100, 0);
  end if;
  connect(driveCycle_EMS_experiment_charge_only.pin_p, powerTrain_RB_simulation.pin_p) annotation(
    Line(points = {{-22, 12}, {22, 12}, {22, 10}}, color = {0, 0, 255}));
  connect(driveCycle_EMS_experiment_charge_only.pin_n, powerTrain_RB_simulation.pin_n) annotation(
    Line(points = {{-22, -8}, {22, -8}, {22, -10}}, color = {0, 0, 255}));
protected
  annotation(
    experiment(StartTime = 0, StopTime = 1490, Tolerance = 0.0001, Interval = 1));
end Vehicle_RB_simulation_charge_only;
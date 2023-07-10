within VirtualFCS.EMS_experiment.EMS_simulation.OB;

model Vehicle_OB_simulation
  extends Modelica.Icons.Example;
  inner Modelica.Fluid.System system annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //Real fuel_consumption(unit = "kg/100km", start = 0) "The total fuel consumption at the end of simulation";
  //Modelica.Units.SI.Mass hydrogen_mass_init(start = 0) "Initial value of hydrogen mass in tank";
  Modelica.Units.SI.Efficiency eta_vehicle "Vehicle efficiency";
  VirtualFCS.EMS_experiment.DriveCycle_EMS_experiment driveCycle_EMS_experiment(fileName = "C:/Users/benjamins/OneDrive - SINTEF/Documents/HyOPT/XInTheLoop/XInTheLoop/Resources/Data/DriveProfile.mat", tableName = "X") annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  VirtualFCS.EMS_experiment.EMS_simulation.OB.PowerTrain_OB_simulation powerTrain_OB_simulation(C_bat_pack = 180, SOC_init = 0.3, V_max_bat_pack = 54.75, V_min_bat_pack = 37.5, V_nom_bat_pack = 48, V_tank_H2 = 0.074, p_tank_H2 = 3800000)  annotation(
    Placement(visible = true, transformation(origin = {40, 1.77636e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
//when time > 0.1 then
//hydrogen_mass_init = rangeExtenderPowerTrain.fuelCellSystem.fuelCellSubSystems.subSystemHydrogen.tankHydrogen.m;
//end when;
//when terminal() then
//fuel_consumption = (hydrogen_mass_init - rangeExtenderPowerTrain.fuelCellSystem.fuelCellSubSystems.subSystemHydrogen.tankHydrogen.m)/max((vehicleProfile.x*0.00001), 0.01);
//end when;
  if driveCycle_EMS_experiment.P > 0 then
    eta_vehicle = max(((driveCycle_EMS_experiment.P)/max((driveCycle_EMS_experiment.P + (powerTrain_OB_simulation.Power_del_DC_DC*(1 - (powerTrain_OB_simulation.eta_drivetrain*0.01))) + (driveCycle_EMS_experiment.P*(1 - driveCycle_EMS_experiment.eff_drivetrain))), 0.00001))*100, 0);
  else
    eta_vehicle = max(((abs(driveCycle_EMS_experiment.P))/max(abs(driveCycle_EMS_experiment.P) + (abs(powerTrain_OB_simulation.Power_del_DC_DC)*(1 - (powerTrain_OB_simulation.eta_drivetrain*0.01))) + (abs(driveCycle_EMS_experiment.P)*(1 - driveCycle_EMS_experiment.eff_brake)), 0.00001))*100, 0);
  end if;
  connect(driveCycle_EMS_experiment.pin_p, powerTrain_OB_simulation.pin_p) annotation(
    Line(points = {{-22, 10}, {22, 10}}, color = {0, 0, 255}));
  connect(driveCycle_EMS_experiment.pin_n, powerTrain_OB_simulation.pin_n) annotation(
    Line(points = {{-22, -10}, {22, -10}}, color = {0, 0, 255}));
protected
  annotation(
    experiment(StartTime = 0, StopTime = 1490, Tolerance = 0.0001, Interval = 1));
end Vehicle_OB_simulation;

within VirtualFCS.ComponentTesting.ECMS_experiment_testrun;

model Vehicle_ECMS_experiment_testrun
  extends Modelica.Icons.Example;
  inner Modelica.Fluid.System system annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Real fuel_consumption(unit = "kg/100km", start = 0) "The total fuel consumption at the end of simulation";
  Real hydrogen_mass_init(unit = "kg", start = 0) "Initial value of hydrogen mass in tank";
  Real eta_vehicle(unit = "100") "Vehicle efficiency";
  VirtualFCS.ComponentTesting.ECMS_experiment_testrun.DriveCycle_experiment_testrun driveCycle_experiment_testrun(fileName = "C:/Users/benjamins/OneDrive - SINTEF/Documents/Virtual-FCS/EMS/Experiment_2023_drivecycle.mat", tableName = "drivecycle")  annotation(
    Placement(visible = true, transformation(origin = {-40, -6.66134e-16}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
  VirtualFCS.ComponentTesting.ECMS_experiment_testrun.PowerTrainECMS_experiment_testrun powerTrainECMS_experiment_testrun(C_bat_pack = 180, I_rated_FC_stack = 100, N_FC_stack = 180, SOC_init = 0.3, V_max_bat_pack = 54.75, V_min_bat_pack = 37.5, V_nom_bat_pack = 48, V_rated_FC_stack = 116, i_L_FC_stack = 130)  annotation(
    Placement(visible = true, transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  when time > 0.1 then
//hydrogen_mass_init = powerTrainECMS.fCSystem.fCSubSystems.subSystemHydrogen.tankHydrogen.m;
    hydrogen_mass_init = 0;
  end when;
  when terminal() then
//fuel_consumption = (hydrogen_mass_init - powerTrainECMS.fCSystem.fCSubSystems.subSystemHydrogen.tankHydrogen.m)/max((vehicleProfile.x*0.00001), 0.01);
    fuel_consumption = 0;
  end when;
  if driveCycle_experiment_testrun.P > 0 then
    eta_vehicle = max(((driveCycle_experiment_testrun.P)/max((driveCycle_experiment_testrun.P + (powerTrainECMS_experiment_testrun.Power_del_DC_DC*(1 - (powerTrainECMS_experiment_testrun.eta_drivetrain*0.01))) + (driveCycle_experiment_testrun.P*(1 - driveCycle_experiment_testrun.eff_drivetrain))), 0.00001))*100, 0);
  else
    eta_vehicle = max(((abs(driveCycle_experiment_testrun.P))/max(abs(driveCycle_experiment_testrun.P) + (abs(powerTrainECMS_experiment_testrun.Power_del_DC_DC)*(1 - (powerTrainECMS_experiment_testrun.eta_drivetrain*0.01))) + (abs(driveCycle_experiment_testrun.P)*(1 - driveCycle_experiment_testrun.eff_brake)), 0.00001))*100, 0);
  end if;
  connect(driveCycle_experiment_testrun.pin_p, powerTrainECMS_experiment_testrun.pin_p) annotation(
    Line(points = {{-22, 10}, {22, 10}}, color = {0, 0, 255}));
  connect(driveCycle_experiment_testrun.pin_n, powerTrainECMS_experiment_testrun.pin_n) annotation(
    Line(points = {{-22, -10}, {22, -10}}, color = {0, 0, 255}));
protected
  annotation(
    experiment(StartTime = 0, StopTime = 1750, Tolerance = 1e-04, Interval = 1));
end Vehicle_ECMS_experiment_testrun;

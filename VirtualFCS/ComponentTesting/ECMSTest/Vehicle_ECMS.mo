within VirtualFCS.ComponentTesting.ECMSTest;

model Vehicle_ECMS
  extends Modelica.Icons.Example;
  inner Modelica.Fluid.System system annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Vehicles.VehicleProfile vehicleProfile(VN = VirtualFCS.Vehicles.VehicleProfile.vehicle_name.Mirai, useRegenerativeBreaking = true) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
  VirtualFCS.Vehicles.DriveCycle driveCycle(v = VirtualFCS.Vehicles.DriveCycle.speed_profile.NEDC) annotation(
    Placement(visible = true, transformation(origin = {-60, -6.66134e-16}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
  Real fuel_consumption(unit = "kg/100km", start = 0) "The total fuel consumption at the end of simulation";
  Real hydrogen_mass_init(unit = "kg", start = 0) "Initial value of hydrogen mass in tank";
  Real eta_vehicle(unit = "100") "Vehicle efficiency";
  VirtualFCS.ComponentTesting.ECMSTest.PowerTrainECMS powerTrainECMS(SOC_init = 0.5)  annotation(
    Placement(visible = true, transformation(origin = {60, 1.77636e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  when time > 0.1 then
//hydrogen_mass_init = powerTrainECMS.fCSystem.fCSubSystems.subSystemHydrogen.tankHydrogen.m;
    hydrogen_mass_init = 0;
  end when;
  when terminal() then
//fuel_consumption = (hydrogen_mass_init - powerTrainECMS.fCSystem.fCSubSystems.subSystemHydrogen.tankHydrogen.m)/max((vehicleProfile.x*0.00001), 0.01);
    fuel_consumption = 0;
  end when;
  if vehicleProfile.useRegenerativeBreaking then
    if vehicleProfile.P > 0 then
      eta_vehicle = max(((vehicleProfile.P)/max((vehicleProfile.P + (powerTrainECMS.Power_del_DC_DC*(1 - (powerTrainECMS.eta_drivetrain*0.01))) + (vehicleProfile.F_drag*vehicleProfile.v) + (vehicleProfile.F_accel*vehicleProfile.v) + (vehicleProfile.F_roll*vehicleProfile.v) + (vehicleProfile.P*(1 - vehicleProfile.eff_drivetrain))), 0.00001))*100, 0);
    else
      eta_vehicle = max(((abs(vehicleProfile.P))/max(abs(vehicleProfile.P) + (abs(powerTrainECMS.Power_del_DC_DC)*(1 - (powerTrainECMS.eta_drivetrain*0.01))) + (vehicleProfile.F_drag*vehicleProfile.v) + (abs(vehicleProfile.F_accel)*vehicleProfile.v) + (vehicleProfile.F_roll*vehicleProfile.v) + (abs(vehicleProfile.P)*(1 - vehicleProfile.eff_brake)), 0.00001))*100, 0);
    end if;
  else
    if vehicleProfile.P > 0 then
      eta_vehicle = max(((vehicleProfile.P)/max((vehicleProfile.P + (powerTrainECMS.Power_del_DC_DC*(1 - (powerTrainECMS.eta_drivetrain*0.01))) + (vehicleProfile.F_drag*vehicleProfile.v) + (vehicleProfile.F_accel*vehicleProfile.v) + (vehicleProfile.F_roll*vehicleProfile.v) + (vehicleProfile.P*(1 - vehicleProfile.eff_drivetrain))), 0.00001))*100, 0);
    else
      eta_vehicle = 0;
    end if;
  end if;
  connect(driveCycle.y, vehicleProfile.vehicleVelocity) annotation(
    Line(points = {{-37, 0}, {-26, 0}}, color = {0, 0, 127}));
  connect(vehicleProfile.pin_p, powerTrainECMS.pin_p) annotation(
    Line(points = {{18, 10}, {42, 10}}, color = {0, 0, 255}));
  connect(vehicleProfile.pin_n, powerTrainECMS.pin_n) annotation(
    Line(points = {{18, -10}, {42, -10}}, color = {0, 0, 255}));
protected
  annotation(
    experiment(StartTime = 0, StopTime = 10000, Tolerance = 1e-04, Interval = 1));
end Vehicle_ECMS;

within VirtualFCS.EMS_experiment;

block EMS_RB
  parameter Real I_nom_FC_stack(unit = "A") = 45 "FC stack nominal operating current";
  // Assumed to be maximum FC system efficiency operating point
  parameter Real ramp_up(unit = "1/s") = 20 "FC stack current ramp up rate";
  parameter Real SOC_max(unit = "1") = 0.8 "Maximum SOC limit";
  parameter Real SOC_min(unit = "1") = 0.2 "Minimum SOC limit";
  Modelica.Blocks.Sources.Constant shut_down(k = 0) annotation(
    Placement(visible = true, transformation(origin = {-70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Hysteresis hysteresis(pre_y_start = true, uHigh = SOC_max, uLow = SOC_min) annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant setFuelCellCurrent(k = -I_nom_FC_stack) annotation(
    Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Abs abs1 annotation(
    Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput sensorSOC annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-121, -1}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch switch1 annotation(
    Placement(visible = true, transformation(origin = {18, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = ramp_up) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput controlInterface annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput sensorBattVoltage annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput sensorFCVoltage annotation(
    Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {-20, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MultiProduct multiProduct(nu = 2) annotation(
    Placement(visible = true, transformation(origin = {-46, -38}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Math.Max max2 annotation(
    Placement(visible = true, transformation(origin = {-76, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max1 annotation(
    Placement(visible = true, transformation(origin = {-54, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression minFcVoltage(y = 10) annotation(
    Placement(visible = true, transformation(origin = {-114, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression minBattVoltage(y = 40) annotation(
    Placement(visible = true, transformation(origin = {-110, -104}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  
  if time >= 50 then
    slewRateLimiter.u = switch1.y;
    controlInterface = max(abs1.y, 2500);
  else
    slewRateLimiter.u = 0;
    controlInterface = 0;
  end if;
  connect(hysteresis.y, switch1.u2) annotation(
    Line(points = {{-58, 0}, {6, 0}}, color = {255, 0, 255}));
  connect(shut_down.y, switch1.u1) annotation(
    Line(points = {{-58, 40}, {-40, 40}, {-40, 8}, {6, 8}}, color = {0, 0, 127}));
  connect(sensorSOC, hysteresis.u) annotation(
    Line(points = {{-120, 0}, {-82, 0}, {-82, 0}, {-82, 0}}, color = {0, 0, 127}));
  connect(setFuelCellCurrent.y, multiProduct.u[1]) annotation(
    Line(points = {{-78, -30}, {-70, -30}, {-70, -38}, {-52, -38}}, color = {0, 0, 127}));
  connect(multiProduct.y, division.u1) annotation(
    Line(points = {{-38, -38}, {-32, -38}}, color = {0, 0, 127}));
  connect(max2.y, multiProduct.u[2]) annotation(
    Line(points = {{-64, 86}, {-56, 86}, {-56, -38}, {-52, -38}}, color = {0, 0, 127}));
  connect(sensorFCVoltage, max2.u2) annotation(
    Line(points = {{-120, 80}, {-88, 80}}, color = {0, 0, 127}));
  connect(minFcVoltage.y, max2.u1) annotation(
    Line(points = {{-102, 110}, {-98, 110}, {-98, 92}, {-88, 92}}, color = {0, 0, 127}));
  connect(max1.y, division.u2) annotation(
    Line(points = {{-42, -86}, {-38, -86}, {-38, -50}, {-32, -50}}, color = {0, 0, 127}));
  connect(sensorBattVoltage, max1.u1) annotation(
    Line(points = {{-120, -80}, {-66, -80}}, color = {0, 0, 127}));
  connect(minBattVoltage.y, max1.u2) annotation(
    Line(points = {{-98, -104}, {-78, -104}, {-78, -92}, {-66, -92}}, color = {0, 0, 127}));
  connect(setFuelCellCurrent.y, switch1.u3) annotation(
    Line(points = {{-78, -30}, {-62, -30}, {-62, -20}, {-8, -20}, {-8, -8}, {6, -8}}, color = {0, 0, 127}));
  connect(slewRateLimiter.y, abs1.u) annotation(
    Line(points = {{62, 0}, {68, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {50, 50, 50}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-8, 121}, textColor = {0, 0, 255}, extent = {{-54, 17}, {54, -17}}, textString = "%name")}, coordinateSystem(initialScale = 0.1, extent = {{-100, -100}, {100, 100}})),
    Documentation(info = "<html><head></head><body><div>The EnergyManagementSystem component is designed to manage the flow of power between the fuel cell stack, battery, vehicle load, and balance-of-plant load. It splits the load according to pre-defined energy management rules, which are implemented within the bounds of the battery management system and the fuel cell control unit.</div><div><br></div>This model implements a simple energy management algorithm for a hybrid fuel cell &amp; battery system. The model reads the state-of-charge (SOC) of the battery. If it is less than a lower threshold value, then a signal is sent to activate the fuel cell with a given electric current. The rate at which current can be demanded from the fuel cell is limited by a slew rate.&nbsp;</body></html>"));
end EMS_RB;

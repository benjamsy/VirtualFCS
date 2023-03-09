within VirtualFCS.Control;

block EMS_CDCS "Implement algorithms to control the energy and power distribution in a hybrid system."
  // Based on: https://www.sciencedirect.com/science/article/pii/S0360319912016515?via%3Dihub
  //---- Parameters ----
  parameter Real ramp_up(unit = "1/s") = 20 "FC stack current ramp up rate" annotation(
    Dialog(group = "Control Parameters"));
  parameter Real V_HV_Bus(unit = "V") = "Voltage of the HV Bus" annotation(
    Dialog(group = "Powertrain Parameters"));
  //---- Fuel cell ----
  parameter Real I_min_FC_stack(unit = "A") "FC stack minimum operating current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter Real I_nom_FC_stack(unit = "A") "FC stack nominal operating current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter Real I_max_FC_stack(unit = "A") "FC stack maximum current" annotation(
    Dialog(group = "Powertrain Parameters"));
  //---- Battery ----
  parameter Real SOC_b(unit = "1") = 0.4 "SOC threshold of battery" annotation(
    Dialog(group = "Control Parameters"));
  //---- Control parameters ----
  parameter Real k(unit = "1") = 5000 "Coefficient" annotation(
    Dialog(group = "Control Parameters"));   // Coefficient to control for SOC under / over the benchmark.
  parameter Real I_max_batt(unit = "A") = 500 "Battery max current" annotation(
    Dialog(group = "Powertrain Parameters"));
  Real SOC(unit = "1") "State of charge of battery";
  final Real currentD(start = 0, unit = "A") "Current request";
  //---- Set current variable ----
  final Real set_current(start = 0) "Fuel cell set current";
  //---- Names ----
  //---- Connections ----
  Modelica.Blocks.Interfaces.RealInput sensorInputSOC annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput controlOutputFuelCellCurrent annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = ramp_up) annotation(
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Abs abs1 annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression setCurrent(y = set_current) annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput currentRequest annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  //---- Redefine variables ----
  SOC = sensorInputSOC;
  currentD = currentRequest;
  //---- Control Algorithm ----
  if SOC > (SOC_b + 0.05)then
    set_current = 0;
  else
    if (currentD - k*(SOC - SOC_b)) > I_max_FC_stack then
      set_current = I_max_FC_stack;
    elseif (currentD - k*(SOC - SOC_b)) < I_min_FC_stack then
      set_current = 0;
    else
      set_current = (currentD - k*(SOC - SOC_b));
    end if;
  end if;
//---- Connections ----
  connect(setCurrent.y, slewRateLimiter.u) annotation(
    Line(points = {{-38, 0}, {-22, 0}}, color = {0, 0, 127}));
  connect(slewRateLimiter.y, abs1.u) annotation(
    Line(points = {{2, 0}, {38, 0}}, color = {0, 0, 127}));
  connect(abs1.y, controlOutputFuelCellCurrent) annotation(
    Line(points = {{62, 0}, {110, 0}}, color = {0, 0, 127}));
  annotation( //Update documentation. 
    Icon(graphics = {Rectangle( fillColor = {0, 70, 40}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{-100, 100}, {100, -100}}, radius = 35), Rectangle(fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-43.2, 41.4}, {43, -45}}, radius = 8), Rectangle(origin = {-35, -3}, fillColor = {255, 0, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-22.2, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-35, -3}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, lineThickness = 0, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-9.5, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {3.3, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {16.1, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {28.85, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {0, 30}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, 17.2}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, 4.35}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -8.45}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -21.25}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -34.05}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(fillColor = {0, 70, 40}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{39.8, 38.3}, {-40, -41.8}}, radius = 5), Rectangle( fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-27.3, 25.5}, {27, -29}}), Rectangle(fillColor = {0, 70, 40}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-24.1, 22.25}, {23.85, -25.8}})}), 
    Documentation(info = "<html><head></head><body><div>The EnergyManagementSystem component is designed to manage the flow of power between the fuel cell stack, battery, vehicle load, and balance-of-plant load. It splits the load according to pre-defined energy management rules, which are implemented within the bounds of the battery management system and the fuel cell control unit.</div><div><br></div>This model implements a simple energy management algorithm for a hybrid fuel cell &amp; battery system. The model reads the state-of-charge (SOC) of the battery. If it is less than a lower threshold value, then a signal is sent to activate the fuel cell with a given electric current. The rate at which current can be demanded from the fuel cell is limited by a slew rate.&nbsp;</body></html>"));
end EMS_CDCS;

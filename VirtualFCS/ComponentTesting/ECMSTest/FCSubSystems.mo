within VirtualFCS.ComponentTesting.ECMSTest;

model FCSubSystems
  // System
  outer Modelica.Fluid.System system "System properties";
  // Medium decleration
  replaceable package Cathode_Medium = Modelica.Media.Air.MoistAir;
  replaceable package Anode_Medium = Modelica.Media.IdealGases.SingleGases.H2 constrainedby Modelica.Media.Interfaces.PartialSimpleIdealGasMedium;
  replaceable package Coolant_Medium = Modelica.Media.Water.ConstantPropertyLiquidWater constrainedby Modelica.Media.Interfaces.PartialMedium;
  // H2 Subsystem Paramters
  parameter Real m_FC_subsystems(unit = "kg") = 20;
  parameter Real V_tank_H2(unit = "m3") = 0.13 "H2 tank volume";
  parameter Real p_tank_H2(unit = "Pa") = 35000000 "H2 tank initial pressure";
  Modelica.Fluid.Interfaces.FluidPort_a H2_port_a(redeclare package Medium = Anode_Medium) annotation(
    Placement(visible = true, transformation(origin = {-66, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-70, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b H2_port_b(redeclare package Medium = Anode_Medium) annotation(
    Placement(visible = true, transformation(origin = {-54, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput contolInput[2] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a air_port_a(redeclare package Medium = Cathode_Medium) annotation(
    Placement(visible = true, transformation(origin = {52, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {70, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b air_port_b(redeclare package Medium = Cathode_Medium) annotation(
    Placement(visible = true, transformation(origin = {66, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a coolant_port_a(redeclare package Medium = Coolant_Medium) annotation(
    Placement(visible = true, transformation(origin = {-6, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b coolant_port_b(redeclare package Medium = Coolant_Medium) annotation(
    Placement(visible = true, transformation(origin = {6, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression realExpression(y = 0.003)  annotation(
    Placement(visible = true, transformation(origin = {-36, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Fluid.Sources.MassFlowSource_T boundary(redeclare package Medium = Anode_Medium, nPorts = 1, use_m_flow_in = true)  annotation(
    Placement(visible = true, transformation(origin = {-54, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Fluid.Sources.FixedBoundary boundary1(redeclare package Medium = Anode_Medium, nPorts = 1)  annotation(
    Placement(visible = true, transformation(origin = {-98, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.MassFlowSource_T massFlowSource_T(redeclare package Medium = Coolant_Medium, nPorts = 1, use_m_flow_in = true) annotation(
    Placement(visible = true, transformation(origin = {6, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression realExpression1(y = 1) annotation(
    Placement(visible = true, transformation(origin = {34, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Fluid.Sources.FixedBoundary fixedBoundary(redeclare package Medium = Coolant_Medium, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-22, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.MassFlowSource_T massFlowSource_T1(redeclare package Medium = Cathode_Medium, nPorts = 1, use_m_flow_in = true) annotation(
    Placement(visible = true, transformation(origin = {68, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression realExpression2(y = 1) annotation(
    Placement(visible = true, transformation(origin = {104, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Fluid.Sources.FixedBoundary fixedBoundary1(redeclare package Medium = Cathode_Medium, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {32, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(H2_port_b, boundary.ports[1]) annotation(
    Line(points = {{-54, 60}, {-54, -26}}));
  connect(realExpression.y, boundary.m_flow_in) annotation(
    Line(points = {{-46, -64}, {-62, -64}, {-62, -46}}, color = {0, 0, 127}));
  connect(boundary1.ports[1], H2_port_a) annotation(
    Line(points = {{-88, 48}, {-66, 48}, {-66, 60}}, color = {0, 127, 255}));
  connect(massFlowSource_T.ports[1], coolant_port_b) annotation(
    Line(points = {{6, -4}, {6, 60}}, color = {0, 127, 255}));
  connect(realExpression1.y, massFlowSource_T.m_flow_in) annotation(
    Line(points = {{24, -54}, {-2, -54}, {-2, -24}}, color = {0, 0, 127}));
  connect(fixedBoundary.ports[1], coolant_port_a) annotation(
    Line(points = {{-12, 34}, {-6, 34}, {-6, 60}}, color = {0, 127, 255}));
  connect(realExpression2.y, massFlowSource_T1.m_flow_in) annotation(
    Line(points = {{94, -24}, {60, -24}, {60, -4}}, color = {0, 0, 127}));
  connect(massFlowSource_T1.ports[1], air_port_b) annotation(
    Line(points = {{68, 16}, {66, 16}, {66, 60}}, color = {0, 127, 255}));
  connect(fixedBoundary1.ports[1], air_port_a) annotation(
    Line(points = {{42, 26}, {52, 26}, {52, 60}}, color = {0, 127, 255}));
  annotation(
    Icon(graphics = {Text(origin = {-31, -100}, textColor = {0, 0, 255}, extent = {{-105, 8}, {167, -40}}, textString = "%name"), Rectangle(extent = {{-100, 100}, {100, -100}}), Rectangle(fillColor = {0, 60, 101}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{-100, 100}, {100, -100}}, radius = 35), Text(origin = {-82, 60}, textColor = {255, 255, 255}, extent = {{-22, 10}, {22, -10}}, textString = "H2"), Text(origin = {-2, 60}, textColor = {255, 255, 255}, extent = {{-22, 10}, {22, -10}}, textString = "Cool"), Text(origin = {80, 60}, textColor = {255, 255, 255}, extent = {{-22, 10}, {22, -10}}, textString = "Air")}, coordinateSystem(initialScale = 0.1)));
end FCSubSystems;

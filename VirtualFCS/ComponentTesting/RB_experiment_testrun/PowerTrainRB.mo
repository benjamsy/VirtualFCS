within VirtualFCS.ComponentTesting.RB_experiment_testrun;

model PowerTrainRB
  // System
  outer Modelica.Fluid.System system "System properties";
  // Powertraom parameters
  parameter Real m_powertrain(unit = "kg") = fCSystem.m_FC_system + batterySystem.m_bat_pack;
  parameter Real V_HV_Bus(unit = "V") = 343 "Voltage of the HV Bus";
  // H2 Subsystem Paramters
  parameter Real V_tank_H2(unit = "m3") = 0.13 "H2 tank volume";
  parameter Real p_tank_H2(unit = "Pa") = 35000000 "H2 tank initial pressure";
  // Fuel Cell Stack Paramters
  parameter Real m_FC_stack(unit = "kg") = 14.3 "FC stack mass";
  parameter Real L_FC_stack(unit = "m") = 0.255 "FC stack length";
  parameter Real W_FC_stack(unit = "m") = 0.760 "FC stack width";
  parameter Real H_FC_stack(unit = "m") = 0.060 "FC stack height";
  parameter Real vol_FC_stack(unit = "m3") = L_FC_stack*W_FC_stack*H_FC_stack "FC stack volume";
  parameter Real V_rated_FC_stack(unit = "V") = 57.9 "FC stack maximum operating voltage";
  parameter Real I_rated_FC_stack(unit = "A") = 300 "FC stack minimum operating current";
  parameter Real i_L_FC_stack(unit = "A") = 1.7*I_rated_FC_stack "FC stack limiting current (max)";
  parameter Real I_nom_FC_stack(unit = "A") = 0.25*I_rated_FC_stack "FC stack nominal current";
  parameter Real N_FC_stack(unit = "1") = floor(V_rated_FC_stack/0.6433) "FC stack number of cells";
  // Battery Pack Parameters
  parameter Real m_bat_pack(unit = "kg") = 100 "Mass of the pack";
  parameter Real L_bat_pack(unit = "m") = 0.6 "Battery pack length";
  parameter Real W_bat_pack(unit = "m") = 0.45 "Battery pack width";
  parameter Real H_bat_pack(unit = "m") = 0.1 "Battery pack height";
  parameter Real Cp_bat_pack(unit = "J/(kg.K)") = 1000 "Specific Heat Capacity";
  parameter Real V_min_bat_pack(unit = "V") = 240 "Battery pack minimum voltage";
  parameter Real V_nom_bat_pack(unit = "V") = 336 "Battery pack nominal voltage";
  parameter Real V_max_bat_pack(unit = "V") = 403.2 "Battery pack maximum voltage";
  parameter Real C_bat_pack(unit = "A.h") = 200 "Battery pack nominal capacity";
  parameter Real SOC_init = 0.5 "Battery pack initial state of charge";
  
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(
    Placement(visible = true, transformation(origin = {40, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
    Placement(visible = true, transformation(origin = {-40, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrochemical.Battery.BatterySystem batterySystem(C_bat_pack = C_bat_pack, Cp_bat_pack = Cp_bat_pack, H_bat_pack = H_bat_pack, L_bat_pack = L_bat_pack, SOC_init = SOC_init, V_max_bat_pack = V_max_bat_pack, V_min_bat_pack = V_min_bat_pack, V_nom_bat_pack = V_nom_bat_pack, W_bat_pack = W_bat_pack) annotation(
    Placement(visible = true, transformation(origin = {-28, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VirtualFCS.Electrical.DCConverter converter(vDCref = V_HV_Bus) annotation(
    Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VirtualFCS.Electrical.DC_converter dC_converter annotation(
    Placement(visible = true, transformation(origin = {22, -44}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
  // Power & Efficiencies
  Real Power_del_DC_DC(unit = "W") "Power delivered from the DC/DC converter";
  Real Power_FC(unit = "W") "Power delivered from the FC system";
  Real Power_batt(unit = "W") "Power delivered from the batt system";
  Real Power_FC_batt(unit = "W") "Power from FC and Batt";
  Real eta_drivetrain(unit = "100") "Efficiency of the drivetrain";
  Real eta_DC_DC(unit = "100") = 100 "Efficiency of the DC/DC converter";
  VirtualFCS.ComponentTesting.ECMSTest.FCSystem fCSystem(I_rated_FC_stack = I_rated_FC_stack, N_FC_stack = N_FC_stack, V_tank_H2 = V_tank_H2, i_L_FC_stack = i_L_FC_stack, p_tank_H2 = p_tank_H2) annotation(
    Placement(visible = true, transformation(origin = {70, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MultiProduct multiProduct(nu = 2) annotation(
    Placement(visible = true, transformation(origin = {58, -12}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  Modelica.Blocks.Sources.RealExpression batt_V(y = batterySystem.pin_p.v) annotation(
    Placement(visible = true, transformation(origin = {146, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.RealExpression fc_V(y = fCSystem.pin_p.v) annotation(
    Placement(visible = true, transformation(origin = {146, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {104, -16}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  VirtualFCS.ECMS_experiment.EMS_RB ems_rb(SOC_min = 0.4)  annotation(
    Placement(visible = true, transformation(origin = {-63, -27}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getFCVoltage(y = fCSystem.pin_p.v)  annotation(
    Placement(visible = true, transformation(origin = {-122, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression getBattVoltage(y = batterySystem.pin_p.v)  annotation(
    Placement(visible = true, transformation(origin = {-122, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  
  Power_del_DC_DC = converter.dc_p1.i*converter.dc_p1.v;
  Power_FC = fCSystem.pin_n.i*fCSystem.pin_p.v;
  Power_batt = batterySystem.pin_n.i*batterySystem.pin_p.v;
  Power_FC_batt = Power_batt + Power_FC;
  if Power_del_DC_DC > 0 then
    eta_drivetrain = min(max(((Power_del_DC_DC)/max((Power_FC + Power_batt + ((Power_FC*(1 - (fCSystem.eta_FC_sys*0.01))) + (Power_batt*(1 - (batterySystem.eta_batt*0.01))) + (Power_del_DC_DC*(1 - (eta_DC_DC*0.01))))), 0.000001))*100, 0), 100);
  else
    eta_drivetrain = min(max(((-Power_del_DC_DC)/max((Power_FC - Power_batt + ((Power_FC*(1 - (fCSystem.eta_FC_sys*0.01))) - (Power_batt*(1 - (batterySystem.eta_batt*0.01))) - (Power_del_DC_DC*(1 - (eta_DC_DC*0.01))))), 0.000001))*100, 0), 100);
  end if;
  connect(pin_n, ground.p) annotation(
    Line(points = {{-40, 96}, {-70, 96}, {-70, 80}}, color = {0, 0, 255}));
  connect(converter.dc_n2, batterySystem.pin_n) annotation(
    Line(points = {{-6, 20}, {-6, -22}, {-32, -22}, {-32, -62}}, color = {0, 0, 255}));
  connect(converter.dc_p2, batterySystem.pin_p) annotation(
    Line(points = {{6, 20}, {6, -36}, {-24, -36}, {-24, -62}}, color = {0, 0, 255}));
  connect(converter.dc_n1, pin_n) annotation(
    Line(points = {{-6, 40}, {-38, 40}, {-38, 96}, {-40, 96}}, color = {0, 0, 255}));
  connect(converter.dc_p1, pin_p) annotation(
    Line(points = {{6, 40}, {6, 80}, {40, 80}, {40, 96}}, color = {0, 0, 255}));
  connect(dC_converter.pin_nFC, batterySystem.pin_n) annotation(
    Line(points = {{12, -54}, {-32, -54}, {-32, -62}, {-32, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_pFC, batterySystem.pin_p) annotation(
    Line(points = {{12, -34}, {-24, -34}, {-24, -62}, {-24, -62}}, color = {0, 0, 255}));
  connect(dC_converter.pin_nBus, fCSystem.pin_n) annotation(
    Line(points = {{32, -54}, {66, -54}, {66, -62}}, color = {0, 0, 255}));
  connect(fCSystem.pin_p, dC_converter.pin_pBus) annotation(
    Line(points = {{76, -62}, {76, -34}, {32, -34}}, color = {0, 0, 255}));
  connect(division.y, multiProduct.u[1]) annotation(
    Line(points = {{94, -16}, {64, -16}, {64, -12}}, color = {0, 0, 127}));
  connect(batt_V.y, division.u1) annotation(
    Line(points = {{136, -2}, {128, -2}, {128, -10}, {116, -10}}, color = {0, 0, 127}));
  connect(fc_V.y, division.u2) annotation(
    Line(points = {{136, -30}, {128, -30}, {128, -22}, {116, -22}}, color = {0, 0, 127}));
  connect(multiProduct.y, dC_converter.I_Ref) annotation(
    Line(points = {{50, -12}, {22, -12}, {22, -32}}, color = {0, 0, 127}));
  connect(getFCVoltage.y, ems_rb.sensorFCVoltage) annotation(
    Line(points = {{-110, -6}, {-98, -6}, {-98, -14}, {-80, -14}}, color = {0, 0, 127}));
  connect(getBattVoltage.y, ems_rb.sensorBattVoltage) annotation(
    Line(points = {{-110, -58}, {-98, -58}, {-98, -38}, {-80, -38}}, color = {0, 0, 127}));
  connect(batterySystem.sensorOutput, ems_rb.sensorSOC) annotation(
    Line(points = {{-38, -72}, {-92, -72}, {-92, -28}, {-82, -28}}, color = {0, 0, 127}));
  connect(ems_rb.controlInterface, multiProduct.u[2]) annotation(
    Line(points = {{-46, -26}, {-40, -26}, {-40, 6}, {72, 6}, {72, -12}, {64, -12}}, color = {0, 0, 127}));
protected
  annotation(
    Icon(graphics = {Text(origin = {-4, -12}, textColor = {0, 0, 255}, extent = {{-150, 120}, {150, 150}}, textString = "%name"), Text(origin = {17, 123}, extent = {{3, 5}, {-3, -5}}, textString = "text"), Rectangle(fillColor = {0, 60, 101}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{-100, 100}, {100, -100}}, radius = 35), Polygon(fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-16.7, 56.9}, {19.3, 56.9}, {5, 11.8}, {28.4, 11.8}, {-18.7, -56.5}, {-20, -56}, {-5.3, -6}, {-28.5, -6}, {-16.7, 56.9}})}, coordinateSystem(initialScale = 0.1)));
end PowerTrainRB;

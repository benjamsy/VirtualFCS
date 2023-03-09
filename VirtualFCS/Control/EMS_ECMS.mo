within VirtualFCS.Control;

block EMS_ECMS

//---- Imports ----
  import OMCallPythonAdvanced;
  import OMCallPythonAdvanced.Py;
  import OMCallPythonAdvanced.Py.finalize;
  import OMCallPythonAdvanced.Py.initialize;
  import OMCallPythonAdvanced.Py.PythonHandle;
  import OMCallPythonAdvanced.Py.run;
  import OMCallPythonAdvanced.Py.nineRealArgumentsReturnReal;
  import Modelica.Utilities.Streams.print;
  //---- Parameters ----
  parameter Real ramp_up(unit = "1/s") = 20 "FC stack current ramp up rate" annotation(
    Dialog(group = "Control Parameters"));
  //---- Fuel cell ----
  parameter Real I_min_FC_stack(unit = "A") "FC stack minimum operating current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter Real I_nom_FC_stack(unit = "A") "FC stack nominal operating current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter Real I_max_FC_stack(unit = "A") "FC stack maximum current" annotation(
    Dialog(group = "Powertrain Parameters"));
  parameter Real n_cell(unit = "1") "Number of cells in fuel cell stack" annotation(
    Dialog(group = "Powertrain Parameters"));
  Real LHV_H2(unit = "Ws/g") = 119959.2 "Hydrogen lower heating value";
  final Real bop_power(unit = "W", start = 0) "Balance of plant power";
  Real R = 8.314 "Universal gas constant";
  Real F = 96485.3321 "Faradays number";
  Real p_0 = 100000;
  Real i_0_FC_stack(unit = "A") = 0.0091 "FC stack cell exchange current";
  Real i_x_FC_stack(unit = "A") = 0.001 "FC stack cell cross-over current";
  Real b_1_FC_stack(unit = "V/dec") = 0.0985 "FC stack cell Tafel slope";
  Real b_2_FC_stack(unit = "V/dec") = 0.0985 "FC stack cell trasport limitation factor";
  Real R_0_FC_stack(unit = "Ohm") = 0.00022 * n_cell "FC stack cell ohmic resistance";
  Real U_0(unit = "V") = 1.229 "Theoretical maximum voltage for a single cell";
  Real fc_V(unit = "V") "FC voltage calculated from FC current output of optimization";
  // Used to calculate test /and control parameters. Will not matter if removed.
  //---- Battery ----
  parameter Real SOC_min(unit = "1") = 0.4 "Minimum allowed SOC of battery" annotation(
    Dialog(group = "Control Parameters"));
  parameter Real SOC_max(unit = "1") = 0.7 "Maximum allowed SOC of battery" annotation(
    Dialog(group = "Control Parameters"));
  parameter Real I_min_batt(unit = "A") = 500 "Battery min current" annotation(
    Dialog(group = "Control Parameters"));
  parameter Real I_max_batt(unit = "A") = 500 "Battery max current" annotation(
    Dialog(group = "Control Parameters"));
  Real SOC(unit = "1") "State of charge of battery";
  Real batt_V(unit = "V") "Battery voltage real";
  Real batt_I(unit = "A") "Battery Current real";
  Real OCV_batt(unit = "V") "Open circuit voltage of the battery";
  Real internalResistance(unit = "Ohm") = 0.028 "Internal resistance of battery real time";
  Real batterySetCurrent(unit = "A") "Battery set current calculated by the optimization solver";
  // Use this value further if this assumed to be const.
  final Real internalResistance1(unit = "Ohm", start = 0.1) "Internal resistance of battery real time";
  // Use this value if IR is SOC dependent like in: https://doi.org/10.3390/en12112076  - Obtained from curve fitting (Polynomial)
  // -- Battery charging/discharging efficiencies --
  final Real eta_b_chr(unit = "1", start = 1) "Battery charge efficiency";
  final Real eta_b_dischr(unit = "1", start = 1) "Battery discharge efficiency";
  final Integer eta_b_chr_counter(start = 1) "Counts the number of charge efficiencies calculated";
  final Real eta_b_chr_sum(start = 1);
  final Real eta_b_chr_avg(start = 1) "Battery charge efficiency average";
  final Integer eta_b_dischr_counter(start = 1) "Counts the number of discharge efficiencies calculated";
  final Real eta_b_dischr_sum(start = 1);
  final Real eta_b_dischr_avg(start = 1) "Battery discharge efficiency average";
  //---- Time
  Real t = time;
  final Real t_comp(start = 0);
  parameter Integer t_const = 50 "FC should not start before this time" annotation(
    Dialog(group = "Control Parameters"));
  // Assumtion on start-up procedure. This is not modelled, so FC is just turned off this time.
  //---- Necessary for ECMS ----
  final Real sigma(start = 1);
  Real mu(unit = "1") = 0.6;
  //Real mu(unit = "1") = 3;
  final Real powerD(start = 0, unit = "W") "Power request";
  final Real currentD(start = 0, unit = "A") "Current request";
  final Real k_b_f(start = 1);
  final Real C_fc(start = 0);
  final Real C_batt(start = 0);
  final Real C_batt_2(start = 0); // C_batt*k_b
  final Real C(start = 0);
  final Real C_sum(start = 0);
  Modelica.Blocks.Math.ContinuousMean C_a(u = C); // Average total consumption
  Modelica.Blocks.Math.ContinuousMean C_a_fc(u = C_fc); // Average fc consumption
  Modelica.Blocks.Math.ContinuousMean C_a_batt(u = C_batt); // Average batt consumption
  final Real C_avg;
  final Real C_avg_fc;
  final Real C_avg_batt;
  //---- Set current variable ---- Three is necessary to ensure constraints are maintained
  final Real set_current_1(start = 0) "Fuel cell set current";
  final Real set_current_2(start = 0) "Fuel cell set current";
  final Real set_current_3(start = 0) "Fuel cell set current";
  final Boolean turn_off(start = false);
  // Boolean to control if the FC is on or off.
  //---- Python Handle ----
  Py.PythonHandle pyHandle = Py.PythonHandle() "Define Python Handle";
  //---- Python minimization solver program ----
  String pyProgram = "
def ECMS_main(powerDemand, SoC, empty, b_V, sig, bopP, fc_temp, pH2, pO2):
\t import numpy
\t import scipy
\t import scipy.optimize
\t from scipy.optimize import Bounds
\t import math

\t import sys

\t print(sys.version)

\t bounds = Bounds([0, float(" + String(I_min_batt) + ")], [float(" + String(I_max_FC_stack - 1) + "), float(" + String(I_max_batt) + ")])

\t cons = {'type':'eq', 'fun' : lambda y : ((float(" + String(n_cell) + ") * (float(" + String(U_0) + ") - float(" + String(R) + ") * fc_temp / (2 * float(" + String(F) + ")) * math.log(1 / (pH2 / float(" + String(p_0) + ") * (pO2 / float(" + String(p_0) + "))**0.5)) - float(" + String(b_1_FC_stack) + ") * math.log10((y[0] + float(" + String(i_x_FC_stack) + ")) / float(" + String(i_0_FC_stack) + ")) + float(" + String(b_2_FC_stack) + ") * math.log10(1 - (y[0] + float(" + String(i_x_FC_stack) + ")) / float(" + String(I_max_FC_stack) + ")))) * y[0]) + (b_V * y[1]) - powerDemand}

\t res = scipy.optimize.minimize(lambda x : ((((float(" + String(n_cell) + ") * (float(" + String(U_0) + ") - float(" + String(R) + ") * fc_temp / (2 * float(" + String(F) + ")) * math.log(1 / (pH2 / float(" + String(p_0) + ") * (pO2 / float(" + String(p_0) + "))**0.5)) - float(" + String(b_1_FC_stack) + ") * math.log10((x[0] + float(" + String(i_x_FC_stack) + ")) / float(" + String(i_0_FC_stack) + ")) + float(" + String(b_2_FC_stack) + ") * math.log10(1 - (x[0] + float(" + String(i_x_FC_stack) + ")) / float(" + String(I_max_FC_stack) + ")))) * x[0]) / (float(" + String(LHV_H2) + ") * (max(((x[0] * (float(" + String(n_cell) + ") * (float(" + String(U_0) + ") - float(" + String(R) + ") * fc_temp / (2 * float(" + String(F) + ")) * math.log(1 / (pH2 / float(" + String(p_0) + ") * (pO2 / float(" + String(p_0) + "))**0.5)) - float(" + String(b_1_FC_stack) + ") * math.log10((x[0] + float(" + String(i_x_FC_stack) + ")) / float(" + String(i_0_FC_stack) + ")) + float(" + String(b_2_FC_stack) + ") * math.log10(1 - (x[0] + float(" + String(i_x_FC_stack) + ")) / float(" + String(I_max_FC_stack) + ")))) - bopP) / (float(" + String(LHV_H2) + ") * 2.016 * (float(" + String(n_cell) + ") * (x[0] + 0.000001) / (2 * float(" + String(F) + "))))),0.1)))) + (3 - 2*float(" + String(mu) + ")*(SoC - 0.5*(float(" + String(SOC_max) + ") + float(" + String(SOC_min) + ")))/(float(" + String(SOC_max) + ") - float(" + String(SOC_min) + "))) * ((float(sig) * float(b_V) * x[1]) / (float(" + String(LHV_H2) + ")))), numpy.array([0,0]), method='SLSQP', bounds=bounds, constraints=cons)

\t return float(res.x[0])

  ";
  //---- Names ----
  constant String pyModuleName = "__main__";
  constant String pyFunctionName = "ECMS_main";
  //---- Connections ----
  Modelica.Blocks.Interfaces.RealInput sensorInputSOC annotation(
    Placement(visible = true, transformation(origin = {-220, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-220, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput controlOutputFuelCellCurrent annotation(
    Placement(visible = true, transformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = ramp_up) annotation(
    Placement(visible = true, transformation(origin = {62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Abs abs1 annotation(
    Placement(visible = true, transformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression setCurrent(y = set_current_3) annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput powerRequest annotation(
    Placement(visible = true, transformation(origin = {-220, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-220, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput batteryVoltage annotation(
    Placement(visible = true, transformation(origin = {-100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput OCV annotation(
    Placement(visible = true, transformation(origin = {-100, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-142, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput batteryCurrent annotation(
    Placement(visible = true, transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput bopPower annotation(
    Placement(visible = true, transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-58, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput fcTemperature annotation(
    Placement(visible = true, transformation(origin = {100, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {60, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput p_H2 annotation(
    Placement(visible = true, transformation(origin = {200, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {140, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput p_O2 annotation(
    Placement(visible = true, transformation(origin = {100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {100, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Continuous.Filter filter(analogFilter = Modelica.Blocks.Types.AnalogFilter.CriticalDamping, f_cut = 0.7, order = 5)  annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Real battPowerV2;
  Real fcPowerV2;
  Real fcPowerAverage;
  Modelica.Blocks.Math.ContinuousMean P_fc_avg(u = fcPowerV2); // Average total consumption
  Real testTest;
  Real battFcPower;
  Real testConstrain;
  Real fc_eff;
  Real fc_efficiency_avg;
  Modelica.Blocks.Math.ContinuousMean etaFcSys(u = fc_eff);
initial equation
//---- Import necessary Python libraries ----
  Py.initialize(pyHandle);
  Py.run(pyHandle, "import sys");
  Py.run(pyHandle, "import os");
equation
//---- Redefine variables ----
  SOC = sensorInputSOC;
  batt_V = batteryVoltage;
  powerD = powerRequest;
  bop_power = bopPower;
  OCV_batt = OCV;
  batt_I = batteryCurrent;
  
//---- Recalculations ----
  fc_V = n_cell*(U_0 - R*fcTemperature/(2*F)*log(1/(p_H2/p_0*(p_O2/p_0)^0.5)) - b_1_FC_stack*log10((set_current_1 + i_x_FC_stack)/i_0_FC_stack) + b_2_FC_stack*log10(1 - (set_current_1 + i_x_FC_stack)/I_max_FC_stack)); // Calculates FC voltage from the set_current from optimization algorithm
  currentD = powerD/batt_V; // Calculates the current demand (on the battery side of converters)
  batterySetCurrent = currentD - (fcPowerV2/batt_V); // Calculates the "real" battery set current inside the optimization solver
  battPowerV2 = batterySetCurrent * batt_V; // Calculates the battery power assumed by the optimization solver, is not the same as real battery power
  fcPowerV2 = set_current_1 * fc_V; // Calculates the fuel cell set power decided by the optimization solver
  battFcPower = fcPowerV2 + battPowerV2; // Calculates the combined power from FC and batt inside the optimization solver
  testConstrain = (fc_V * set_current_1) + (batt_V * batterySetCurrent) - powerD; // Calculates the constrain inside the optimization solver (Should be close to zero)
//---- Consumptions ----
  C_fc = (((n_cell*(U_0 - R*fcTemperature/(2*F)*log(1/(p_H2/p_0*(p_O2/p_0)^0.5)) - b_1_FC_stack*log10((set_current_1 + i_x_FC_stack)/i_0_FC_stack) + b_2_FC_stack*log10(1 - (set_current_1 + i_x_FC_stack)/I_max_FC_stack)))*set_current_1)/(LHV_H2*(max(((set_current_1*(n_cell*(U_0 - R*fcTemperature/(2*F)*log(1/(p_H2/p_0*(p_O2/p_0)^0.5)) - b_1_FC_stack*log10((set_current_1 + i_x_FC_stack)/i_0_FC_stack) + b_2_FC_stack*log10(1 - (set_current_1 + i_x_FC_stack)/I_max_FC_stack))) - bop_power)/(LHV_H2*2.016*(n_cell*(set_current_1 + 0.000001)/(2*F)))), 0.1)))); // Calculates C_fc the same as the optimization solver. This will vary from the actual FC since set_current_1 is used, and no dynamic behavious is considered.
  C_batt = ((sigma*batt_V*batterySetCurrent)/(LHV_H2)); // Calculates C_batt same as the optimization solver. Will vary from actual batt.
  k_b_f = 3-2*mu*((SOC - 0.5*(SOC_max + SOC_min))/(SOC_max - SOC_min)); // Calculates the k_b term
  C_batt_2 = k_b_f*C_batt; // Calculates k_b * C_batt
  C = C_fc + k_b_f*C_batt; // Calcualtes C_tot
  C_avg = C_a.y; // Average total consumption
  C_avg_fc = C_a_fc.y; // Average fc consumption
  C_avg_batt = C_a_batt.y; // Average batt consumption
  fcPowerAverage = P_fc_avg.y;
  testTest = C_avg_fc/max(fcPowerAverage, 0.0000001);
  fc_eff = (max(((set_current_1 * fc_V - bop_power) / (LHV_H2 * 2.016 *(n_cell * (set_current_1 + 0.000001) / (2 * F)))),0.1)) * 100;
  fc_efficiency_avg = etaFcSys.y;
  
  
  if batt_I >= 0 then
// battery is discharging, from: https://doi.org/10.1016/j.conengprac.2011.06.008
    internalResistance1 = 0.3 - 0.2461*SOC + 0.1464*SOC^2 - 6.4437*SOC^3 + 68.8914*SOC^4 - 284.6643*SOC^5 + 605.996*SOC^6 - 706.5709*SOC^7 + 428.894*SOC^8 - 106.0429*SOC^9;
//Polynomial curve fitting if internalResistance1 is used.
    eta_b_dischr = 0.5*(1 + sqrt(1 - 4*internalResistance*batt_I*batt_V/OCV_batt^2));
    eta_b_chr = 1;
  elseif batt_I < 0 then
// battery is charging
    internalResistance1 = 0.265 - 0.175*SOC + 2.6391*SOC^2 - 34.6741*SOC^3 + 218.5401*SOC^4 - 737.1513*SOC^5 + 1421.8683*SOC^6 - 1572.5482*SOC^7 + 927.3865*SOC^8 - 225.8408*SOC^9;
//Polynomial curve fitting if internalResistance1 is used.
    eta_b_chr = 2/(1 + sqrt(1 - 4*internalResistance*batt_I*batt_V/OCV_batt^2));
    eta_b_dischr = 1;
  else
    internalResistance1 = 0.1;
    eta_b_chr = 1;
    eta_b_dischr = 1;
  end if;
  when t >= pre(t_comp) + 1 then
// Python program is only run 1 time each second to reduce runtime
    t_comp = pre(t_comp) + 1;
    if batt_I < 0 then
// Battery is Charging
      eta_b_chr_counter = pre(eta_b_chr_counter) + 1;
      eta_b_chr_sum = pre(eta_b_chr_sum) + eta_b_chr;
      eta_b_chr_avg = eta_b_chr_sum/eta_b_chr_counter;
      eta_b_dischr_counter = pre(eta_b_dischr_counter);
      eta_b_dischr_sum = pre(eta_b_dischr_sum);
      eta_b_dischr_avg = pre(eta_b_dischr_avg);
      sigma = eta_b_dischr_avg*eta_b_chr;
    else
// Battery is Discharging
      eta_b_dischr_counter = pre(eta_b_dischr_counter) + 1;
      eta_b_dischr_sum = pre(eta_b_dischr_sum) + eta_b_dischr;
      eta_b_dischr_avg = eta_b_dischr_sum/eta_b_dischr_counter;
      eta_b_chr_counter = pre(eta_b_chr_counter);
      eta_b_chr_sum = pre(eta_b_chr_sum);
      eta_b_chr_avg = pre(eta_b_chr_avg);
      sigma = 1/(eta_b_chr_avg*eta_b_dischr);
    end if;
    set_current_1 = Py.nineRealArgumentsReturnReal(pyHandle, powerD, SOC, 1, batt_V, sigma, bop_power, fcTemperature, p_H2, p_O2, pyProgram, pyModuleName, pyFunctionName); //Cost function is ran.
    C_sum = pre(C_sum) + C;
// ---- Control sequence to ensure safe operation. ----
    if SOC > SOC_max then
      turn_off = true;
    elseif SOC < (SOC_max + SOC_min)/2 then
      turn_off = false;
    else
      turn_off = pre(turn_off);
    end if;
    if turn_off == false then
      set_current_2 = max(set_current_1, I_min_FC_stack);
    else
      set_current_2 = 0;
    end if;
    if set_current_2 < I_min_FC_stack or t < t_const then
      set_current_3 = 0;
    else
      set_current_3 = set_current_2;
    end if;
  end when;
  connect(slewRateLimiter.y, abs1.u) annotation(
    Line(points = {{74, 0}, {90, 0}}, color = {0, 0, 127}));
  connect(abs1.y, controlOutputFuelCellCurrent) annotation(
    Line(points = {{114, 0}, {210, 0}}, color = {0, 0, 127}));
  connect(setCurrent.y, filter.u) annotation(
    Line(points = {{-38, 0}, {-12, 0}}, color = {0, 0, 127}));
  connect(filter.y, slewRateLimiter.u) annotation(
    Line(points = {{12, 0}, {50, 0}}, color = {0, 0, 127}));
  annotation( //Update documentation. 
    Icon(graphics = {Rectangle(fillColor = {0, 70, 40}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{-200, 100}, {200, -100}}, radius = 35), Rectangle(fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-43.2, 41.4}, {43, -45}}, radius = 8), Rectangle(origin = {-35, -3}, fillColor = {255, 0, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-22.2, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-35, -3}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, lineThickness = 0, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {-9.5, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {3.3, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {16.1, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {28.85, -3}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{1.4, 58.9}, {4.5, -56.5}}), Rectangle(origin = {0, 30}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, 17.2}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, 4.35}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -8.45}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -21.25}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(origin = {0, -34.05}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-57.7, 1.85}, {57.35, -1.3}}), Rectangle(fillColor = {0, 70, 40}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{39.8, 38.3}, {-40, -41.8}}, radius = 5), Rectangle( fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-27.3, 25.5}, {27, -29}}), Rectangle(fillColor = {0, 70, 40}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-24.1, 22.25}, {23.85, -25.8}})}, coordinateSystem(extent = {{-200, -100}, {200, 100}}, initialScale = 0.1)), Diagram(coordinateSystem(extent = {{-200, -100}, {200, 100}}, initialScale = 0.1)),
    Documentation(info = "<html><head></head><body><div>The EnergyManagementSystem component is designed to manage the flow of power between the fuel cell stack, battery, vehicle load, and balance-of-plant load. It splits the load according to pre-defined energy management rules, which are implemented within the bounds of the battery management system and the fuel cell control unit.</div><div><br></div></body></html>"));
end EMS_ECMS;

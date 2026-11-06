# UAV Flight Model

MATLAB flight models and structural design codes for a fixed-wing electric UAV.
Mass 6.89 kg, wing span 1.68 m, 6S 4200 mAh battery.

## flight_model
Mission: take-off, climb, cruise, loiter, cruise, descent, landing.

| Script | Model |
| --- | --- |
| FM1_PointMass.m | Point mass |
| FM2_3DOF.m | 3-DOF longitudinal |
| FM3_6DOF.m | 6-DOF |

Keep all files in one folder and run a script from there. Aircraft data is in
Flight_Model_Wing_Detail.m. Figures and video are saved in FM1_Results,
FM2_Results and FM3_Results.

## structural_design
| Script | Purpose |
| --- | --- |
| WORK_WING.m | Wing shear flow, buckling and stress |
| WORK_FUSELAGE.m | Fuselage loads, longerons and bulkheads |
| WORK_LG.m | Main landing gear leg |

Tested on MATLAB R2026a.
| WORK_LG.m | Main landing gear leg |

Tested on MATLAB R2026a.
| WORK_LG.m | Main landing gear leg |

Tested on MATLAB 2026a

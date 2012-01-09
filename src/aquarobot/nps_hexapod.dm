# DynaMechs V 2.0.3 ascii
# hexapod from EOD sims sans the arm

Graphics_Models {
	Number_Graphics_Models  5
        "nps_torso.scm"
        "nps_leglink1.scm"
        "nps_leglink2.scm"
        "nps_leglink3.scm"
        "nps_footpad.xan"
}

System {
    DynamicRefMember {
	Graphics_Model_Index	 0

	Mass                     210.28
	Inertia                  14.7   0.0  0.0
		                  0.0  14.7  0.0
		                  0.0   0.0  8.72
	Center_of_Gravity        0.0 0.0 0.0
	Number_of_Contact_Points	4
	Contact_Locations	 0.0 0.0 -0.5
				 0.0 0.0 -0.5
				 0.0 0.0 -0.5
				 0.0 0.0 -0.5

	Position                 8.0  10.0  3.0
	Orientation_Quat         0.0  0.0  0.0  1.0    # (x,y,z w)

	Velocity                 0.0 0.0 0.0 0.0 0.0 0.0
    }

    Articulation {
	RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass                     5.03
	    Inertia                  0.02   0.0    0.0
			             0.0    0.07   0.0
			             0.0    0.0    0.13
	    Center_of_Gravity        0.10 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.375 0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.0472 1.0472
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     2

	    Mass                     32.61
	    Inertia                  0.22  -0.39   0.0
				    -0.39   3.6    0.0
	                             0.0    0.0    3.7
	    Center_of_Gravity        0.2 0.06 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.20 1.5708 0.0 0.1
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.86013 1.2815
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     3

	    Mass                     35.81
	    Inertia                  0.18   0.0    0.0
				     0.0    4.0    0.0
	                             0.0    0.0    4.0
	    Center_of_Gravity        0.3 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.52 0.0 0.0 -1.5
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-2.730086 0.411507
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	SphericalLink {
	    Graphics_Model_Index     4

	    Mass                     10.14
	    Inertia                  0.164 0.0    0.0
				     0.0   0.084  0.0
	                             0.0   0.0    0.084
	    Center_of_Gravity        0.015 0.0 0.0

	    Number_of_Contact_Points	 9
	    Contact_Locations        0.03  0.0       0.225
	                             0.03  0.159099  0.159099
	                             0.03  0.225     0.0
	                             0.03  0.159099 -0.159099
	                             0.03  0.0      -0.225
	                             0.03 -0.159099 -0.159099
	                             0.03 -0.225     0.0
	                             0.03 -0.159099  0.159099
	                             0.03  0.0       0.0

	    Position_From_Inboard_Link   1.02 0.0 0.0
	    Initial_Joint_Angles         0.0 0.0 -0.47070
	    Initial_Angular_Velocity     0.0 0.0 0.0
	    Axes_Limits                  0.5  0.0  0.0
	    Joint_Limit_Spring_Constant  800.0
	    Joint_Limit_Damper_Constant  10.0
	    Joint_Friction		 0.35
	}
    }

    Articulation {
	    ZScrewTxLink {
		ZScrew_Parameters	 0.0 1.0471976
	    }

	    RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass                     5.03
	    Inertia                  0.02   0.0    0.0
			             0.0    0.07   0.0
			             0.0    0.0    0.13
	    Center_of_Gravity        0.10 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.375 0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.0472 1.0472
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     2

	    Mass                     32.61
	    Inertia                  0.22  -0.39   0.0
				    -0.39   3.6    0.0
	                             0.0    0.0    3.7
	    Center_of_Gravity        0.2 0.06 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.20 1.5708 0.0 0.1
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.86013 1.2815
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     3

	    Mass                     35.81
	    Inertia                  0.18   0.0    0.0
				     0.0    4.0    0.0
	                             0.0    0.0    4.0
	    Center_of_Gravity        0.3 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.52 0.0 0.0 -1.5
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-2.730086 0.411507
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	SphericalLink {
	    Graphics_Model_Index     4

	    Mass                     10.14
	    Inertia                  0.164 0.0    0.0
				     0.0   0.084  0.0
	                             0.0   0.0    0.084
	    Center_of_Gravity        0.015 0.0 0.0

	    Number_of_Contact_Points	 9
	    Contact_Locations        0.03  0.0       0.225
	                             0.03  0.159099  0.159099
	                             0.03  0.225     0.0
	                             0.03  0.159099 -0.159099
	                             0.03  0.0      -0.225
	                             0.03 -0.159099 -0.159099
	                             0.03 -0.225     0.0
	                             0.03 -0.159099  0.159099
	                             0.03  0.0       0.0

	    Position_From_Inboard_Link   1.02 0.0 0.0
	    Initial_Joint_Angles         0.0 0.0 -0.47070
	    Initial_Angular_Velocity     0.0 0.0 0.0
	    Axes_Limits                  0.5  0.0  0.0
	    Joint_Limit_Spring_Constant  800.0
	    Joint_Limit_Damper_Constant  10.0
	    Joint_Friction		 0.35
	}
    }

    Articulation {
	    ZScrewTxLink {
		ZScrew_Parameters	 0.0 2.0943951
	    }

	    RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass                     5.03
	    Inertia                  0.02   0.0    0.0
			             0.0    0.07   0.0
			             0.0    0.0    0.13
	    Center_of_Gravity        0.10 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.375 0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.0472 1.0472
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     2

	    Mass                     32.61
	    Inertia                  0.22  -0.39   0.0
				    -0.39   3.6    0.0
	                             0.0    0.0    3.7
	    Center_of_Gravity        0.2 0.06 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.20 1.5708 0.0 0.1
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.86013 1.2815
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     3

	    Mass                     35.81
	    Inertia                  0.18   0.0    0.0
				     0.0    4.0    0.0
	                             0.0    0.0    4.0
	    Center_of_Gravity        0.3 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.52 0.0 0.0 -1.5
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-2.730086 0.411507
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	SphericalLink {
	    Graphics_Model_Index     4

	    Mass                     10.14
	    Inertia                  0.164 0.0    0.0
				     0.0   0.084  0.0
	                             0.0   0.0    0.084
	    Center_of_Gravity        0.015 0.0 0.0

	    Number_of_Contact_Points	 9
	    Contact_Locations        0.03  0.0       0.225
	                             0.03  0.159099  0.159099
	                             0.03  0.225     0.0
	                             0.03  0.159099 -0.159099
	                             0.03  0.0      -0.225
	                             0.03 -0.159099 -0.159099
	                             0.03 -0.225     0.0
	                             0.03 -0.159099  0.159099
	                             0.03  0.0       0.0

	    Position_From_Inboard_Link   1.02 0.0 0.0
	    Initial_Joint_Angles         0.0 0.0 -0.47070
	    Initial_Angular_Velocity     0.0 0.0 0.0
	    Axes_Limits                  0.5  0.0  0.0
	    Joint_Limit_Spring_Constant  800.0
	    Joint_Limit_Damper_Constant  10.0
	    Joint_Friction		 0.35
	}
    }

    Articulation {
	    ZScrewTxLink {
		ZScrew_Parameters	 0.0 3.1415927
	    }

	    RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass                     5.03
	    Inertia                  0.02   0.0    0.0
			             0.0    0.07   0.0
			             0.0    0.0    0.13
	    Center_of_Gravity        0.10 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.375 0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.0472 1.0472
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     2

	    Mass                     32.61
	    Inertia                  0.22  -0.39   0.0
				    -0.39   3.6    0.0
	                             0.0    0.0    3.7
	    Center_of_Gravity        0.2 0.06 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.20 1.5708 0.0 0.1
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.86013 1.2815
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     3

	    Mass                     35.81
	    Inertia                  0.18   0.0    0.0
				     0.0    4.0    0.0
	                             0.0    0.0    4.0
	    Center_of_Gravity        0.3 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.52 0.0 0.0 -1.5
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-2.730086 0.411507
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	SphericalLink {
	    Graphics_Model_Index     4

	    Mass                     10.14
	    Inertia                  0.164 0.0    0.0
				     0.0   0.084  0.0
	                             0.0   0.0    0.084
	    Center_of_Gravity        0.015 0.0 0.0

	    Number_of_Contact_Points	 9
	    Contact_Locations        0.03  0.0       0.225
	                             0.03  0.159099  0.159099
	                             0.03  0.225     0.0
	                             0.03  0.159099 -0.159099
	                             0.03  0.0      -0.225
	                             0.03 -0.159099 -0.159099
	                             0.03 -0.225     0.0
	                             0.03 -0.159099  0.159099
	                             0.03  0.0       0.0

	    Position_From_Inboard_Link   1.02 0.0 0.0
	    Initial_Joint_Angles         0.0 0.0 -0.47070
	    Initial_Angular_Velocity     0.0 0.0 0.0
	    Axes_Limits                  0.5  0.0  0.0
	    Joint_Limit_Spring_Constant  800.0
	    Joint_Limit_Damper_Constant  10.0
	    Joint_Friction		 0.35
	}
    }

    Articulation {
	    ZScrewTxLink {
		ZScrew_Parameters	 0.0 -2.0943951
	    }

	    RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass                     5.03
	    Inertia                  0.02   0.0    0.0
			             0.0    0.07   0.0
			             0.0    0.0    0.13
	    Center_of_Gravity        0.10 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.375 0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.0472 1.0472
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     2

	    Mass                     32.61
	    Inertia                  0.22  -0.39   0.0
				    -0.39   3.6    0.0
	                             0.0    0.0    3.7
	    Center_of_Gravity        0.2 0.06 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.20 1.5708 0.0 0.1
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.86013 1.2815
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     3

	    Mass                     35.81
	    Inertia                  0.18   0.0    0.0
				     0.0    4.0    0.0
	                             0.0    0.0    4.0
	    Center_of_Gravity        0.3 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.52 0.0 0.0 -1.5
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-2.730086 0.411507
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	SphericalLink {
	    Graphics_Model_Index     4

	    Mass                     10.14
	    Inertia                  0.164 0.0    0.0
				     0.0   0.084  0.0
	                             0.0   0.0    0.084
	    Center_of_Gravity        0.015 0.0 0.0

	    Number_of_Contact_Points	 9
	    Contact_Locations        0.03  0.0       0.225
	                             0.03  0.159099  0.159099
	                             0.03  0.225     0.0
	                             0.03  0.159099 -0.159099
	                             0.03  0.0      -0.225
	                             0.03 -0.159099 -0.159099
	                             0.03 -0.225     0.0
	                             0.03 -0.159099  0.159099
	                             0.03  0.0       0.0

	    Position_From_Inboard_Link   1.02 0.0 0.0
	    Initial_Joint_Angles         0.0 0.0 -0.47070
	    Initial_Angular_Velocity     0.0 0.0 0.0
	    Axes_Limits                  0.5  0.0  0.0
	    Joint_Limit_Spring_Constant  800.0
	    Joint_Limit_Damper_Constant  10.0
	    Joint_Friction		 0.35
	}
    }

    Articulation {
	    ZScrewTxLink {
		ZScrew_Parameters	 0.0 -1.0471976
	    }

	    RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass                     5.03
	    Inertia                  0.02   0.0    0.0
			             0.0    0.07   0.0
			             0.0    0.0    0.13
	    Center_of_Gravity        0.10 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.375 0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.0472 1.0472
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     2

	    Mass                     32.61
	    Inertia                  0.22  -0.39   0.0
				    -0.39   3.6    0.0
	                             0.0    0.0    3.7
	    Center_of_Gravity        0.2 0.06 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.20 1.5708 0.0 0.1
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1.86013 1.2815
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	RevoluteLink {
	    Graphics_Model_Index     3

	    Mass                     35.81
	    Inertia                  0.18   0.0    0.0
				     0.0    4.0    0.0
	                             0.0    0.0    4.0
	    Center_of_Gravity        0.3 0.0 0.0

	    Number_of_Contact_Points	 0

	    MDH_Parameters		 0.52 0.0 0.0 -1.5
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-2.730086 0.411507
	    Joint_Limit_Spring_Constant  5000.0
	    Joint_Limit_Damper_Constant  500.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}

	SphericalLink {
	    Graphics_Model_Index     4

	    Mass                     10.14
	    Inertia                  0.164 0.0    0.0
				     0.0   0.084  0.0
	                             0.0   0.0    0.084
	    Center_of_Gravity        0.015 0.0 0.0

	    Number_of_Contact_Points	 9
	    Contact_Locations        0.03  0.0       0.225
	                             0.03  0.159099  0.159099
	                             0.03  0.225     0.0
	                             0.03  0.159099 -0.159099
	                             0.03  0.0      -0.225
	                             0.03 -0.159099 -0.159099
	                             0.03 -0.225     0.0
	                             0.03 -0.159099  0.159099
	                             0.03  0.0       0.0

	    Position_From_Inboard_Link   1.02 0.0 0.0
	    Initial_Joint_Angles         0.0 0.0 -0.47070
	    Initial_Angular_Velocity     0.0 0.0 0.0
	    Axes_Limits                  0.5  0.0  0.0
	    Joint_Limit_Spring_Constant  800.0
	    Joint_Limit_Damper_Constant  10.0
	    Joint_Friction		 0.35
	}
    }
}

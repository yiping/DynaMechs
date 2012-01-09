# DynaMechs V 2.0.3 ascii
# tree parameter file

Graphics_Models {
	Number_Graphics_Models  2
	"../models/cart_body.xan"
	"../models/cart_wheel.xan"
}

System {
    DynamicRefMember {
	Graphics_Model_Index	 0

	Mass			 2.5
	Inertia			 2.2   0.0  0.0
				 0.0   1.1  0.0
				 0.0   0.0  2.2
	Center_of_Gravity	 0.0 0.0 0.0

	Number_of_Contact_Points	8 
	Contact_Locations	-1.0  0.25  0.9
				 1.0  0.25  0.9
				-1.0 -0.25  0.9
				 1.0 -0.25  0.9
				-1.0  0.25 -0.9
				 1.0  0.25 -0.9
				-1.0 -0.25 -0.9
				 1.0 -0.25 -0.9

	Position	  7.0   10.0   5.0
	Orientation_Quat  0.7071 0.0 0.0 0.7071
	Velocity	  0.0 0.0 0.0 0.0 0.0 0.0
    }

# wheel 1
    Articulation {
	ZScrewTxLink {
	    ZScrew_Parameters	 1.0 0.0
	}

	RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass			 0.2
	    Inertia			 0.3   0.0  0.0
					 0.0   0.3  0.0
					 0.0   0.0  0.8
	    Center_of_Gravity		 0.0 0.0 0.0

	    Number_of_Contact_Points	 16
	    Contact_Locations		 0.5     0.0    0.0
					 0.4619  0.1913 0.0
					 0.3535  0.3535 0.0
					 0.1913  0.4619 0.0
					 0.0     0.5    0.0
					-0.1913  0.4619 0.0
					-0.3535  0.3535 0.0
					-0.4619  0.1913 0.0
					-0.5     0.0    0.0
					-0.4619 -0.1913 0.0
					-0.3535 -0.3535 0.0
					-0.1913 -0.4619 0.0
					 0.0    -0.5    0.0
					 0.1913 -0.4619 0.0
					 0.3535 -0.3535 0.0
					 0.4619 -0.1913 0.0

	    MDH_Parameters		 0.9  0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1000.172 1000.172 
	    Joint_Limit_Spring_Constant  0.0
	    Joint_Limit_Damper_Constant  0.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}
    }

# wheel 2
    Articulation {
	ZScrewTxLink {
	    ZScrew_Parameters	 -1.0 0.0
	}

	RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass			 0.2
	    Inertia			 0.3   0.0  0.0
					 0.0   0.3  0.0
					 0.0   0.0  0.8
	    Center_of_Gravity		 0.0 0.0 0.0

	    Number_of_Contact_Points	 16
	    Contact_Locations		 0.5     0.0    0.0
					 0.4619  0.1913 0.0
					 0.3535  0.3535 0.0
					 0.1913  0.4619 0.0
					 0.0     0.5    0.0
					-0.1913  0.4619 0.0
					-0.3535  0.3535 0.0
					-0.4619  0.1913 0.0
					-0.5     0.0    0.0
					-0.4619 -0.1913 0.0
					-0.3535 -0.3535 0.0
					-0.1913 -0.4619 0.0
					 0.0    -0.5    0.0
					 0.1913 -0.4619 0.0
					 0.3535 -0.3535 0.0
					 0.4619 -0.1913 0.0

	    MDH_Parameters		 0.9  0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1000.172 1000.172 
	    Joint_Limit_Spring_Constant  0.0
	    Joint_Limit_Damper_Constant  0.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}
    }

# wheel 3
    Articulation {
	ZScrewTxLink {
	    ZScrew_Parameters	 1.0 0.0
	}

	RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass			 0.2
	    Inertia			 0.3   0.0  0.0
					 0.0   0.3  0.0
					 0.0   0.0  0.8
	    Center_of_Gravity		 0.0 0.0 0.0

	    Number_of_Contact_Points	 16
	    Contact_Locations		 0.5     0.0    0.0
					 0.4619  0.1913 0.0
					 0.3535  0.3535 0.0
					 0.1913  0.4619 0.0
					 0.0     0.5    0.0
					-0.1913  0.4619 0.0
					-0.3535  0.3535 0.0
					-0.4619  0.1913 0.0
					-0.5     0.0    0.0
					-0.4619 -0.1913 0.0
					-0.3535 -0.3535 0.0
					-0.1913 -0.4619 0.0
					 0.0    -0.5    0.0
					 0.1913 -0.4619 0.0
					 0.3535 -0.3535 0.0
					 0.4619 -0.1913 0.0

	    MDH_Parameters		-0.9  0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1000.172 1000.172 
	    Joint_Limit_Spring_Constant  0.0
	    Joint_Limit_Damper_Constant  0.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}
    }

# wheel 4
    Articulation {
	ZScrewTxLink {
	    ZScrew_Parameters	-1.0 0.0
	}

	RevoluteLink {
	    Graphics_Model_Index	 1

	    Mass			 0.2
	    Inertia			 0.3   0.0  0.0
					 0.0   0.3  0.0
					 0.0   0.0  0.8
	    Center_of_Gravity		 0.0 0.0 0.0

	    Number_of_Contact_Points	 16
	    Contact_Locations		 0.5     0.0    0.0
					 0.4619  0.1913 0.0
					 0.3535  0.3535 0.0
					 0.1913  0.4619 0.0
					 0.0     0.5    0.0
					-0.1913  0.4619 0.0
					-0.3535  0.3535 0.0
					-0.4619  0.1913 0.0
					-0.5     0.0    0.0
					-0.4619 -0.1913 0.0
					-0.3535 -0.3535 0.0
					-0.1913 -0.4619 0.0
					 0.0    -0.5    0.0
					 0.1913 -0.4619 0.0
					 0.3535 -0.3535 0.0
					 0.4619 -0.1913 0.0

	    MDH_Parameters		-0.9  0.0 0.0 0.0
	    Initial_Joint_Velocity	 0.0
	    Joint_Limits		-1000.172 1000.172 
	    Joint_Limit_Spring_Constant  0.0
	    Joint_Limit_Damper_Constant  0.0

	    Actuator_Type		 0
	    Joint_Friction		 0.35
	}
    }

}

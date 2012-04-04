# DynaMechs V 4.0 ascii
# kurmet parameter file

Articulation {
	Name		"Articulation"
	Graphics_Model	""

	Position		3	3	0
	Orientation_Quat	0.5	0.5	0.5	0.5

	

	MobileBaseLink {
		Name		"Torso"
		Graphics_Model	"./humanoid_model/torso.xan"

		Mass			12.1
		Inertia			0.218	0.0027	0.0117
					0.0027	0.257	0.0048
					0.0117	0.0048	0.121
		Center_of_Gravity	0.038	0.0008	0.0849	
		Number_of_Contact_Points	0

		Position 0 .6 0
		Orientation_Quat  .707 .707 0 0
		Velocity 0 0 0 0 0 0
	}
	# End Mobile Base

	
	# Lower Body
	Branch {
		ZScrewTxLink {
			Name			"ZScrewTorso"
			ZScrew_Parameters	0.09	-3.14159265
		}
	
		# Right Leg
		Branch {
			
			SphericalLink {
				Name		"RightThigh"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	-0.0001	-0.0037	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link 0 0 -.09
				Initial_Joint_Angles	0 0 0
				Initial_Angular_Velocity	0 0 0
				Axes_Limits			0	0  0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Joint_Friction		0
			}
	
			RevoluteLink {
				Name		"RightShank"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	-0.0003	-0.0002	
				Number_of_Contact_Points	1
					Contact_Locations	 0.2500         0         0
	
	
				MDH_Parameters		0.25	0	0	0.0	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"RightAnkle1"
				Graphics_Model	"./humanoid_model/nullObj.xan"
	
				Mass			0
				Inertia			0 0 0
								0 0 0
								0 0 0
				Center_of_Gravity	0 	0	0	
				Number_of_Contact_Points	0	
	
				MDH_Parameters		0.25	0	0	0	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"RightAnkle2"
				Graphics_Model	"./humanoid_model/ankle.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	-0.0003	-0.0002	
				Number_of_Contact_Points	4
					Contact_Locations	 0.01         .05         .1
										 0.01         .05         -.05
										 0.01         -.05         .1
										 0.01         -.05         -.05
	
	
				MDH_Parameters		0	1.5708	0	0.0
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			# End Ankle
		}
		# End Right Leg
		
		# Left Leg
		Branch {
		
			SphericalLink {
				Name		"LeftThigh"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	-0.0001	-0.0015	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link 0 0 .09
					Initial_Joint_Angles	0 0 0
					Initial_Angular_Velocity	0 0 0
					Axes_Limits			0	0  0
					Joint_Limit_Spring_Constant	0
					Joint_Limit_Damper_Constant	0
					Joint_Friction		0
			}
	
			RevoluteLink {
				Name		"LeftShank"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	-0.0003	-0.0002	
				Number_of_Contact_Points	0
	
				MDH_Parameters		0.25	0	0	0	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"LeftAnkle1"
				Graphics_Model	"./humanoid_model/nullObj.xan"
	
				Mass			0
				Inertia			0 0 0
								0 0 0
								0 0 0
				Center_of_Gravity	0 	0	0	
				Number_of_Contact_Points	0	
	
				MDH_Parameters		0.25	0	0	0	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"LeftAnkle2"
				Graphics_Model	"./humanoid_model/ankle.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	-0.0003	-0.0002	
				Number_of_Contact_Points	4
					Contact_Locations	 0.01         .05         .1
										 0.01         .05         -.05
										 0.01         -.05         .1
										 0.01         -.05         -.05
	
				MDH_Parameters		0	1.5708	0	0.0
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			# End Left Shank
		}
		# End Left leg
	}
	#End Lower Body
	
	# Upper Body
	Branch {
	
		# Right Arm
		Branch {
			
			SphericalLink {
				Name		"RightArm"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	-0.0001	-0.0037	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link .4 0 0
				Initial_Joint_Angles	0 .5 2
				Initial_Angular_Velocity	0 0 0
				Axes_Limits			0	0  0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Joint_Friction		0
			}
	
			RevoluteLink {
				Name		"RightForearm"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	-0.0003	-0.0002	
				Number_of_Contact_Points	1
					Contact_Locations	 0.2500         0         0
	
	
				MDH_Parameters		0.25	0	0	-0.5	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			# End Forearm
		}
		# End Right Arm
		
		# Left Arm
		Branch {
		
			SphericalLink {
				Name		"LeftArm"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	-0.0001	-0.0015	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link .4 0 .18
					Initial_Joint_Angles	0 -.5 2
					Initial_Angular_Velocity	0 0 0
					Axes_Limits			0	0  0
					Joint_Limit_Spring_Constant	0
					Joint_Limit_Damper_Constant	0
					Joint_Friction		0
			}
	
			RevoluteLink {
				Name		"LeftForearm"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	-0.0003	-0.0002	
				Number_of_Contact_Points	1
				Contact_Locations	 0.2500         0         0
	
				MDH_Parameters		0.25	0	0	-.5	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			# End Left Forearm
		}
		# End Left Arm
	}
	#End Upper Body
}

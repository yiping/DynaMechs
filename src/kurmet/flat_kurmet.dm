# DynaMechs V 4.0 ascii
# kurmet parameter file

Articulation {
	Name		"Articulation"
	Graphics_Model	""

	Position		3	3	0
	Orientation_Quat	0.5	0.5	0.5	0.5
	Branch {
		RevoluteLink {
			Name		"Pillar"
			Graphics_Model	"./kurmet_model/pillar.xan"

			Mass			0.00001
			Inertia			0.00000001	0	0
						    0	0.00000001	0
						    0	0	0.00000001
			Center_of_Gravity	0	0	0	
			Number_of_Contact_Points	0

			MDH_Parameters		0	-1.5708	0.46	0	
			Initial_Joint_Velocity	0
			Joint_Limits			0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type		0
			Joint_Friction		0
		}

		RevoluteLink {
			Name		"Boom"
			Graphics_Model	"./kurmet_model/boom.xan"

			Mass			0.00000027
			Inertia			0.000001272	0	0
						    0	0.00000041	0
						    0	0	 0.000001272
			Center_of_Gravity	0	-2.08	0	
			Number_of_Contact_Points	0

			MDH_Parameters		0	1.5708	0	1.5708	
			Initial_Joint_Velocity	0
			Joint_Limits			0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type		0
			Joint_Friction		0
		}

		RevoluteLink {
			Name		"Torso"
			Graphics_Model	"./kurmet_model/flat_torso.xan"

			Mass			12.1
			Inertia			0.218	0.0027	0.0117
						0.0027	0.257	0.0048
						0.0117	0.0048	0.121
			Center_of_Gravity	0.038	0.0008	0	
			Number_of_Contact_Points	0

			MDH_Parameters		0	1.5708	2.08	0	
			Initial_Joint_Velocity	0
			Joint_Limits			0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type		0
			Joint_Friction		0
		}

		ZScrewTxLink {
			Name			"ZScrewTorso"
			ZScrew_Parameters	0.0	-1.5708
		}

		ZScrewTxLink {
			Name			"ZScrewPelvis"
			ZScrew_Parameters	0	-1.5708
		}

		Branch {
			RevoluteLink {
				Name		"RightThigh"
				Graphics_Model	"./kurmet_model/thigh.xan"

				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	-0.0001	-0.0037	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	

				MDH_Parameters		0	0	-0.00	-0.22	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}

			RevoluteLink {
				Name		"RightShank"
				Graphics_Model	"./kurmet_model/shank.xan"

				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	-0.0003	-0.0002	
				Number_of_Contact_Points	1
			        Contact_Locations	 0.2500         0         0


				MDH_Parameters		0.25	0	0	0.1	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}

		}
		RevoluteLink {
			Name		"LeftThigh"
			Graphics_Model	"./kurmet_model/thigh.xan"

			Mass			0.81
			Inertia			0.00039	-1.6e-005	-0.00021
						-1.6e-005	0.0127	0
						-0.00021	0	0.0127
			Center_of_Gravity	0.0784	-0.0001	-0.0015	
			Number_of_Contact_Points	0

			MDH_Parameters		0	0	0.0	0.1	
			Initial_Joint_Velocity	0
			Joint_Limits			0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type		0
			Joint_Friction		0
		}

		RevoluteLink {
			Name		"LeftShank"
			Graphics_Model	"./kurmet_model/shank.xan"

			Mass			0.63
			Inertia			0.000238	-2e-006	-1e-006
						-2e-006	0.0116	0
						-1e-006	0	0.0116
			Center_of_Gravity	0.0964	-0.0003	-0.0002	
			Number_of_Contact_Points	1
			Contact_Locations	 0.2500         0         0

			MDH_Parameters		0.25	0	0	0.1	
			Initial_Joint_Velocity	0
			Joint_Limits			0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type		0
			Joint_Friction		0
		}

	}
}

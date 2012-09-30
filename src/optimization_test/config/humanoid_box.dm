# DynaMechs V 4.0 ascii
# humanoid parameter file

Articulation {
	Name				"Articulation"
	Graphics_Model		""
	Position			0	0	0
	Orientation_Quat	0	0	0	1

	MobileBaseLink {
		Name	"Torso"
		Graphics_Model	"./humanoid_box_model/torso.xan"
		Mass				11.051360
		Inertia				0.643093	0.000000	0.000000
							0.000000	0.748940	0.000000
							0.000000	0.000000	0.123410
		Center_of_Gravity	0.000000	0.000000	0.232848
		Number_of_Contact_Points	0
		Position	2.000000	2.000000	0.620000
		#Position	0.500000	.5000000	0.512
		#Orientation_Quat	0.000000	0.000000	0.3827	0.9239
		Orientation_Quat	0.000000	0.000000	0 1
		Velocity 0 0 0 0 0 0
	}
	# End "Torso"

	Branch {

		ZScrewTxLink {
			Name	"ZScrewTorso"
			ZScrew_Parameters	0.000000	0.000000
		}
		# End "ZScrewTorso"

		Branch {

			QuaternionLink {
				Name	"RightThigh"
				Graphics_Model	"./humanoid_box_model/thigh.xan"
				Mass				1.912000
				Inertia				0.001985	0.000000	0.000000
									0.000000	0.040649	0.000000
									0.000000	0.000000	0.040913
				Center_of_Gravity	0.120028	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.277200	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	-0.072975	0.030450
				Orientation_Quat	-0.547419	0.447585	0.447585	0.547419
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "RightThigh"

			RevoluteLink {
				Name	"RightShank"
				Graphics_Model	"./humanoid_box_model/shank.xan"
				Mass				0.889080
				Inertia				0.000550	0.000000	0.000000
									0.000000	0.015438	0.000000
									0.000000	0.000000	0.015438
				Center_of_Gravity	0.108661	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.250950	0.000000	0.000000
				MDH_Parameters	0.277200	0.000000	0.000000	0.500000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "RightShank"

			RevoluteLink {
				Name	"RightAnkle1"
				Graphics_Model	"./humanoid_box_model/ankle.xan"
				Mass				0.000000
				Inertia				0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
				Center_of_Gravity	0.000000	0.000000	0.000000
				Number_of_Contact_Points	0
				MDH_Parameters	0.250950	0.000000	0.000000	-0.300000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "RightAnkle1"

			RevoluteLink {
				Name	"RightAnkle2"
				Graphics_Model	"./humanoid_box_model/foot.xan"
				Mass				0.277240
				Inertia				0.001227	0.000000	-0.000174
									0.000000	0.001122	0.000000
									-0.000174	0.000000	0.000289
				Center_of_Gravity	0.015750	0.000000	0.039900
				Number_of_Contact_Points	4
				Contact_Locations	0.031500	0.046200	-0.039900
									0.031500	-0.046200	-0.039900
									0.031500	0.046200	0.119700
									0.031500	-0.046200	0.119700
				MDH_Parameters	0.000000	1.570796	0.000000	0.000000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "RightAnkle2"
		}
		# End Branch

		Branch {

			QuaternionLink {
				Name	"LeftThigh"
				Graphics_Model	"./humanoid_box_model/thigh.xan"
				Mass				1.912000
				Inertia				0.001985	0.000000	0.000000
									0.000000	0.040649	0.000000
									0.000000	0.000000	0.040913
				Center_of_Gravity	0.120028	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.277200	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	0.072975	0.030450
				Orientation_Quat	-0.547419	0.447585	0.447585	0.547419
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "LeftThigh"

			RevoluteLink {
				Name	"LeftShank"
				Graphics_Model	"./humanoid_box_model/shank.xan"
				Mass				0.889080
				Inertia				0.000550	0.000000	0.000000
									0.000000	0.015438	0.000000
									0.000000	0.000000	0.015438
				Center_of_Gravity	0.108661	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.250950	0.000000	0.000000
				MDH_Parameters	0.277200	0.000000	0.000000	0.500000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "LeftShank"

			RevoluteLink {
				Name	"LeftAnkle1"
				Graphics_Model	"./humanoid_box_model/ankle.xan"
				Mass				0.000000
				Inertia				0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
				Center_of_Gravity	0.000000	0.000000	0.000000
				Number_of_Contact_Points	0
				MDH_Parameters	0.250950	0.000000	0.000000	-0.300000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "LeftAnkle1"

			RevoluteLink {
				Name	"LeftAnkle2"
				Graphics_Model	"./humanoid_box_model/foot.xan"
				Mass				0.277240
				Inertia				0.001227	0.000000	-0.000174
									0.000000	0.001122	0.000000
									-0.000174	0.000000	0.000289
				Center_of_Gravity	0.015750	0.000000	0.039900
				Number_of_Contact_Points	4
				Contact_Locations	0.031500	0.046200	-0.039900
									0.031500	-0.046200	-0.039900
									0.031500	0.046200	0.119700
									0.031500	-0.046200	0.119700
				MDH_Parameters	0.000000	1.570796	0.000000	0.000000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "LeftAnkle2"
		}
		# End Branch
	}
	# End Branch

	Branch {

		QuaternionLink {
			Name	"RightArm"
			Graphics_Model	"./humanoid_box_model/upperArm.xan"
			Mass				0.535360
			Inertia				0.000309	0.000000	0.000000
								0.000000	0.003730	0.000000
								0.000000	0.000000	0.003648
			Center_of_Gravity	0.067754	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.155400	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	-0.107625	0.337050
			Orientation_Quat	0.411088	0.481323	-0.588681	0.502781
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "RightArm"

		RevoluteLink {
			Name	"RightForeArm"
			Graphics_Model	"./humanoid_box_model/foreArm.xan"
			Mass				0.420640
			Inertia				0.000157	0.000000	0.000000
								0.000000	0.014740	0.000000
								0.000000	0.000000	0.014719
			Center_of_Gravity	0.171864	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.252000	0.000000	0.000000
			MDH_Parameters	0.155400	0.000000	0.000000	0.500000
			Initial_Joint_Velocity	0
			Joint_Limits	0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type	0
			Joint_Friction	0
		}
		# End "RightForeArm"
	}
	# End Branch

	Branch {

		QuaternionLink {
			Name	"LeftArm"
			Graphics_Model	"./humanoid_box_model/upperArm.xan"
			Mass				0.535360
			Inertia				0.000309	0.000000	0.000000
								0.000000	0.003730	0.000000
								0.000000	0.000000	0.003648
			Center_of_Gravity	0.067754	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.155400	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	0.107625	0.337050
			Orientation_Quat	0.502781	0.588681	-0.481323	0.411088
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "LeftArm"

		RevoluteLink {
			Name	"RightArm"
			Graphics_Model	"./humanoid_box_model/foreArm.xan"
			Mass				0.420640
			Inertia				0.000157	0.000000	0.000000
								0.000000	0.014740	0.000000
								0.000000	0.000000	0.014719
			Center_of_Gravity	0.171864	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.252000	0.000000	0.000000
			MDH_Parameters	0.155400	0.000000	0.000000	0.500000
			Initial_Joint_Velocity	0
			Joint_Limits	0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type	0
			Joint_Friction	0
		}
		# End "RightArm"
	}
	# End Branch
}
# End Articulation


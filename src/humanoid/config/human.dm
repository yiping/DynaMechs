# DynaMechs V 4.0 ascii
# humanoid parameter file

Articulation {
	Name				"Articulation"
	Graphics_Model		""
	Position			0	0	0
	Orientation_Quat	0	0	0	1

	MobileBaseLink {
		Name	"Torso"
		Graphics_Model	"./human_model/torso.xan"
		Mass				40.460000
		Inertia				5.315424	0.000000	0.000000
							0.000000	6.122224	0.000000
							0.000000	0.000000	0.933470
		Center_of_Gravity	0.000000	0.000000	0.335286
		Number_of_Contact_Points	0
		Position	2.000000	2.000000	1.763918
		Orientation_Quat	0.000000	0.000000	0.000000	1.000000
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
				Graphics_Model	"./human_model/thigh.xan"
				Mass				7.000000
				Inertia				0.006809	0.000000	0.000000
									0.000000	0.358413	0.000000
									0.000000	0.000000	0.356507
				Center_of_Gravity	0.187125	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.432160	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	-0.168454	0.000000
				Orientation_Quat	0.000000	0.000000	0.000000	1.000000
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "RightThigh"

			RevoluteLink {
				Name	"RightShank"
				Graphics_Model	"./human_model/shank.xan"
				Mass				3.255000
				Inertia				0.003192	0.000000	0.000000
									0.000000	0.168026	0.000000
									0.000000	0.000000	0.167132
				Center_of_Gravity	0.187889	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.433924	0.000000	0.000000
				MDH_Parameters	0.432160	0.000000	0.000000	0.000000
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
				Graphics_Model	"./human_model/ankle.xan"
				Mass				0.000000
				Inertia				0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
				Center_of_Gravity	0.000000	0.000000	0.000000
				Number_of_Contact_Points	0
				MDH_Parameters	0.433924	0.000000	0.000000	0.000000
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
				Graphics_Model	"./human_model/foot.xan"
				Mass				1.015000
				Inertia				0.011437	0.000000	-0.002340
									0.000000	0.012242	0.000000
									-0.002340	0.000000	0.002397
				Center_of_Gravity	0.034396	0.000000	0.067029
				Number_of_Contact_Points	4
				Contact_Locations	0.068793	0.048508	-0.067029
									0.068793	-0.048508	-0.067029
									0.068793	0.048508	0.201087
									0.068793	-0.048508	0.201087
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
				Graphics_Model	"./human_model/thigh.xan"
				Mass				7.000000
				Inertia				0.006809	0.000000	0.000000
									0.000000	0.358413	0.000000
									0.000000	0.000000	0.356507
				Center_of_Gravity	0.187125	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.432160	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	0.168454	0.000000
				Orientation_Quat	0.000000	0.000000	0.000000	1.000000
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "LeftThigh"

			RevoluteLink {
				Name	"LeftShank"
				Graphics_Model	"./human_model/shank.xan"
				Mass				3.255000
				Inertia				0.003192	0.000000	0.000000
									0.000000	0.168026	0.000000
									0.000000	0.000000	0.167132
				Center_of_Gravity	0.187889	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.433924	0.000000	0.000000
				MDH_Parameters	0.432160	0.000000	0.000000	0.000000
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
				Graphics_Model	"./human_model/ankle.xan"
				Mass				0.000000
				Inertia				0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
				Center_of_Gravity	0.000000	0.000000	0.000000
				Number_of_Contact_Points	0
				MDH_Parameters	0.433924	0.000000	0.000000	0.000000
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
				Graphics_Model	"./human_model/foot.xan"
				Mass				1.015000
				Inertia				0.011437	0.000000	-0.002340
									0.000000	0.012242	0.000000
									-0.002340	0.000000	0.002397
				Center_of_Gravity	0.034396	0.000000	0.067029
				Number_of_Contact_Points	4
				Contact_Locations	0.068793	0.048508	-0.067029
									0.068793	-0.048508	-0.067029
									0.068793	0.048508	0.201087
									0.068793	-0.048508	0.201087
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
			Graphics_Model	"./human_model/upperArm.xan"
			Mass				1.960000
			Inertia				0.001099	0.000000	0.000000
								0.000000	0.058083	0.000000
								0.000000	0.000000	0.058391
			Center_of_Gravity	0.143047	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.328089	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	-0.228427	0.508008
			Orientation_Quat	0.000000	0.000000	0.000000	1.000000
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "RightArm"

		RevoluteLink {
			Name	"RightForeArm"
			Graphics_Model	"./human_model/foreArm.xan"
			Mass				1.540000
			Inertia				0.000350	0.000000	0.000000
								0.000000	0.169639	0.000000
								0.000000	0.000000	0.169804
			Center_of_Gravity	0.305560	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.448035	0.000000	0.000000
			MDH_Parameters	0.328089	0.000000	0.000000	0.000000
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
			Graphics_Model	"./human_model/upperArm.xan"
			Mass				1.960000
			Inertia				0.001099	0.000000	0.000000
								0.000000	0.058083	0.000000
								0.000000	0.000000	0.058391
			Center_of_Gravity	0.143047	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.328089	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	0.228427	0.508008
			Orientation_Quat	0.000000	0.000000	0.000000	1.000000
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "LeftArm"

		RevoluteLink {
			Name	"RightArm"
			Graphics_Model	"./human_model/foreArm.xan"
			Mass				1.540000
			Inertia				0.000350	0.000000	0.000000
								0.000000	0.169639	0.000000
								0.000000	0.000000	0.169804
			Center_of_Gravity	0.305560	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.448035	0.000000	0.000000
			MDH_Parameters	0.328089	0.000000	0.000000	0.000000
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


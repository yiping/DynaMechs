# DynaMechs V 4.0 ascii
# crank parameter file

ClosedArticulation {
   Name 	     "slider_crank"
   Graphics_Model    ""
   Position	     1.375 2.25 0.5
   Orientation_Quat  0.0 0.0 0.0 1.0

   TreeStructure {
      StaticRootLink {
         Name                "base"
         Graphics_Model      "../models/crank_base.xan"
      }

      ZScrewTxLink {
         Name                ""
         ZScrew_Parameters   0.275  0.0
      }

      RevoluteLink {
	 Name "crank"
	 Graphics_Model                 "../models/crank_wheel.xan"

	 Mass			     	2.0
	 Inertia		        0.02167   0.0      0.0
					0.0       0.02167  0.0
					0.0       0.0      0.04
	 Center_of_Gravity		0.0 0.0 0.0

	 Number_of_Contact_Points	0

	 MDH_Parameters		 	0.25 1.5708 0.15 0.0
	 Initial_Joint_Velocity	 	5.0
	 Joint_Limits			-1e10 1e10
	 Joint_Limit_Spring_Constant  	0.0
	 Joint_Limit_Damper_Constant  	0.0

	 Actuator_Type		 	0
	 Joint_Friction		 	0.001
      }

      SphericalLink {
      	 Name "linkage"
	 Graphics_Model                 "../models/crank_linkage.xan"

	 Mass                     	0.25
	 Inertia                  	0.000078  0.0       0.0
				        0.0       0.03004   0.0
	                                0.0       0.0       0.03004
	 Center_of_Gravity        	0.3 0.0 0.0

	 Number_of_Contact_Points	0

	 Position_From_Inboard_Link   	0.2 0.0 0.075
	 Initial_Joint_Angles         	0.0 0.0 0.0
	 Initial_Angular_Velocity     	0.0 0.0 0.0
	 Axes_Limits                  	0.0 0.0  0.0
	 Joint_Limit_Spring_Constant  	0.0
	 Joint_Limit_Damper_Constant  	0.0
	 Joint_Friction		     	0.0
      }


      SphericalLink {
	 Name "piston"
	 Graphics_Model                 "../models/crank_piston.xan"

	 Mass                     	0.5
	 Inertia                  	0.0003063 0.0      0.0
			         	0.0       0.003903 0.0
	                                0.0       0.0      0.003903
	 Center_of_Gravity        	0.075 0.0 0.0

	 Number_of_Contact_Points	0

	 Position_From_Inboard_Link   	0.6 0.0 0.0
	 Initial_Joint_Angles         	0.0 0.0 0.0
	 Initial_Angular_Velocity     	0.0 0.0 0.0
	 Axes_Limits                  	0.0 0.0  0.0
	 Joint_Limit_Spring_Constant  	0.0
	 Joint_Limit_Damper_Constant  	0.0
	 Joint_Friction		     	0.0
      }
   }

   SecondaryJoints {
     HardPrismaticJoint {
	 Name 		"secPrisJnt_1"
	 Stabilization  BAUMGARTE

         Link_A_Name    "base"
         Link_B_Name    "piston"

         Joint_A_Position      0.0 -0.5  0.5
	 Rotation_Matrix_A     0.0  0.0  1.0
		               0.0  1.0  0.0
		              -1.0  0.0  0.0

	 Joint_B_Position      0.0  0.0  0.0
         Rotation_Matrix_B     0.0  0.0  1.0
			      -1.0  0.0  0.0
			       0.0 -1.0  0.0

         Joint_Friction        0.0

	 Position_Constraint_Spring        100.0
	 Position_Constraint_Damper         20.0

	 Orientation_Constraint_Spring     100.0
	 Orientation_Constraint_Damper      20.0
      }
   }
}

# not parsed -- can replace the hard constraint above

   SecondaryJoints {
     SoftPrismaticJoint {
	 Name 		"secPrisJnt_1"

         Link_A_Name    "base"
         Link_B_Name    "piston"

         Joint_A_Position      0.0 -0.5  0.5
	 Rotation_Matrix_A     0.0  0.0  1.0
		               0.0  1.0  0.0
		              -1.0  0.0  0.0

	 Joint_B_Position      0.0  0.0  0.0
         Rotation_Matrix_B     0.0  0.0  1.0
			      -1.0  0.0  0.0
			       0.0 -1.0  0.0

         Joint_Friction        0.0

	 Position_Constraint_Spring       500.0
	 Position_Constraint_Damper        25.0

	 Orientation_Constraint_Spring     10.0
	 Orientation_Constraint_Damper      1.0
      }
   }

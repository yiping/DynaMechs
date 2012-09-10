<?php
function insert_tabs($count=1)
{
	$output = ' ';
	for($x = 1; $x <= $count; $x++)
	{
	   $output .= "\t";
	}
    return $output;
}



function Print_Matrix($c)
{
	$rows = count($c);	//echo $rows;
	$cols = count($c[0]);	//echo $cols;
	for ($i=0; $i<$rows; $i++)
	{
		if ($cols>1)
		{
			for($j=0;$j<$cols;$j++)
			{
				echo insert_tabs(1).sprintf("%f",$c[$i][$j]);
			}
			echo "\n";
		}
		else
		{
			echo insert_tabs(1).sprintf("%f",$c[$i]);
			echo "\n";
		}
	}
	echo "\n";
}



$RotationMatrices = array(
"leftHip" => array( array(0,1,0), array(-1, 0,0), array(0, 0, 1) ),
"leftHipPitchRoll" => array( array(0,1,0), array(0,0,1), array(1,0,0) ),
"leftKneeUpper" => array( array(0,0,-1), array(-1,0,0), array(0,1,0) ),
"leftKneeLower" => array( array(0,0,-1), array(-1,0,0), array(0,1,0) ),
"leftAnklePitch" => array( array(0,0,-1), array(-1,0,0), array(0,1,0) ),
"leftFoot" => array( array(0,0,-1), array(0,1,0), array(1,0,0)  ),
);

var_dump($RotationMatrices["leftHip"]);
Print_Matrix($RotationMatrices["leftHip"]);
$BodyName = "leftHip";
var_dump($BodyName);
echo $BodyName."\n";
Print_Matrix($RotationMatrices[$BodyName]);


?>

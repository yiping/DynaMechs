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

function Matrix_Add($a,$b) //Matrix Add
{
	$m=count($a);
	$n=count($a[0]);
	$m1=count($b);
	$n1=count($b[0]);
	if(($m!=$m1)||($n!=$n1))
		exit( "\nError: Matrix_Add(): matrix dimension NOT matched !!!!!!\n\n\n");
	else
	{
		for($i=0;$i<$m;$i++)
		{
			if ($n > 1)
			{
	   			for($j=0;$j<$n;$j++)
	   			{
					$c[$i][$j]=$a[$i][$j]+$b[$i][$j];
					//echo $c[$i][$j]." ";
	   			}
			}
			else
			{
				$c[$i]=$a[$i]+$b[$i];
			}

		}
		return $c;

	}
}

function Print_Matrix($c)
{
	echo "\n";
	$rows = count($c);	//echo $rows;
	$cols = count($c[0]);	//echo $cols;
	for ($i=0; $i<$rows; $i++)
	{
		if ($cols>1)
		{
			for($j=0;$j<$cols;$j++)
			{
				echo insert_tabs(1).$c[$i][$j];
			}
			echo "\n";
		}
		else
		{
			echo insert_tabs(1).$c[$i];
			echo "\n";
		}
	}
	echo "\n";
}

function Matrix_Mul($a,$b) //Matrix_Multiply
{
	$k=count($a[0]);  //a_cols
	$k1=count($b);    //b_rows

	$m=count($a);	//a_rows
	$n=count($b[0]);	//b_cols

	if($k!=$k1)
		exit("\nError: Matrix_Mul: matrix dimension NOT matched !!!!!!\n\n\n");
	else
	{

		for($i=0;$i<$m;$i++) 	// i - a_row
		{
			if ($n >1)
			{ 
				for($j=0;$j<$n;$j++)	// j - b_col
			   	{  
					$c[$i][$j]=0;
					for($l=0;$l<$k;$l++)
					{
				 		$c[$i][$j]+=$a[$i][$l]*$b[$l][$j];
					}
			   	}
			}
			else //if b has only one column
			{
				for($j=0;$j<$n;$j++)	// j - b_col
			   	{  
					$c[$i]=0;
					for($l=0;$l<$k;$l++)
					{
				 		$c[$i]+=$a[$i][$l]*$b[$l];
					}
			   	}				
			}
   
		}

		return $c;
	}
}

function Transpose($a) 
{
	$m=count($a);   // rows
	$n=count($a[0]);	// cols

	if ($m>1)
	{
		for($i=0;$i<$m;$i++)
		{
			if ($n >1)
			{	
				for($j=0;$j<$n;$j++)
				{
		  	 		$b[$j][$i]=$a[$i][$j];
				}  	 
			}
			else
			{
				$b[0][$i]=$a[$i];
			}
		}
	}
	else // if transpose a 1 x n matrix
	{
		for($j=0;$j<$n;$j++)
		{
  	 		$b[$j]=$a[0][$j];
		}  	 
	}
	return $b;
}


function Scalar_Mul($a, $M)
{
	$rows = count($M);	// rows;
	$cols = count($M[0]);	//$cols;

	for($i=0;$i<$rows;$i++)
	{
		if ($cols > 1)
		{
			for($j=0;$j<$cols;$j++)
			{
				$aM[$i][$j]= $a * $M[$i][$j];
			}
		}
		else
		{
			$aM[$i]= $a * $M[$i];
		}
	}
	return $aM;

}


function Cross($x)
{
	$rows = count($x);	// rows;
	$cols = count($x[0]);	//cols;

	if ($rows==3 && $cols == 1)
	{
		Print_Matrix($x);
		echo "rows= ".$rows."   cols=".$cols." \n";
	   	$M = array(array(0, 	-$x[2], 	$x[1]),
				   array($x[2], 	0, 		-$x[0]),
				   array(-$x[1], $x[0], 	0)		);
	}
	elseif ($rows==1 && $cols == 3)
	{
		Print_Matrix($x);
		echo "rows= ".$rows."   cols=".$cols." \n";
	   	$M = array(array(0, 	-$x[0][2], 	$x[0][1]),
				   array($x[0][2], 	0, 		-$x[0][0]),
				   array(-$x[0][1], $x[0][0], 	0)		);		
	}
	else
	{
		exit("\nError: Cross: matrix dimension WRONG !!!!!!\n\n\n");
	}
	return $M; 
}

// debugging 

$a=array(0=>array(0,2,3,4),1=>array(1.2,3.2,3.2,1.1),2=>array(4.3,5.4,2.7,3.1),3=>array(1.5,6,7,5));
$b=array(0=>array(1,2,3,4),1=>array(1.2,3.2,3.2,1.1),2=>array(4.3,5.4,2.7,3.1),3=>array(1.5,6,7,5));
$x = array(array(1,2,3));
$x1 = array(2,2,2);
$y = array(array(4,5,7));
//$x = array(1,2,3);
//$y = array(3,4,5);
//echo count($x)."\n";
//echo count($x[0])."\n";
$c= Matrix_Add($a,$b);
$z= Matrix_Add($x,$y);
$z1 = Matrix_Mul($x,$x1);
//$z2 = Matrix_Mul($a,$b);
Print_Matrix($x);
Print_Matrix($c);
Print_Matrix($z);
var_dump($z1);
Print_Matrix($z1);
Print_Matrix($a);
Print_Matrix($b);
//var_dump($z1);
//var_dump($c);

$a1=array(0=>array(2,1));//  ,1=>array(1,1)
$a2=array(2,1);
$b1=array(0=>array(1,3),1=>array(2,7));
$c1=Matrix_Mul($a1,$b1);
$c2=Matrix_Mul($b1,$a2);

echo "a1 = ";
Print_Matrix($a1);
echo "b1 = ";
Print_Matrix($b1);
echo "a2 = ";
Print_Matrix($a2);
echo "c1 = a1*b1 = ";
Print_Matrix($c1);
echo "c2 = b1*a2 = ";
Print_Matrix($c2);

echo "------------------\n ";
$u=array(1.1, 2.2, 3.3);
$v=array(array(5,6,7));

$u_trans = Transpose($u);
$v_trans = Transpose($v);
echo "u = ";
Print_Matrix($u);
echo "v = ";
Print_Matrix($v);
echo "uT = ";
Print_Matrix($u_trans);
echo "vT = ";
Print_Matrix($v_trans);

$vv=array(array(5,6,7),array(4, 3,2));
$vvT = Transpose($vv);
echo "vv = ";
Print_Matrix($vv);
echo "vvT = ";
Print_Matrix($vvT);

echo "a = ";
Print_Matrix($a);$aT = Transpose($a);
echo "aT = ";
Print_Matrix($aT);

echo "------------------\n ";

$e = $x;
$e1 = $x1;
$em = Scalar_Mul(2,$e);
$em1 = Scalar_Mul(1.5, $e1);
echo "e = ";
Print_Matrix($e);
echo "e1 = ";
Print_Matrix($e1);

echo "2*e = ";
Print_Matrix($em);
echo "1.5 * e1 = ";
Print_Matrix($em1);

echo "-----------------\n";
Print_Matrix(Cross($x1));
Print_Matrix(Cross($x));

?>

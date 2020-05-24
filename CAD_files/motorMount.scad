$fn=100;
e=0.001;

dist = 70;

union(){
   translate([dist/2,0,0])
      motor();
   translate([-dist/2,0,0])
      motor();
   translate([0,0,1])
   cube([dist-10.5,15,2],center=true);
}



module motor(){
   wall = 1.5;
   depth = 10;
   translate([0,0,depth/2+wall/2])
   difference(){
      cube([10,12,depth]+[2*wall,2*wall,wall],center=true);
      translate([0,0,wall/2+e])
	 cube([10.25,12.25,depth],center=true);
      cylinder(d=4.5, h=4*depth, center=true);
   }
}

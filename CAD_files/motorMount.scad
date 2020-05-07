$fn=100;
e=0.001;

dist = 100;

union(){
   translate([dist/2,0,0])
      motor();
   translate([-dist/2,0,0])
      motor();
   translate([0,0,1])
   cube([dist-10,16,2],center=true);
}



module motor(){
   wall = 2;
   translate([0,0,5+wall/2])
   difference(){
      cube([10,12,10]+[2*wall,2*wall,wall],center=true);
      translate([0,0,wall/2+e])
	 cube([10,12,10],center=true);
      cylinder(d=4, h=20, center=true);
   }
}



$fn=200;

diameter = 50;
lip_dia = 5;
lip_thick = 2;
width = 9;

pulley();

/* difference(){ */
/*    cylinder(d=15,h=5); */
/*    shaft(); */
/*    /\* lead-in's *\/ */
/*    translate([0,0,-0.1]) */
/*       cylinder(d1=4, d2=3,h=1.1); */

/*    } */

module pulley(){
   difference(){
      union(){
	 difference(){
	    union(){
	       cylinder(d=diameter, h=width);
	       cylinder(d=diameter+2*lip_dia, h=lip_thick);
	    }

	    cutouts();

	    /* lead-in's */
	    translate([0,0,width-1])
	       cylinder(d1=3, d2=4,h=1.1);
	    translate([0,0,-0.1])
	       cylinder(d1=4, d2=3,h=1.1);

	    /* Hole for thread */
	    translate([0,-diameter/3,width/2])rotate([90,0,0])
	       cylinder(r=.75,h=diameter);

	 }
	 cylinder(d=diameter/4, h=width);
      }
      shaft();
   }
}
module cutouts() {
   for(i=[0:60:360]) {
      rotate([0,0,i])hull(){
	 translate([0,diameter/8,0])
	    cylinder(d=diameter/10,h=width*3,center=true);
	 translate([0,diameter/3,0])
	    cylinder(d=diameter/3.5,h=width*3,center=true);
      }
   }
}

module shaft() {
   scale([1.05,1.05,1.05])
   intersection(){
      cylinder(d=3.2, h=width*3, center=true);
      translate([0,0.25,0])
	 cube([5,2.5,width*3],center=true);
	 }
}

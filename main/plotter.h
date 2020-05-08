
#ifndef __PLOTTER_H__
#define __PLOTTER_H__


/* 
   This is all brainstorming right now, it wouldn't even compile if I tried.
 */




/* 
need stuff for hardware config,
pin out,
counts to mm (gear ratio + circumference),
distance between pulley?,
distance between anchor?,

 */

struct PlotterConfig {
   /* dist between pulley, dist between anchor, counts to mm */

   /* velocity, accel */

};

struct Plotter{
   /* 
      left/right current count and target

      config data:
      velocity, acceleration, motor spacing

    */
};

void plotterLoop() {
   while(1){
      
      /* I don't like this vvv */
      
      /* calculate next setpoint */
      calcNextPoint( &pos0, &pos1);

      
      /* update motor setpoint */
      setMotorPosition( pos0, pos1);



      /* The gcode portion could also be replaced with other methods of
	 describing location (such as lissajous, traveling salesman, etc.) */

      /* TODO: do I want to use queues or something to link the different portions? */
      
      for(gcode command) {	/* 0: move to (x0,y0), 1: line to (x1,y1), etc.... */
	 /* will have type of command (line, arc, move, etc.) and parameters */

	 /* given type of command, interpolate */
	 /* TODO: what to do if not meeting target??? */
	 switch(gcode type) {
	    case LINE:
	       moveline( data, ....... , period(in mS) );
	       break;
	    case MOVE:
	       break;
	    case ARC1:
	       break;
	    default:
	       break;
	 }
	 while(interpolate) {
	    setMotorPosition( pos0, pos1);
	 }
      }

#endif /* __PLOTTER_H__ */
